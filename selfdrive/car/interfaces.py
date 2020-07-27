import os
import time
from cereal import car, arne182
from common.kalman.simple_kalman import KF1D
from common.realtime import DT_CTRL
from selfdrive.car import gen_empty_fingerprint
from selfdrive.controls.lib.events import Events, Events_arne182
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX
from selfdrive.controls.lib.vehicle_model import VehicleModel
import cereal.messaging as messaging
from common.op_params import opParams
from common.travis_checker import travis
from selfdrive.config import Conversions as CV


GearShifter = car.CarState.GearShifter
EventName = car.CarEvent.EventName
EventNameArne182 = arne182.CarEventArne182.EventNameArne182
MAX_CTRL_SPEED = (V_CRUISE_MAX + 4) * CV.KPH_TO_MS  # 144 + 4 = 92 mph

# generic car and radar interfaces

class CarInterfaceBase():
  def __init__(self, CP, CarController, CarState):
    self.waiting = False
    self.keep_openpilot_engaged = True
    self.disengage_due_to_slow_speed = False
    self.sm = messaging.SubMaster(['pathPlan'])
    self.op_params = opParams()
    self.alca_min_speed = self.op_params.get('alca_min_speed', default=20.0)
    self.CP = CP
    self.VM = VehicleModel(CP)

    self.frame = 0
    self.low_speed_alert = False

    if CarState is not None:
      self.CS = CarState(CP)
      self.cp = self.CS.get_can_parser(CP)
      self.cp_cam = self.CS.get_cam_can_parser(CP)

    self.CC = None
    if CarController is not None:
      self.CC = CarController(self.cp.dbc_name, CP, self.VM)

  @staticmethod
  def calc_accel_override(a_ego, a_target, v_ego, v_target):
    return 1.

  @staticmethod
  def compute_gb(accel, speed):
    raise NotImplementedError

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=[]):  # pylint: disable=dangerous-default-value
    raise NotImplementedError

  # returns a set of default params to avoid repetition in car specific params
  @staticmethod
  def get_std_params(candidate, fingerprint, has_relay):
    ret = car.CarParams.new_message()
    ret.carFingerprint = candidate
    ret.isPandaBlack = has_relay

    # standard ALC params
    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.steerMaxBP = [0.]
    ret.steerMaxV = [1.]
    ret.minSteerSpeed = 0.

    # stock ACC by default
    ret.enableCruise = True
    ret.minEnableSpeed = -1.  # enable is done by stock ACC, so ignore this
    ret.steerRatioRear = 0.  # no rear steering, at least on the listed cars aboveA
    ret.gasMaxBP = [0.]
    ret.gasMaxV = [.5]  # half max brake
    ret.brakeMaxBP = [0.]
    ret.brakeMaxV = [1.]
    ret.openpilotLongitudinalControl = False
    ret.startAccel = 0.0
    ret.stoppingControl = False
    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.]
    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kpV = [1.]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [1.]
    return ret

  # returns a car.CarState, pass in car.CarControl
  def update(self, c, can_strings):
    raise NotImplementedError

  # return sendcan, pass in a car.CarControl
  def apply(self, c):
    raise NotImplementedError

  def create_common_events(self, cs_out, extra_gears=[], gas_resume_speed=-1, pcm_enable=True):
    if cs_out.cruiseState.enabled and not self.CS.out.cruiseState.enabled:  # this lets us modularize which checks we want to turn off op if cc was engaged previoiusly or not
      disengage_event = True
    else:
      if travis:
        disengage_event = True
      else:
        disengage_event = False

    events = Events()
    eventsArne182 = Events_arne182()

    if cs_out.doorOpen and disengage_event:
      events.add(EventName.doorOpen)
    if cs_out.seatbeltUnlatched and disengage_event:
      events.add(EventName.seatbeltNotLatched)
    if cs_out.gearShifter != GearShifter.drive and cs_out.gearShifter not in extra_gears:
      if cs_out.vEgo < 5:
        eventsArne182.add(EventNameArne182.wrongGearArne)
      else:
        events.add(EventName.wrongGear)
    if cs_out.gearShifter == GearShifter.reverse:
      if cs_out.vEgo < 5:
        eventsArne182.add(EventNameArne182.reverseGearArne)
      else:
        events.add(EventName.reverseGear)
    if not cs_out.cruiseState.available:
      events.add(EventName.wrongCarMode)
    if cs_out.espDisabled:
      events.add(EventName.espDisabled)
    if cs_out.gasPressed and disengage_event:
      events.add(EventName.gasPressed)
    if cs_out.stockFcw:
      events.add(EventName.stockFcw)
    if cs_out.stockAeb:
      events.add(EventName.stockAeb)
    if cs_out.vEgo > MAX_CTRL_SPEED:
      events.add(EventName.speedTooHigh)
    if cs_out.cruiseState.nonAdaptive:
      events.add(EventName.wrongCruiseMode)

    # TODO: move this stuff to the capnp strut
    if cs_out.steerError:
      events.add(EventName.steerUnavailable)
    elif cs_out.steerWarning:
      events.add(EventName.steerTempUnavailable)
    # Disable on rising edge of gas or brake. Also disable on brake when speed > 0.
    # Optionally allow to press gas at zero speed to resume.
    # e.g. Chrysler does not spam the resume button yet, so resuming with gas is handy. FIXME!
    if disengage_event and ((cs_out.gasPressed and (not self.CS.out.gasPressed) and cs_out.vEgo > gas_resume_speed) or \
       (cs_out.brakePressed and (not self.CS.out.brakePressed or not cs_out.standstill))):
      events.add(EventName.pedalPressed)

    # we engage when pcm is active (rising edge)
    if pcm_enable:
      if cs_out.cruiseState.enabled and not self.CS.out.cruiseState.enabled:
        events.add(EventName.pcmEnable)
      elif not cs_out.cruiseState.enabled:
        events.add(EventName.pcmDisable)

    return events, eventsArne182

class RadarInterfaceBase():
  def __init__(self, CP):
    self.pts = {}
    self.delay = 0
    self.radar_ts = CP.radarTimeStep

  def update(self, can_strings):
    ret = car.RadarData.new_message()

    if 'NO_RADAR_SLEEP' not in os.environ:
      time.sleep(self.radar_ts)  # radard runs on RI updates

    return ret

class CarStateBase:
  def __init__(self, CP):
    self.CP = CP
    self.car_fingerprint = CP.carFingerprint
    self.cruise_buttons = 0
    self.out = car.CarState.new_message()

    # Q = np.matrix([[10.0, 0.0], [0.0, 100.0]])
    # R = 1e3
    self.v_ego_kf = KF1D(x0=[[0.0], [0.0]],
                         A=[[1.0, DT_CTRL], [0.0, 1.0]],
                         C=[1.0, 0.0],
                         K=[[0.12287673], [0.29666309]])

  def update_speed_kf(self, v_ego_raw):
    if abs(v_ego_raw - self.v_ego_kf.x[0][0]) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_kf.x = [[v_ego_raw], [0.0]]

    v_ego_x = self.v_ego_kf.update(v_ego_raw)
    return float(v_ego_x[0]), float(v_ego_x[1])

  @staticmethod
  def parse_gear_shifter(gear):
    return {'P': GearShifter.park, 'R': GearShifter.reverse, 'N': GearShifter.neutral,
            'E': GearShifter.eco, 'T': GearShifter.manumatic, 'D': GearShifter.drive,
            'S': GearShifter.sport, 'L': GearShifter.low, 'B': GearShifter.brake}.get(gear, GearShifter.unknown)

  @staticmethod
  def get_cam_can_parser(CP):
    return None
