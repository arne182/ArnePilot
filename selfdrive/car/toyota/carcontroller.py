from cereal import car
import json
from common.params import Params
from common.numpy_fast import clip
from selfdrive.car import apply_toyota_steer_torque_limits, create_gas_command, make_can_msg, gen_empty_fingerprint
from selfdrive.car.toyota.toyotacan import create_steer_command, create_ui_command, \
                                           create_accel_command, create_acc_cancel_command, \
                                           create_fcw_command
from selfdrive.car.toyota.values import Ecu, CAR, STATIC_MSGS, SteerLimitParams, TSS2_CAR
from opendbc.can.packer import CANPacker
#from common.op_params import opParams
#import cereal.messaging as messaging

#op_params = opParams()

VisualAlert = car.CarControl.HUDControl.VisualAlert

# Accel limits
ACCEL_HYST_GAP = 0.02  # don't change accel command for small oscilalitons within this value
ACCEL_MAX = 3.5  # 3.5 m/s2
ACCEL_MIN = -3.5 # 3.5 m/s2
ACCEL_SCALE = max(ACCEL_MAX, -ACCEL_MIN)

# Blindspot codes
LEFT_BLINDSPOT = b'\x41'
RIGHT_BLINDSPOT = b'\x42'
BLINDSPOTALWAYSON = False

def set_blindspot_debug_mode(lr,enable):
  if enable:
    m = lr + b'\x02\x10\x60\x00\x00\x00\x00'
  else:
    m = lr + b'\x02\x10\x01\x00\x00\x00\x00'
  return make_can_msg(0x750, m, 0)


def poll_blindspot_status(lr):
  m = lr + b'\x02\x21\x69\x00\x00\x00\x00'
  return make_can_msg(0x750, m, 0)

def accel_hysteresis(accel, accel_steady, enabled):
  # for small accel oscillations within ACCEL_HYST_GAP, don't change the accel command
  if not enabled:
    # send 0 when disabled, otherwise acc faults
    accel_steady = 0.
  elif accel > accel_steady + ACCEL_HYST_GAP:
    accel_steady = accel - ACCEL_HYST_GAP
  elif accel < accel_steady - ACCEL_HYST_GAP:
    accel_steady = accel + ACCEL_HYST_GAP
  accel = accel_steady

  return accel, accel_steady


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.accel_steady = 0.
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.blindspot_blink_counter_left = 0
    self.blindspot_blink_counter_right = 0
    self.blindspot_debug_enabled_left = False
    self.blindspot_debug_enabled_right = False

    #self.sm = messaging.SubMaster(['pathPlan'])
    #self.rightLaneDepart = False
    #self.leftLaneDepart = False

    self.last_fault_frame = -200
    self.steer_rate_limited = False

    self.fake_ecus = set()
    if CP.enableCamera: self.fake_ecus.add(Ecu.fwdCamera)
    if CP.enableDsu: self.fake_ecus.add(Ecu.dsu)
    params = Params()
    try:
      cached_fingerprint = params.get('CachedFingerprint')
      vin = params.get('CarVin')
      finger = gen_empty_fingerprint()
      cached_fingerprint = json.loads(cached_fingerprint)
      finger[0] = {int(key): value for key, value in cached_fingerprint[2].items()}
      if 0x2FF in finger[0] and vin == b'JTMWRREV10D058569': self.fake_ecus.add(Ecu.unknown)
    except:
      pass

    self.packer = CANPacker(dbc_name)

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, hud_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart):
    #if not enabled:
    #  self.sm.update(0)
    #if self.sm.updated['pathPlan']:
    #  blinker = CS.out.leftBlinker or CS.out.rightBlinker
    #  ldw_allowed = CS.out.vEgo > 12.5 and not blinker
    #  CAMERA_OFFSET = op_params.get('camera_offset', 0.06)
    #  right_lane_visible = self.sm['pathPlan'].rProb > 0.5
    #  left_lane_visible = self.sm['pathPlan'].lProb > 0.5
    #  self.rightLaneDepart = bool(ldw_allowed and self.sm['pathPlan'].rPoly[3] > -(0.93 + CAMERA_OFFSET) and right_lane_visible)
    #  self.leftLaneDepart = bool(ldw_allowed and self.sm['pathPlan'].lPoly[3] < (0.93 - CAMERA_OFFSET) and left_lane_visible)
    #  print("blinker")
    #  print(blinker)
    #  print("ldw_allowed")
    #  print(ldw_allowed)
    #  print("CAMERA_OFFSET")
    #  print(CAMERA_OFFSET)
    #  print("right_lane_visible")
    #  print(right_lane_visible)
    #  print("left_lane_visible")
    #  print(left_lane_visible)
    #  print("self.rightLaneDepart")
    #  print(self.rightLaneDepart)
    #  print("self.leftLaneDepart")
    #  print(self.leftLaneDepart)
    # *** compute control surfaces ***

    # gas and brake

    apply_gas = clip(actuators.gas, 0., 1.)

    if CS.CP.enableGasInterceptor:
      # send only negative accel if interceptor is detected. otherwise, send the regular value
      # +0.06 offset to reduce ABS pump usage when OP is engaged
      apply_accel = 0.06 - actuators.brake
    else:
      apply_accel = actuators.gas - actuators.brake

    apply_accel, self.accel_steady = accel_hysteresis(apply_accel, self.accel_steady, enabled)
    apply_accel = clip(apply_accel * ACCEL_SCALE, ACCEL_MIN, ACCEL_MAX)

    if CS.CP.enableGasInterceptor:
      if CS.out.gasPressed:
        apply_accel = max(apply_accel, 0.06)
      if CS.out.brakePressed:
        apply_gas = 0.0
        apply_accel = min(apply_accel, 0.00)
    else:
      if CS.out.gasPressed:
        apply_accel = max(apply_accel, 0.0)
      if CS.out.brakePressed and CS.out.vEgo > 1:
        apply_accel = min(apply_accel, 0.0)

    # steer torque
    new_steer = int(round(actuators.steer * SteerLimitParams.STEER_MAX))

    # only cut torque when steer state is a known fault
    if CS.steer_state in [9, 25]:
      self.last_fault_frame = frame

    # Cut steering for 1s after fault
    if (frame - self.last_fault_frame < 100) or abs(CS.out.steeringRate) > 100 or (abs(CS.out.steeringAngle) > 400 and CS.CP.carFingerprint in [CAR.RAV4H, CAR.PRIUS]):
      new_steer = 0
      apply_steer_req = 0
    else:
      apply_steer_req = 1

    if not enabled and right_lane_depart and CS.out.vEgo > 12.5 and not CS.out.rightBlinker:
      new_steer = self.last_steer + 3
      new_steer = min(new_steer , 800)
      print ("right")
      print (new_steer)
      apply_steer_req = 1

    if not enabled and left_lane_depart and CS.out.vEgo > 12.5 and not CS.out.leftBlinker:
      new_steer = self.last_steer - 3
      new_steer = max(new_steer , -800)
      print ("left")
      print (new_steer)
      apply_steer_req = 1

    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, SteerLimitParams)
    self.steer_rate_limited = new_steer != apply_steer

    if not enabled and abs(apply_steer) > 800 and not (right_lane_depart or left_lane_depart):
      apply_steer = 0
      apply_steer_req = 0

    if not enabled and CS.pcm_acc_status:
      # send pcm acc cancel cmd if drive is disabled but pcm is still on, or if the system can't be activated
      pcm_cancel_cmd = 1

    # on entering standstill, send standstill request
    if CS.out.standstill and not self.last_standstill:
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      # pcm entered standstill or it's disabled
      self.standstill_req = False

    self.last_steer = apply_steer
    self.last_accel = apply_accel
    self.last_standstill = CS.out.standstill

    can_sends = []

    #*** control msgs ***
    #print("steer {0} {1} {2} {3}".format(apply_steer, min_lim, max_lim, CS.steer_torque_motor)

    # toyota can trace shows this message at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    if Ecu.fwdCamera in self.fake_ecus:
      can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req, frame))

      # LTA mode. Set ret.steerControlType = car.CarParams.SteerControlType.angle and whitelist 0x191 in the panda
      # if frame % 2 == 0:
      #   can_sends.append(create_steer_command(self.packer, 0, 0, frame // 2))
      #   can_sends.append(create_lta_steer_command(self.packer, actuators.steerAngle, apply_steer_req, frame // 2))

    # we can spam can to cancel the system even if we are using lat only control
    if (frame % 3 == 0 and CS.CP.openpilotLongitudinalControl) or (pcm_cancel_cmd and Ecu.fwdCamera in self.fake_ecus):
      lead = lead or CS.out.vEgo < 12.    # at low speed we always assume the lead is present do ACC can be engaged

      # Lexus IS uses a different cancellation message
      if pcm_cancel_cmd and CS.CP.carFingerprint == CAR.LEXUS_IS:
        can_sends.append(create_acc_cancel_command(self.packer))
      elif CS.CP.openpilotLongitudinalControl:
        can_sends.append(create_accel_command(self.packer, apply_accel, pcm_cancel_cmd, self.standstill_req, lead))
      else:
        can_sends.append(create_accel_command(self.packer, 0, pcm_cancel_cmd, False, lead))

    if (frame % 2 == 0) and (CS.CP.enableGasInterceptor):
      # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
      # This prevents unexpected pedal range rescaling
      can_sends.append(create_gas_command(self.packer, apply_gas, frame//2))

    # ui mesg is at 100Hz but we send asap if:
    # - there is something to display
    # - there is something to stop displaying
    fcw_alert = hud_alert == VisualAlert.fcw
    steer_alert = hud_alert == VisualAlert.steerRequired

    send_ui = False
    if ((fcw_alert or steer_alert) and not self.alert_active) or \
       (not (fcw_alert or steer_alert) and self.alert_active):
      send_ui = True
      self.alert_active = not self.alert_active
    elif pcm_cancel_cmd:
      # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
      send_ui = True

    if (frame % 100 == 0 or send_ui) and Ecu.fwdCamera in self.fake_ecus:
      can_sends.append(create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, left_line, right_line, left_lane_depart, right_lane_depart))

    if frame % 100 == 0 and (Ecu.dsu in self.fake_ecus or Ecu.unknown in self.fake_ecus):
      can_sends.append(create_fcw_command(self.packer, fcw_alert))

    #*** static msgs ***

    for (addr, ecu, cars, bus, fr_step, vl) in STATIC_MSGS:
      if frame % fr_step == 0 and ecu in self.fake_ecus and CS.CP.carFingerprint in cars:
        can_sends.append(make_can_msg(addr, vl, bus))

    # Enable blindspot debug mode once
    if frame > 1000 and not (CS.CP.carFingerprint in TSS2_CAR or CS.CP.carFingerprint == CAR.CAMRY): # 10 seconds after start and not a tss2 car
      if BLINDSPOTALWAYSON:
        self.blindspot_blink_counter_left += 1
        self.blindspot_blink_counter_right += 1
        #print("debug blindspot alwayson!")
      elif CS.out.leftBlinker:
        self.blindspot_blink_counter_left += 1
        #print("debug Left Blinker on")
      elif CS.out.rightBlinker:
        self.blindspot_blink_counter_right += 1
        #print("debug Right Blinker on")
      else:
        self.blindspot_blink_counter_left = 0
        self.blindspot_blink_counter_right = 0
        if self.blindspot_debug_enabled_left:
          can_sends.append(set_blindspot_debug_mode(LEFT_BLINDSPOT, False))
          #can_sends.append(make_can_msg(0x750, b'\x41\x02\x10\x01\x00\x00\x00\x00', 0))
          self.blindspot_debug_enabled_left = False
          #print ("debug Left blindspot debug disabled")
        if self.blindspot_debug_enabled_right:
          can_sends.append(set_blindspot_debug_mode(RIGHT_BLINDSPOT, False))
          #can_sends.append(make_can_msg(0x750, b'\x42\x02\x10\x01\x00\x00\x00\x00', 0))
          self.blindspot_debug_enabled_right = False
          #print("debug Right blindspot debug disabled")
      if self.blindspot_blink_counter_left > 9 and not self.blindspot_debug_enabled_left: #check blinds
        can_sends.append(set_blindspot_debug_mode(LEFT_BLINDSPOT, True))
        #can_sends.append(make_can_msg(0x750, b'\x41\x02\x10\x60\x00\x00\x00\x00', 0))
        #print("debug Left blindspot debug enabled")
        self.blindspot_debug_enabled_left = True
      if self.blindspot_blink_counter_right > 5 and not self.blindspot_debug_enabled_right: #enable blindspot debug mode
        if CS.out.vEgo > 6: #polling at low speeds switches camera off
          #can_sends.append(make_can_msg(0x750, b'\x42\x02\x10\x60\x00\x00\x00\x00', 0))
          can_sends.append(set_blindspot_debug_mode(RIGHT_BLINDSPOT, True))
          #print("debug Right blindspot debug enabled")
          self.blindspot_debug_enabled_right = True
      if CS.out.vEgo < 6 and self.blindspot_debug_enabled_right: # if enabled and speed falls below 6m/s
        #can_sends.append(make_can_msg(0x750, b'\x42\x02\x10\x01\x00\x00\x00\x00', 0))
        can_sends.append(set_blindspot_debug_mode(RIGHT_BLINDSPOT, False))
        self.blindspot_debug_enabled_right = False
        #print("debug Right blindspot debug disabled")
    if self.blindspot_debug_enabled_left:
      if frame % 20 == 0 and frame > 1001:  # Poll blindspots at 5 Hz
        can_sends.append(poll_blindspot_status(LEFT_BLINDSPOT))
        #can_sends.append(make_can_msg(0x750, b'\x41\x02\x21\x69\x00\x00\x00\x00', 0))
        #print("debug Left blindspot poll")
    if self.blindspot_debug_enabled_right:
      if frame % 20 == 10 and frame > 1005:  # Poll blindspots at 5 Hz
        #can_sends.append(make_can_msg(0x750, b'\x42\x02\x21\x69\x00\x00\x00\x00', 0))
        can_sends.append(poll_blindspot_status(RIGHT_BLINDSPOT))
        #print("debug Right blindspot poll")

    return can_sends
