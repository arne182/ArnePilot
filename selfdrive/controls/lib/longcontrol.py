from cereal import log
from common.numpy_fast import clip, interp
from selfdrive.controls.lib.pid import PIDController
from common.travis_checker import travis
from selfdrive.config import Conversions as CV

LongCtrlState = log.ControlsState.LongControlState

STOPPING_EGO_SPEED = 0.5
MIN_CAN_SPEED = 0.3  # TODO: parametrize this in car interface
STOPPING_TARGET_SPEED = MIN_CAN_SPEED + 0.01
STARTING_TARGET_SPEED = 0.5
BRAKE_THRESHOLD_TO_PID = 0.2

STOPPING_BRAKE_RATE = 0.2  # brake_travel/s while trying to stop
STARTING_BRAKE_RATE = 0.8  # brake_travel/s while releasing on restart
BRAKE_STOPPING_TARGET = 0.7  # apply at least this amount of brake to maintain the vehicle stationary

_MAX_SPEED_ERROR_BP = [0., 30.]  # speed breakpoints
_MAX_SPEED_ERROR_V = [1.5, .8]  # max positive v_pid error VS actual speed; this avoids controls windup due to slow pedal resp

RATE = 100.0


def long_control_state_trans(active, long_control_state, v_ego, v_target, v_pid,
                             output_gb, brake_pressed, cruise_standstill, stop):
  """Update longitudinal control state machine"""
  stopping_condition = stop or (v_ego < 2.0 and cruise_standstill) or \
                       (v_ego < STOPPING_EGO_SPEED and \
                        ((v_pid < STOPPING_TARGET_SPEED and v_target < STOPPING_TARGET_SPEED) or
                        brake_pressed))

  starting_condition = v_target > STARTING_TARGET_SPEED and not cruise_standstill

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if active:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.starting

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif output_gb >= -BRAKE_THRESHOLD_TO_PID:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl():
  def __init__(self, CP, compute_gb):
    self.long_control_state = LongCtrlState.off  # initialized to off
    kdBP = [0., 33, 55., 78]
    kdBP = [i * CV.MPH_TO_MS for i in kdBP]
    kdV = [0.05, 0.4, 0.8, 1.2]
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             (kdBP, kdV),
                             rate=RATE,
                             sat_limit=0.8,
                             convert=compute_gb)
    self.v_pid = 0.0
    self.lastdecelForTurn = False
    #self.had_lead = False
    self.last_output_gb = 0.0

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def dynamic_gas(self, v_ego, gas_interceptor, gas_button_status):
    dynamic = False
    if gas_interceptor:
      if gas_button_status == 0:
        dynamic = True
        x = [0.0, 1.4082, 2.8031, 4.2266, 5.3827, 6.1656, 7.2478, 8.2831, 10.2447, 12.964, 15.423, 18.119, 20.117, 24.4661, 29.0581, 32.7101, 35.7633]
        y = [0.3, 0.304, 0.315, 0.342, 0.365, 0.386, 0.429, 0.454, 0.472, 0.48, 0.489, 0.421, 0.432, 0.480, 0.55, 0.621, 0.7]
      elif gas_button_status == 1:
        y = [0.3, 0.9, 0.9]
      elif gas_button_status == 2:
        y = [0.2, 0.5, 0.7]
    else:
      if gas_button_status == 0:
        dynamic = True
        x = [0.0, 1.4082, 2.8031, 4.2266, 5.3827, 6.1656, 7.2478, 8.2831, 10.2447, 12.964, 15.423, 18.119, 20.117, 24.4661, 29.0581, 32.7101, 35.7633]
        y = [0.3, 0.304, 0.315, 0.342, 0.365, 0.386, 0.429, 0.454, 0.472, 0.48, 0.489, 0.421, 0.432, 0.480, 0.55, 0.621, 0.7]
      elif gas_button_status == 1:
        y = [0.5, 0.95, 0.99]
      elif gas_button_status == 2:
        y = [0.25, 0.2, 0.2]

    if not dynamic:
      x = [0., 9., 55.]  # default BP values

    accel = interp(v_ego, x, y)

    #if dynamic and hasLead:  # dynamic gas profile specific operations, and if lead
    #  if v_ego < 6.7056:  # if under 15 mph
    #    x = [1.61479, 1.99067, 2.28537, 2.49888, 2.6312, 2.68224]
    #    y = [-accel, -(accel / 1.06), -(accel / 1.2), -(accel / 1.8), -(accel / 4.4), 0]  # array that matches current chosen accel value
    #    accel += interp(vRel, x, y)
    #  else:
    #    x = [-0.89408, 0, 0.89408, 4.4704]
    #    y = [-.15, -.05, .005, .05]
    #    accel += interp(vRel, x, y)

    min_return = 0.0
    max_return = 1.0
    return round(max(min(accel, max_return), min_return), 5)  # ensure we return a value between range

  def update(self, active, CS, v_target, v_target_future, a_target, CP, hasLead, radarState, decelForTurn, longitudinalPlanSource, gas_button_status):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    try:
      gas_interceptor = CP.enableGasInterceptor
    except AttributeError:
      gas_interceptor = False
    # Actuation limits
    #gas_max = interp(CS.vEgo, CP.gasMaxBP, CP.gasMaxV)
    gas_max = self.dynamic_gas(CS.vEgo, gas_interceptor, gas_button_status)
    brake_max = interp(CS.vEgo, CP.brakeMaxBP, CP.brakeMaxV)

    # Update state machine
    output_gb = self.last_output_gb
    if radarState is None:
      dRel = 200
    else:
      dRel = radarState.leadOne.dRel
    if hasLead:
      stop = True if (dRel < 4.0 and radarState.leadOne.status) else False
    else:
      stop = False
    self.long_control_state = long_control_state_trans(active, self.long_control_state, CS.vEgo,
                                                       v_target_future, self.v_pid, output_gb,
                                                       CS.brakePressed, CS.cruiseState.standstill, stop)


    v_ego_pid = max(CS.vEgo, MIN_CAN_SPEED)  # Without this we get jumps, CAN bus reports 0 when speed < 0.3

    if self.long_control_state == LongCtrlState.off or (CS.brakePressed or CS.gasPressed and not travis):
      self.v_pid = v_ego_pid
      self.pid.reset()
      output_gb = 0.

    # tracking objects and driving
    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target
      self.pid.pos_limit = gas_max
      self.pid.neg_limit = - brake_max

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      prevent_overshoot = not CP.stoppingControl and CS.vEgo < 1.5 and v_target_future < 0.7
      deadzone = interp(v_ego_pid, CP.longitudinalTuning.deadzoneBP, CP.longitudinalTuning.deadzoneV)
      #if not self.had_lead and has_lead:
      #  if enableGasInterceptor:
      #    self.pid._k_p = ([0., 5., 35.], [1.2, 0.8, 0.5])
      #    self.pid._k_i = ([0., 35.], [0.18, 0.12])
      #  else:
      #    self.pid._k_p = ([0., 5., 35.], [3.6, 2.4, 1.5])
      #    self.pid._k_i = ([0., 35.], [0.54, 0.36])
      #elif self.had_lead and not has_lead:
      #  self.pid._k_p = (CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV)
      #  self.pid._k_i = (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV)
      #self.had_lead = has_lead
      if longitudinalPlanSource == 'cruise':
        if decelForTurn and not self.lastdecelForTurn:
          self.lastdecelForTurn = True
          self.pid._k_p = (CP.longitudinalTuning.kpBP, [x * 0 for x in CP.longitudinalTuning.kpV])
          self.pid._k_i = (CP.longitudinalTuning.kiBP, [x * 0 for x in CP.longitudinalTuning.kiV])
          self.pid.i = 0.0
          self.pid.k_f=1.0
          self.v_pid = CS.vEgo
          self.pid.reset()
        if self.lastdecelForTurn and not decelForTurn:
          self.lastdecelForTurn = False
          self.pid._k_p = (CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV)
          self.pid._k_i = (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV)
          self.pid.k_f=1.0
          self.v_pid = CS.vEgo
          self.pid.reset()
      else:
        if self.lastdecelForTurn:
          self.v_pid = CS.vEgo
          self.pid.reset()
        self.lastdecelForTurn = False
        self.pid._k_p = (CP.longitudinalTuning.kpBP, [x * 1 for x in CP.longitudinalTuning.kpV])
        self.pid._k_i = (CP.longitudinalTuning.kiBP, [x * 1 for x in CP.longitudinalTuning.kiV])
        self.pid.k_f=1.0


      output_gb = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, deadzone=deadzone, feedforward=a_target, freeze_integrator=prevent_overshoot)

      if prevent_overshoot:
        output_gb = min(output_gb, 0.0)

    # Intention is to stop, switch to a different brake control until we stop
    elif self.long_control_state == LongCtrlState.stopping:
      # Keep applying brakes until the car is stopped
      factor = 1
      if hasLead:
        factor = interp(dRel,[2.0,3.0,4.0,5.0,6.0,7.0,8.0], [3.0,2.1,1.5,1.0,0.6,0.29,0.0])
      if not CS.standstill or output_gb > -BRAKE_STOPPING_TARGET:
        output_gb -= STOPPING_BRAKE_RATE / RATE * factor
      output_gb = clip(output_gb, -brake_max, gas_max)

      self.v_pid = CS.vEgo
      self.pid.reset()

    # Intention is to move again, release brake fast before handing control to PID
    elif self.long_control_state == LongCtrlState.starting:
      factor = 1
      if hasLead:
        factor = interp(dRel,[0.0,2.0,4.0,6.0], [0.0,0.5,1.0,2.0])
      if output_gb < -0.2:
        output_gb += STARTING_BRAKE_RATE / RATE * factor
      self.v_pid = CS.vEgo
      self.pid.reset()

    self.last_output_gb = output_gb
    final_gas = clip(output_gb, 0., gas_max)
    final_brake = -clip(output_gb, -brake_max, 0.)

    return final_gas, final_brake
