from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from common.numpy_fast import interp
from cereal import car
from cereal import log

from common.realtime import sec_since_boot

def get_steer_max(CP, v_ego):
  return interp(v_ego, CP.steerMaxBP, CP.steerMaxV)

class LatControlPID(object):
  def __init__(self, CP):
    self.pid = PIController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            k_f=CP.lateralTuning.pid.kf, pos_limit=1.0)
    self.angle_steers_des = 0.
    self.angle_ff_ratio = 0.0
    self.angle_ff_gain = 1.0
    self.rate_ff_gain = 0.01
    self.angle_ff_bp = [[0.5, 5.0],[0.0, 1.0]]
    self.sat_time = 0.0

  def reset(self):
    self.pid.reset()

  def adjust_angle_gain(self):
    if (self.pid.f > 0) == (self.pid.i > 0) and abs(self.pid.i) >= abs(self.previous_integral):
      self.angle_ff_gain *= 1.0001
    else:
      self.angle_ff_gain *= 0.9999
    self.angle_ff_gain = max(1.0, self.angle_ff_gain)
    self.previous_integral = self.pid.i

  def update(self, active, v_ego, angle_steers, angle_steers_rate, steer_override, CP, VM, path_plan):
    pid_log = log.Live100Data.LateralPIDState.new_message()
    pid_log.steerAngle = float(angle_steers)
    pid_log.steerRate = float(angle_steers_rate)

    if v_ego < 0.3 or not active:
      output_steer = 0.0
      pid_log.active = False
      self.pid.reset()
      self.previous_integral = 0.0
    else:
      self.angle_steers_des = path_plan.angleSteers  #  interp(sec_since_boot(), path_plan.mpcTimes, path_plan.mpcAngles)

      steers_max = get_steer_max(CP, v_ego)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max
      steer_feedforward = self.angle_steers_des   # feedforward desired angle
      if CP.steerControlType == car.CarParams.SteerControlType.torque:
        angle_feedforward = steer_feedforward - path_plan.angleOffset
        self.angle_ff_ratio = interp(abs(angle_feedforward), self.angle_ff_bp[0], self.angle_ff_bp[1])
        angle_feedforward *= self.angle_ff_ratio * self.angle_ff_gain
        rate_feedforward = (1.0 - self.angle_ff_ratio) * self.rate_ff_gain * path_plan.rateSteers
        steer_feedforward = v_ego**2 * (rate_feedforward + angle_feedforward)

        if v_ego > 10.0:
          if abs(angle_steers) > (self.angle_ff_bp[0][1] / 2.0):
            self.adjust_angle_gain()
          else:
            self.previous_integral = self.pid.i

      deadzone = 0.0
      output_steer = self.pid.update(self.angle_steers_des, angle_steers, check_saturation=(v_ego > 10), override=steer_override,
                                     feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone)
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = bool(self.pid.saturated)

    # Reset sat_flat always, set it only if needed
    self.sat_flag = False

    # If PID is saturated, set time which it was saturated
    if self.pid.saturated and self.sat_time < 0.5:
      self.sat_time = sec_since_boot()

    # To save cycles, nest in sat_time check
    if self.sat_time > 0.5:
      # If its been saturated for 0.7 seconds then set flag
      if (sec_since_boot() - self.sat_time) > 0.7:
        self.sat_flag = True

      # If it is no longer saturated, clear the sat flag and timer
      if not self.pid.saturated:
        self.sat_time = 0.0

    if CP.steerControlType == car.CarParams.SteerControlType.torque:
      return output_steer, path_plan.angleSteers, pid_log
    else:
      return self.angle_steers_des, path_plan.angleSteers
