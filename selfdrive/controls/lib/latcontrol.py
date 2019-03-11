from selfdrive.controls.lib.pid import PIController
from common.numpy_fast import interp
from common.realtime import sec_since_boot
from cereal import car
import math


def get_steer_max(CP, v_ego):
  return interp(v_ego, CP.steerMaxBP, CP.steerMaxV)


def apply_deadzone(angle, deadzone):
  if angle > deadzone:
    angle -= deadzone
  elif angle < -deadzone:
    angle += deadzone
  else:
    angle = 0.
  return angle


class LatControl(object):
  def __init__(self, CP):
    self.smooth_factor = CP.steerDampening * 100.
    self.ff_angle_factor = 1.0
    self.ff_rate_factor = 10.0
    self.dampened_angle_steers = 0.0
    # Eliminate break-points, since they aren't needed (and would cause problems for resonance)
    KpV = [interp(25.0, CP.steerKpBP, CP.steerKpV)]
    KiV = [interp(25.0, CP.steerKiBP, CP.steerKiV)]
    self.pid = PIController(([0.], KpV),
                            ([0.], KiV),
                            k_f=CP.steerKf, pos_limit=1.0)
    self.feed_forward = 0.0
    self.steer_counter = 1.0
    self.steer_counter_prev = 0.0
    self.rough_steers_rate = 0.0
    self.prev_angle_steers = 0.0
    self.calculate_rate = True

  def reset(self):
    self.pid.reset()

  def update(self, active, v_ego, angle_steers, angle_rate, angle_offset, steer_override, CP, VM, path_plan):

    if angle_rate == 0.0 and self.calculate_rate:
      if angle_steers != self.prev_angle_steers:
        self.steer_counter_prev = self.steer_counter
        self.rough_steers_rate = (self.rough_steers_rate + 100.0 * (angle_steers - self.prev_angle_steers) / self.steer_counter_prev) / 2.0
        self.steer_counter = 0.0
      elif self.steer_counter >= self.steer_counter_prev:
        self.rough_steers_rate = (self.steer_counter * self.rough_steers_rate) / (self.steer_counter + 1.0)
      self.steer_counter += 1.0
      angle_rate = self.rough_steers_rate
    else:
      # If non-zero angle_rate is provided, use it instead
      self.calculate_rate = False

    if v_ego < 0.3 or not active:
      output_steer = 0.0
      self.pid.reset()
      self.feed_forward = 0.0
      self.angle_steers_des = angle_steers
    else:
      # Interpolate desired angle between MPC updates
      self.angle_steers_des = interp(sec_since_boot() + CP.steerLatency, path_plan.mpcTimes, path_plan.mpcAngles)

      if CP.steerControlType == car.CarParams.SteerControlType.torque:
        steers_max = get_steer_max(CP, v_ego)
        self.pid.pos_limit = steers_max
        self.pid.neg_limit = -steers_max
        deadzone = 0

        desired_rate = self.angle_steers_des - float(angle_steers)
        projected_angle_steers = float(angle_steers) + CP.steerDampening * float(angle_rate)
        self.dampened_angle_steers = ((self.smooth_factor * self.dampened_angle_steers) + projected_angle_steers) / (1. + self.smooth_factor)

        angle_feed_forward = self.ff_angle_factor * apply_deadzone(self.angle_steers_des - float(angle_offset), 0.5)
        rate_feed_forward = self.ff_rate_factor * desired_rate

        # Decide which feed forward mode should be used (angle or rate).  Use more dominant mode, but only if conditions are met
        rate_more_significant = abs(rate_feed_forward) > abs(angle_feed_forward)
        rate_angle_same_direction = (angle_feed_forward < 0) == (rate_feed_forward < 0)
        more_rate_desired = abs(desired_rate) > abs(angle_rate)
        rate_other_direction = (desired_rate < 0) != (angle_rate < 0)

        # Spread out feed_forward over the actuator's response time to reduce noise
        if rate_more_significant and rate_angle_same_direction and (more_rate_desired or rate_other_direction):
          self.feed_forward = ((self.smooth_factor * self.feed_forward) + v_ego**2 * rate_feed_forward) / (1. + self.smooth_factor)
        else:
          self.feed_forward = ((self.smooth_factor * self.feed_forward) + v_ego**2 * angle_feed_forward) / (1. + self.smooth_factor)

        output_steer = self.pid.update(self.angle_steers_des, self.dampened_angle_steers, check_saturation=(v_ego > 10),
                                        override=steer_override, feedforward=self.feed_forward, speed=v_ego, deadzone=deadzone)

    self.sat_flag = self.pid.saturated
    self.prev_angle_steers = angle_steers

    if CP.steerControlType == car.CarParams.SteerControlType.torque:
      return output_steer, path_plan.mpcAngles[1]
    else:
      return self.angle_steers_des, path_plan.mpcAngles[1]
