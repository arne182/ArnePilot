import os
from common.numpy_fast import interp
import math

import selfdrive.messaging as messaging
from selfdrive.swaglog import cloudlog
from common.realtime import sec_since_boot
from selfdrive.controls.lib.radar_helpers import _LEAD_ACCEL_TAU
from selfdrive.controls.lib.longitudinal_mpc import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LONG
from selfdrive.phantom import Phantom
import time

LOG_MPC = os.environ.get('LOG_MPC', False)


class LongitudinalMpc():
  def __init__(self, mpc_id):
    self.mpc_id = mpc_id

    self.setup_mpc()
    self.v_mpc = 0.0
    self.v_mpc_future = 0.0
    self.a_mpc = 0.0
    self.v_cruise = 0.0
    self.prev_lead_status = False
    self.prev_lead_x = 0.0
    self.new_lead = False
    self.v_ego = 0.0
    self.car_state = None
    self.last_cost = 0
    self.car_data = {"lead_vels": [], "traffic_vels": []}
    self.mpc_frame = 0  # idea thanks to kegman
    self.last_time = None
    self.v_lead = None
    self.x_lead = None
    self.df_frame = 0
    self.phantom = Phantom(timeout=1.)

    self.last_cloudlog_t = 0.0

  def send_mpc_solution(self, pm, qp_iterations, calculation_time):
    qp_iterations = max(0, qp_iterations)
    dat = messaging.new_message()
    dat.init('liveLongitudinalMpc')
    dat.liveLongitudinalMpc.xEgo = list(self.mpc_solution[0].x_ego)
    dat.liveLongitudinalMpc.vEgo = list(self.mpc_solution[0].v_ego)
    dat.liveLongitudinalMpc.aEgo = list(self.mpc_solution[0].a_ego)
    dat.liveLongitudinalMpc.xLead = list(self.mpc_solution[0].x_l)
    dat.liveLongitudinalMpc.vLead = list(self.mpc_solution[0].v_l)
    dat.liveLongitudinalMpc.cost = self.mpc_solution[0].cost
    dat.liveLongitudinalMpc.aLeadTau = self.a_lead_tau
    dat.liveLongitudinalMpc.qpIterations = qp_iterations
    dat.liveLongitudinalMpc.mpcId = self.mpc_id
    dat.liveLongitudinalMpc.calculationTime = calculation_time
    pm.send('liveLongitudinalMpc', dat)

  def setup_mpc(self):
    ffi, self.libmpc = libmpc_py.get_libmpc(self.mpc_id)
    self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                     MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)

    self.mpc_solution = ffi.new("log_t *")
    self.cur_state = ffi.new("state_t *")
    self.cur_state[0].v_ego = 0
    self.cur_state[0].a_ego = 0
    self.a_lead_tau = _LEAD_ACCEL_TAU

  def set_cur_state(self, v, a):
    self.cur_state[0].v_ego = v
    self.cur_state[0].a_ego = a

  def get_acceleration(self):  # calculate acceleration to generate more accurate following distances
    a = 0.0
    if len(self.car_data["lead_vels"]) > self.calc_rate(2):
      num = (self.car_data["lead_vels"][-1] - self.car_data["lead_vels"][0])
      den = len(self.car_data["lead_vels"]) / self.calc_rate()
      if den > 0:
        a = num / float(den)
    return a

  def save_car_data(self):
    if self.v_lead is not None:
      while len(self.car_data["lead_vels"]) > self.calc_rate(3):  # 3 seconds
        del self.car_data["lead_vels"][0]
      self.car_data["lead_vels"].append(self.v_lead)

      if self.mpc_frame >= self.calc_rate():  # add to traffic list every second so we're not working with a huge list
        while len(self.car_data["traffic_vels"]) > 180:  # 3 minutes of traffic logging
          del self.car_data["traffic_vels"][0]
        self.car_data["traffic_vels"].append(self.v_lead)
        self.mpc_frame = 0  # reset every second
      self.mpc_frame += 1  # increment every frame

    else:  # if no car, reset lead car list; ignore for traffic
      self.car_data["lead_vels"] = []

  def get_traffic_level(self):  # based on fluctuation of v_lead
    lead_vels = self.car_data["traffic_vels"]
    if len(lead_vels) < 20:  # seconds
      return 1.0
    lead_vel_diffs = [abs(vel - lead_vels[idx - 1]) for idx, vel in enumerate(lead_vels) if idx != 0]
    x = [0.0, 0.21, 0.466, 0.722, 0.856, 0.96, 1.0]  # 1 is estimated to be heavy traffic
    y = [1.2, 1.19, 1.17, 1.13, 1.09, 1.04, 1.0]
    traffic_mod = interp(sum(lead_vel_diffs)/len(lead_vel_diffs), x, y)
    x = [20.1168, 24.5872]  # min speed is 45mph for traffic level mod
    y = [0.2, 0.0]
    traffic_mod = max(traffic_mod - interp(self.v_ego, x, y), 1.0)
    return traffic_mod

  def smooth_follow(self):  # in m/s
    x_vel = [0.0, 5.222, 11.164, 14.937, 20.973, 33.975, 42.469]
    y_mod = [1.542, 1.553, 1.599, 1.68, 1.75, 1.855, 1.9]

    if self.v_ego > 6.7056:  # 8 mph
      TR = interp(self.v_ego, x_vel, y_mod)
    else:  # this allows us to get slightly closer to the lead car when stopping, while being able to have smooth stop and go
      x = [4.4704, 6.7056]  # smoothly ramp TR between 10 and 15 mph from 1.8s to defined TR above at 15mph
      y = [1.8, interp(x[1], x_vel, y_mod)]
      TR = interp(self.v_ego, x, y)

    if self.v_lead is not None:  # since the new mpc now handles braking nicely, simplify mods
      x = [2.68, 2.1, 1.26, 0.61, 0, 0.61, 1.26, 2.1, 2.68]  # relative velocity values todo: add back negative values to get farther away from lead when it's slower
      y = [0.272, 0.154, 0.053, 0.017, 0, -0.017, -0.053, -0.154, -0.272]  # modification values
      TR_mod = interp(self.v_lead - self.v_ego, x, y)  # quicker acceleration/don't brake when lead is overtaking

      # x = [-1.49, -1.1, -0.67, 0.0, 0.67, 1.1, 1.49]  # todo: work on adding this back
      # y = [0.056, 0.032, 0.016, 0.0, -0.016, -0.032, -0.056]
      # TR_mod += interp(self.get_acceleration(), x, y)  # when lead car has been braking over the past 3 seconds, slightly increase TR

      TR += TR_mod
      #TR *= self.get_traffic_level()  # modify TR based on last minute of traffic data  # todo: look at getting this to work, a model could be used
    if TR < 0.9:
      return 0.9
    else:
      return round(TR, 3)

  def get_cost(self, TR):
    x = [.9, 1.8, 2.7]
    y = [4.5, 2.8, 1.3]
    if self.x_lead is not None and self.v_ego is not None and self.v_ego != 0:
      real_TR = self.x_lead / float(self.v_ego)  # switched to cost generation using actual distance from lead car; should be safer
      if abs(real_TR - TR) >= .25:  # use real TR if diff is greater than x safety threshold
        TR = real_TR
    if self.v_lead is not None and self.v_ego > 5:
      factor = min(1,max(2,(self.v_lead - self.v_ego)/2 + 1.5))
      return min(round(float(interp(TR, x, y)), 3)/factor, 0.1)
    else:
      return round(float(interp(TR, x, y)), 3)

  def get_TR(self):
    read_distance_lines = 2

    if self.v_ego < 2.0 and read_distance_lines != 2:
      return 1.8
    elif (self.car_state.leftBlinker or self.car_state.rightBlinker) and self.v_ego > 8.9408:  # don't get super close when signaling in a turn lane
      if self.last_cost != 1.0:
        self.libmpc.change_tr(MPC_COST_LONG.TTC, 1.0, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.last_cost = 1.0
      return 0.9  # accelerate for lane change
    elif read_distance_lines == 1:
      if self.last_cost != 1.0:
        self.libmpc.change_tr(MPC_COST_LONG.TTC, 1.0, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.last_cost = 1.0
      return 0.9  # 10m at 40km/hr
    elif read_distance_lines == 2:
      self.save_car_data()
      TR = self.smooth_follow()
      cost = self.get_cost(TR)
      self.libmpc.change_tr(MPC_COST_LONG.TTC, cost, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
      self.last_cost = cost
      return TR
    else:
      if self.last_cost != 0.05:
        self.libmpc.change_tr(MPC_COST_LONG.TTC, 0.05, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.last_cost = 0.05
      return 2.7  # 30m at 40km/hr

  def calc_rate(self, seconds=1.0, new_frame=False):  # return current rate of long_mpc in fps/hertz
    current_time = time.time()
    if self.last_time is None or (current_time - self.last_time) <= 0:
      rate = int(round(40.42 * seconds))  # average of tests on long_mpc
    else:
      rate = (1.0 / (current_time - self.last_time)) * seconds

    min_return = 10
    max_return = seconds * 100
    if new_frame:
      self.last_time = current_time
    return int(round(max(min(rate, max_return), min_return)))  # ensure we return a value between range, in hertz

  def process_phantom(self, lead):
    if lead is not None and lead.status:
      v_lead = max(0.0, lead.vLead)
      if v_lead < 0.1 or -lead.aLeadK / 2.0 > v_lead:
        v_lead = 0.0
      # if radar lead is available, ensure we use that as the real lead rather than ignoring it and running into it
      # todo: this is buggy and probably needs to be looked at
      x_lead = min(9.144, lead.dRel)
      v_lead = min(self.phantom.data["speed"], v_lead)
    else:
      x_lead = 9.144
      v_lead = self.phantom.data["speed"]
    return x_lead, v_lead

  def update(self, pm, CS, lead, v_cruise_setpoint):
    v_ego = CS.vEgo
    self.car_state = CS
    self.v_ego = CS.vEgo
    self.phantom.update()

    # Setup current mpc state
    self.cur_state[0].x_ego = 0.0

    if self.phantom.data["status"]:
      if self.phantom.data["speed"] != 0.0:
        x_lead, v_lead = self.process_phantom(lead)
      elif 'lost_connection' in self.phantom.data:
        x_lead = 1.5
        v_lead = 0.  # stop as quickly as possible
      else:  # else, smooth deceleration
        x_lead = 3.75
        v_lead = max(self.v_ego - (.7 / max(max(self.v_ego, 0) ** .4, .01)), 0.0)  # smoothly decelerate to 0 todo: tune this!

      a_lead = 0.0
      self.a_lead_tau = lead.aLeadTau
      self.new_lead = False
      if not self.prev_lead_status or abs(x_lead - self.prev_lead_x) > 2.5:
        self.libmpc.init_with_simulation(self.v_mpc, x_lead, v_lead, a_lead, self.a_lead_tau)
        self.new_lead = True

      self.prev_lead_status = True
      self.prev_lead_x = x_lead
      self.cur_state[0].x_l = x_lead
      self.cur_state[0].v_l = v_lead
    elif lead is not None and lead.status:
      x_lead = lead.dRel
      v_lead = max(0.0, lead.vLead)
      a_lead = lead.aLeadK

      if (v_lead < 0.1 or -a_lead / 2.0 > v_lead):
        v_lead = 0.0
        a_lead = 0.0

      self.v_lead = v_lead
      self.x_lead = x_lead

      self.a_lead_tau = lead.aLeadTau
      self.new_lead = False
      if not self.prev_lead_status or abs(x_lead - self.prev_lead_x) > 2.5:
        self.libmpc.init_with_simulation(self.v_mpc, x_lead, v_lead, a_lead, self.a_lead_tau)
        self.new_lead = True

      self.prev_lead_status = True
      self.prev_lead_x = x_lead
      self.cur_state[0].x_l = x_lead
      self.cur_state[0].v_l = v_lead
    else:
      self.prev_lead_status = False
      # Fake a fast lead car, so mpc keeps running
      self.cur_state[0].x_l = 50.0
      self.cur_state[0].v_l = v_ego + 10.0
      a_lead = 0.0
      self.v_lead = None
      self.x_lead = None
      self.a_lead_tau = _LEAD_ACCEL_TAU

    # Calculate mpc
    t = sec_since_boot()
    TR = self.get_TR()
    n_its = self.libmpc.run_mpc(self.cur_state, self.mpc_solution, self.a_lead_tau, a_lead, TR)
    duration = int((sec_since_boot() - t) * 1e9)

    if LOG_MPC:
      self.send_mpc_solution(pm, n_its, duration)

    # Get solution. MPC timestep is 0.2 s, so interpolation to 0.05 s is needed
    self.v_mpc = self.mpc_solution[0].v_ego[1]
    self.a_mpc = self.mpc_solution[0].a_ego[1]
    self.v_mpc_future = self.mpc_solution[0].v_ego[10]

    # Reset if NaN or goes through lead car
    crashing = any(lead - ego < -50 for (lead, ego) in zip(self.mpc_solution[0].x_l, self.mpc_solution[0].x_ego))
    nans = any(math.isnan(x) for x in self.mpc_solution[0].v_ego)
    backwards = min(self.mpc_solution[0].v_ego) < -0.01

    if ((backwards or crashing) and self.prev_lead_status) or nans:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Longitudinal mpc %d reset - backwards: %s crashing: %s nan: %s" % (
                          self.mpc_id, backwards, crashing, nans))

      self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                       MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
      self.cur_state[0].v_ego = v_ego
      self.cur_state[0].a_ego = 0.0
      self.v_mpc = v_ego
      self.a_mpc = CS.aEgo
      self.prev_lead_status = False
