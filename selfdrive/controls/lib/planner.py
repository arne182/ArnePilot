#!/usr/bin/env python3
import math
from datetime import datetime
import time
import zmq
import numpy as np
from common.params import Params
from common.numpy_fast import interp
import selfdrive.messaging_arne as messaging_arne
import selfdrive.messaging as messaging
from cereal import car
from common.realtime import sec_since_boot, DT_PLAN
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.speed_smoother import speed_smoother
from selfdrive.controls.lib.longcontrol import LongCtrlState, MIN_CAN_SPEED
from selfdrive.controls.lib.fcw import FCWChecker
from selfdrive.controls.lib.long_mpc import LongitudinalMpc
from common.op_params import opParams
op_params = opParams()
offset = op_params.get('speed_offset', 0) # m/s
osm = op_params.get('osm', True)

NO_CURVATURE_SPEED = 90.0

MAX_SPEED = 255.0

LON_MPC_STEP = 0.2  # first step is 0.2s
MAX_SPEED_ERROR = 2.0
AWARENESS_DECEL = -0.2     # car smoothly decel at .2m/s^2 when user is distracted

# lookup tables VS speed to determine min and max accels in cruise
# make sure these accelerations are smaller than mpc limits
_A_CRUISE_MIN_V_FOLLOWING = [-4.0, -3.5, -3.0, -2.5, -2.0]
_A_CRUISE_MIN_V = [-1.6, -0.7, -0.6, -0.5, -0.3]
_A_CRUISE_MIN_BP = [0.0, 5.0, 10.0, 20.0, 55.0]

# need fast accel at very low speed for stop and go
# make sure these accelerations are smaller than mpc limits

_A_CRUISE_MAX_V = [3.0, 3.0, 1.5, .5, .3]
_A_CRUISE_MAX_V_ECO = [1.0, 1.5, 1.0, 0.3, 0.1]
_A_CRUISE_MAX_V_SPORT = [3.0, 3.5, 4.0, 4.0, 4.0]
_A_CRUISE_MAX_V_FOLLOWING = [1.3, 1.6, 1.2, .7, .3]
_A_CRUISE_MAX_BP = [0., 5., 10., 20., 55.]

# Lookup table for turns
_A_TOTAL_MAX_V = [2.3, 3.0, 3.9]
_A_TOTAL_MAX_BP = [0., 25., 55.]


# Model speed kalman stuff
_MODEL_V_A = [[1.0, DT_PLAN], [0.0, 1.0]]
_MODEL_V_C = [1.0, 0]
# calculated with observation std of 2m/s and accel proc noise of 2m/s**2
_MODEL_V_K = [[0.07068858], [0.04826294]]

# 75th percentile
SPEED_PERCENTILE_IDX = 7

def calc_cruise_accel_limits(v_ego, following, gas_button_status):
  if following:
    a_cruise_min = interp(v_ego, _A_CRUISE_MIN_BP, _A_CRUISE_MIN_V_FOLLOWING)
  else:
    a_cruise_min = interp(v_ego, _A_CRUISE_MIN_BP, _A_CRUISE_MIN_V)

  if following:
    a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V_FOLLOWING)
  else:
    if gas_button_status == 1:
      a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V_SPORT)
    elif gas_button_status == 2:
      a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V_ECO)
    else:
      a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V)
  return np.vstack([a_cruise_min, a_cruise_max])

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP, angle_later):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego**2 * abs(angle_steers) * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_y2 = v_ego**2 * abs(angle_later) * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = a_total_max - a_y
  a_x_allowed2 = a_total_max - a_y2
  
  a_target[1] = min(a_target[1], a_x_allowed, a_x_allowed2)
  a_target[0] = min(a_target[0], a_target[1])
  
  return a_target


class Planner():
  def __init__(self, CP):
    self.CP = CP
    self.arne_sm = messaging_arne.SubMaster(['arne182Status', 'latControl'])
    self.mpc1 = LongitudinalMpc(1)
    self.mpc2 = LongitudinalMpc(2)

    self.v_acc_start = 0.0
    self.a_acc_start = 0.0

    self.v_acc = 0.0
    self.v_acc_future = 0.0
    self.a_acc = 0.0
    self.v_cruise = 0.0
    self.a_cruise = 0.0
    self.v_model = 0.0
    self.a_model = 0.0

    self.longitudinalPlanSource = 'cruise'
    self.fcw_checker = FCWChecker()
    self.path_x = np.arange(192)

    self.params = Params()

  def choose_solution(self, v_cruise_setpoint, enabled):
    if enabled:
      solutions = {'model': self.v_model, 'cruise': self.v_cruise}
      if self.mpc1.prev_lead_status:
        solutions['mpc1'] = self.mpc1.v_mpc
      if self.mpc2.prev_lead_status:
        solutions['mpc2'] = self.mpc2.v_mpc

      slowest = min(solutions, key=solutions.get)

      self.longitudinalPlanSource = slowest
      # Choose lowest of MPC and cruise
      if slowest == 'mpc1':
        self.v_acc = self.mpc1.v_mpc
        self.a_acc = self.mpc1.a_mpc
      elif slowest == 'mpc2':
        self.v_acc = self.mpc2.v_mpc
        self.a_acc = self.mpc2.a_mpc
      elif slowest == 'cruise':
        self.v_acc = self.v_cruise
        self.a_acc = self.a_cruise
      elif slowest == 'model':
        self.v_acc = self.v_model
        self.a_acc = self.a_model

    self.v_acc_future = min([self.mpc1.v_mpc_future, self.mpc2.v_mpc_future, v_cruise_setpoint])

  def update(self, sm, pm, CP, VM, PP):
    self.arne_sm.update(0)
    gas_button_status = self.arne_sm['arne182Status'].gasbuttonstatus
    """Gets called when new radarState is available"""
    cur_time = sec_since_boot()
    v_ego = sm['carState'].vEgo
    blinkers = sm['carState'].leftBlinker or sm['carState'].rightBlinker
    if blinkers:
      steering_angle = 0.
      angle_later = 0.
    else:
      steering_angle = sm['carState'].steeringAngle
      if v_ego < 11:
        angle_later = 0.
      else:
        angle_later = self.arne_sm['latControl'].anglelater
    
    if gas_button_status == 1:
      speed_ahead_distance = 150
    elif gas_button_status == 2:
      speed_ahead_distance = 350
    else:
      speed_ahead_distance = 250

    long_control_state = sm['controlsState'].longControlState
    v_cruise_kph = sm['controlsState'].vCruise
    force_slow_decel = sm['controlsState'].forceDecel
    v_cruise_setpoint = v_cruise_kph * CV.KPH_TO_MS

    lead_1 = sm['radarState'].leadOne
    lead_2 = sm['radarState'].leadTwo

    enabled = (long_control_state == LongCtrlState.pid) or (long_control_state == LongCtrlState.stopping)
    following = lead_1.status and lead_1.dRel < 45.0 and lead_1.vLeadK > v_ego and lead_1.aLeadK > 0.0
    
    v_speedlimit = NO_CURVATURE_SPEED
    v_curvature_map = NO_CURVATURE_SPEED
    v_speedlimit_ahead = NO_CURVATURE_SPEED

    if len(sm['model'].path.poly):
      path = list(sm['model'].path.poly)

      # Curvature of polynomial https://en.wikipedia.org/wiki/Curvature#Curvature_of_the_graph_of_a_function
      # y = a x^3 + b x^2 + c x + d, y' = 3 a x^2 + 2 b x + c, y'' = 6 a x + 2 b
      # k = y'' / (1 + y'^2)^1.5
      # TODO: compute max speed without using a list of points and without numpy
      y_p = 3 * path[0] * self.path_x**2 + 2 * path[1] * self.path_x + path[2]
      y_pp = 6 * path[0] * self.path_x + 2 * path[1]
      curv = y_pp / (1. + y_p**2)**1.5

      a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
      v_curvature = np.sqrt(a_y_max / np.clip(np.abs(curv), 1e-4, None))
      model_speed = np.min(v_curvature)
      model_speed = max(20.0 * CV.MPH_TO_MS, model_speed) # Don't slow down below 20mph
    else:
      model_speed = MAX_SPEED
    now = datetime.now()
    
    try:
      if sm['liveMapData'].speedLimitValid and osm and (sm['liveMapData'].lastGps.timestamp -time.mktime(now.timetuple()) * 1000) < 10000:
        speed_limit = sm['liveMapData'].speedLimit
        if v_speedlimit is not None and offset is not None and v_speedlimit > offset:
          v_speedlimit = speed_limit + offset
        else:
          v_speedlimit = speed_limit
      else:
        speed_limit = None
      if sm['liveMapData'].speedLimitAheadValid and sm['liveMapData'].speedLimitAheadDistance < speed_ahead_distance and (sm['liveMapData'].lastGps.timestamp -time.mktime(now.timetuple()) * 1000) < 10000:
        distanceatlowlimit = 50
        if sm['liveMapData'].speedLimitAhead < 21/3.6:
          distanceatlowlimit = speed_ahead_distance = (v_ego - sm['liveMapData'].speedLimitAhead)*3.6*2
          if distanceatlowlimit < 50:
            distanceatlowlimit = 0
          distanceatlowlimit = min(distanceatlowlimit,100)
          speed_ahead_distance = (v_ego - sm['liveMapData'].speedLimitAhead)*3.6*5
          speed_ahead_distance = min(speed_ahead_distance,300)
          speed_ahead_distance = max(speed_ahead_distance,50)
        if speed_limit is not None and sm['liveMapData'].speedLimitAheadDistance > distanceatlowlimit and v_ego + 3 < sm['liveMapData'].speedLimitAhead + (speed_limit - sm['liveMapData'].speedLimitAhead)*sm['liveMapData'].speedLimitAheadDistance/speed_ahead_distance:
          speed_limit_ahead = sm['liveMapData'].speedLimitAhead + (speed_limit - sm['liveMapData'].speedLimitAhead)*(sm['liveMapData'].speedLimitAheadDistance - distanceatlowlimit)/(speed_ahead_distance - distanceatlowlimit)
        else:
          speed_limit_ahead = sm['liveMapData'].speedLimitAhead
        if v_speedlimit_ahead is not None and offset is not None and v_speedlimit_ahead > offset:
          v_speedlimit_ahead = speed_limit_ahead + offset
        else:
          v_speedlimit_ahead = speed_limit_ahead
      if sm['liveMapData'].curvatureValid and osm and (sm['liveMapData'].lastGps.timestamp -time.mktime(now.timetuple()) * 1000) < 10000:
        curvature = abs(sm['liveMapData'].curvature)
        radius = 1/max(1e-4, curvature)
        if radius > 500:
          c=0.7 # 0.7 at 1000m = 95 kph
        elif radius > 250: 
          c = 2.7-1/250*radius # 1.7 at 264m 76 kph
        else:
          c= 3.0 - 13/2500 *radius # 3.0 at 15m 24 kph
        v_curvature_map = math.sqrt(c*radius)
        v_curvature_map = min(NO_CURVATURE_SPEED, v_curvature_map)
    except KeyError:
      pass
    
    decel_for_turn = bool(v_curvature_map < min([v_cruise_setpoint, v_speedlimit, v_ego + 1.]))
    v_cruise_setpoint = min([v_cruise_setpoint, v_curvature_map, v_speedlimit, v_speedlimit_ahead])
    
    
    # Calculate speed for normal cruise control
    if enabled:
      accel_limits = [float(x) for x in calc_cruise_accel_limits(v_ego, following, gas_button_status)]
      jerk_limits = [min(-0.1, accel_limits[0]), max(0.1, accel_limits[1])]  # TODO: make a separate lookup for jerk tuning
      accel_limits_turns = limit_accel_in_turns(v_ego, steering_angle, accel_limits, self.CP, angle_later)

      if force_slow_decel:
        # if required so, force a smooth deceleration
        accel_limits_turns[1] = min(accel_limits_turns[1], AWARENESS_DECEL)
        accel_limits_turns[0] = min(accel_limits_turns[0], accel_limits_turns[1])
        
      if decel_for_turn and sm['liveMapData'].distToTurn < speed_ahead_distance:
        time_to_turn = max(1.0, sm['liveMapData'].distToTurn / max((v_ego + v_curvature_map)/2, 1.))
        required_decel = min(0, (v_curvature_map - v_ego) / time_to_turn)
        accel_limits[0] = max(accel_limits[0], required_decel)
      if v_speedlimit_ahead < v_speedlimit and self.longitudinalPlanSource =='cruise' and v_ego > v_speedlimit_ahead and sm['liveMapData'].speedLimitAheadDistance > 0.10:
        required_decel = min(0, (v_speedlimit_ahead*v_speedlimit_ahead - v_ego*v_ego)/(sm['liveMapData'].speedLimitAheadDistance*2))
        required_decel = max(required_decel, -3.0)
        accel_limits[0] = required_decel
        accel_limits[1] = required_decel
        self.a_acc_start = required_decel

      
      self.v_cruise, self.a_cruise = speed_smoother(self.v_acc_start, self.a_acc_start,
                                                    v_cruise_setpoint,
                                                    accel_limits_turns[1], accel_limits_turns[0],
                                                    jerk_limits[1], jerk_limits[0],
                                                    LON_MPC_STEP)

      self.v_model, self.a_model = speed_smoother(self.v_acc_start, self.a_acc_start,
                                                    model_speed,
                                                    2*accel_limits[1], accel_limits[0],
                                                    2*jerk_limits[1], jerk_limits[0],
                                                    LON_MPC_STEP)

      # cruise speed can't be negative even is user is distracted
      self.v_cruise = max(self.v_cruise, 0.)
    else:
      starting = long_control_state == LongCtrlState.starting
      a_ego = min(sm['carState'].aEgo, 0.0)
      reset_speed = MIN_CAN_SPEED if starting else v_ego
      reset_accel = self.CP.startAccel if starting else a_ego
      self.v_acc = reset_speed
      self.a_acc = reset_accel
      self.v_acc_start = reset_speed
      self.a_acc_start = reset_accel
      self.v_cruise = reset_speed
      self.a_cruise = reset_accel

    self.mpc1.set_cur_state(self.v_acc_start, self.a_acc_start)
    self.mpc2.set_cur_state(self.v_acc_start, self.a_acc_start)

    self.mpc1.update(pm, sm['carState'], lead_1, v_cruise_setpoint)
    self.mpc2.update(pm, sm['carState'], lead_2, v_cruise_setpoint)

    self.choose_solution(v_cruise_setpoint, enabled)

    # determine fcw
    if self.mpc1.new_lead:
      self.fcw_checker.reset_lead(cur_time)

    
    fcw = self.fcw_checker.update(self.mpc1.mpc_solution, cur_time,
                                  sm['controlsState'].active,
                                  v_ego, sm['carState'].aEgo,
                                  lead_1.dRel, lead_1.vLead, lead_1.aLeadK,
                                  lead_1.yRel, lead_1.vLat,
                                  lead_1.fcw, blinkers) and not sm['carState'].brakePressed
    if fcw:
      cloudlog.info("FCW triggered %s", self.fcw_checker.counters)

    radar_dead = not sm.alive['radarState']

    radar_errors = list(sm['radarState'].radarErrors)
    radar_fault = car.RadarData.Error.fault in radar_errors
    radar_can_error = car.RadarData.Error.canError in radar_errors

    # **** send the plan ****
    plan_send = messaging.new_message()
    plan_send.init('plan')

    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState', 'radarState'])

    plan_send.plan.mdMonoTime = sm.logMonoTime['model']
    plan_send.plan.radarStateMonoTime = sm.logMonoTime['radarState']

    # longitudal plan
    plan_send.plan.vCruise = float(self.v_cruise)
    plan_send.plan.aCruise = float(self.a_cruise)
    plan_send.plan.vStart = float(self.v_acc_start)
    plan_send.plan.aStart = float(self.a_acc_start)
    plan_send.plan.vTarget = float(self.v_acc)
    plan_send.plan.aTarget = float(self.a_acc)
    plan_send.plan.vTargetFuture = float(self.v_acc_future)
    plan_send.plan.hasLead = self.mpc1.prev_lead_status
    plan_send.plan.longitudinalPlanSource = self.longitudinalPlanSource
    
    plan_send.plan.vCurvature = float(v_curvature_map)
    plan_send.plan.decelForTurn = bool(decel_for_turn or v_speedlimit_ahead < min([v_speedlimit, v_ego + 1.]))
    plan_send.plan.mapValid = True
    
    radar_valid = not (radar_dead or radar_fault)
    plan_send.plan.radarValid = bool(radar_valid)
    plan_send.plan.radarCanError = bool(radar_can_error)

    plan_send.plan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.rcv_time['radarState']

    # Send out fcw
    plan_send.plan.fcw = fcw

    pm.send('plan', plan_send)

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_acc_sol = self.a_acc_start + (DT_PLAN / LON_MPC_STEP) * (self.a_acc - self.a_acc_start)
    v_acc_sol = self.v_acc_start + DT_PLAN * (a_acc_sol + self.a_acc_start) / 2.0
    self.v_acc_start = v_acc_sol
    self.a_acc_start = a_acc_sol
