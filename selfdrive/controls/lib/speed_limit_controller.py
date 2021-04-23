import numpy as np
from cereal import log, car
from common.params import Params
from common.realtime import sec_since_boot
from selfdrive.controls.lib.speed_smoother import speed_smoother
from selfdrive.controls.lib.events import Events

_LON_MPC_STEP = 0.2  # Time stemp of longitudinal control (5 Hz)
_WAIT_TIME_LIMIT_RISE = 2.0  # Waiting time before raising the speed limit.
_MIN_ADAPTING_BRAKE_ACC = -2.0  # Minimum acceleration allowed when adapting to lower speed limit.
_SPEED_OFFSET_TH = -5.0  # Maximum offset between speed limit and current speed for adapting state.
_LIMIT_ADAPT_TIME = 4.0  # Ideal time (s) to adapt to lower speed limit. i.e. braking.


_MAX_SPEED_OFFSET_DELTA = 1.0  # m/s Maximum delta for speed limit changes.

SpeedLimitControlState = log.ControlsState.SpeedLimitControlState
EventName = car.CarEvent.EventName


def _description_for_state(speed_limit_control_state):
  if speed_limit_control_state == SpeedLimitControlState.inactive:
    return 'INACTIVE'
  if speed_limit_control_state == SpeedLimitControlState.tempInactive:
    return 'TEMP_INACTIVE'
  if speed_limit_control_state == SpeedLimitControlState.adapting:
    return 'ADAPTING'
  if speed_limit_control_state == SpeedLimitControlState.active:
    return 'ACTIVE'


class SpeedLimitController():
  def __init__(self, CP):
    self._params = Params()
    self._last_params_update = 0.0
    self._is_metric = self._params.get("IsMetric", True, encoding='utf8') == "1"
    self._is_enabled = self._params.get("SpeedLimitControl", True, encoding='utf8') == "1"
    self._speed_limit_perc_offset = float(self._params.get("SpeedLimitPercOffset", True))
    self._CP = CP
    self._op_enabled = False
    self._active_jerk_limits = [0.0, 0.0]
    self._active_accel_limits = [0.0, 0.0]
    self._adapting_jerk_limits = [_MIN_ADAPTING_BRAKE_ACC, 1.0]
    self._v_ego = 0.0
    self._a_ego = 0.0
    self._v_offset = 0.0
    self._v_cruise_setpoint = 0.0
    self._v_cruise_setpoint_prev = 0.0
    self._v_cruise_setpoint_changed = False
    self._speed_limit_set = 0.0
    self._speed_limit_set_prev = 0.0
    self._speed_limit_set_change = 0.0
    self._speed_limit = 0.0
    self._speed_limit_prev = 0.0
    self._speed_limit_changed = False
    self._last_speed_limit_set_change_ts = 0.0
    self._state = SpeedLimitControlState.inactive
    self._state_prev = SpeedLimitControlState.inactive
    self._adapting_cycles = 0

    self.v_limit = 0.0
    self.a_limit = 0.0
    self.v_limit_future = 0.0

  @property
  def state(self):
    return self._state

  @state.setter
  def state(self, value):
    if value != self._state:
      print(f'Speed Limit Controller state: {_description_for_state(value)}')
      if value == SpeedLimitControlState.adapting:
        self._adapting_cycles = 0  # Reset adapting state cycle count when entereing state.
      elif value == SpeedLimitControlState.tempInactive:
        # Make sure speed limit is set to `set` value, this will have the effect
        # of canceling delayed increase limit, if pending.
        self._speed_limit = self._speed_limit_set
        self._speed_limit_prev = self._speed_limit
    self._state = value

  @property
  def is_active(self):
    return self.state > SpeedLimitControlState.tempInactive

  @property
  def speed_limit(self):
    return self._speed_limit * (1.0 + self._speed_limit_perc_offset / 100.0)

  def _update_params(self):
    time = sec_since_boot()
    if time > self._last_params_update + 5.0:
      self._speed_limit_perc_offset = float(self._params.get("SpeedLimitPercOffset"))
      self._is_enabled = self._params.get("SpeedLimitControl", encoding='utf8') == "1"
      print(f'Updated Speed limit params. enabled: {self._is_enabled}, \
              perc_offset: {self._speed_limit_perc_offset:.1f}')
      self._last_params_update = time

  def _update_calculations(self):
    # Track the time when speed limit set value changes.
    time = sec_since_boot()
    if self._speed_limit_set != self._speed_limit_set_prev:
      self._last_speed_limit_set_change_ts = time
    # Update speed limit from the set value.
    # - Imediate when changing from 0 or when updating to a lower speed limit.
    # - After a predefined period of time when increasing speed limit.
    if self._speed_limit != self._speed_limit_set:
      if self._speed_limit == 0.0 or self._speed_limit_set < self._speed_limit:
        self._speed_limit = self._speed_limit_set
      elif time > self._last_speed_limit_set_change_ts + _WAIT_TIME_LIMIT_RISE:
        self._speed_limit = self._speed_limit_set
    # Update current velocity offset (error)
    self._v_offset = self.speed_limit - self._v_ego
    # Update change tracking variables
    self._speed_limit_changed = self._speed_limit != self._speed_limit_prev
    self._v_cruise_setpoint_changed = self._v_cruise_setpoint != self._v_cruise_setpoint_prev
    self._speed_limit_set_change = self._speed_limit_set - self._speed_limit_set_prev
    self._speed_limit_prev = self._speed_limit
    self._v_cruise_setpoint_prev = self._v_cruise_setpoint
    self._speed_limit_set_prev = self._speed_limit_set

  def _state_transition(self):
    self._state_prev = self._state
    # In any case, if op is disabled, or speed limit control is disabled
    # or the reported speed limit is 0, deactivate.
    if not self._op_enabled or not self._is_enabled or self._speed_limit == 0:
      self.state = SpeedLimitControlState.inactive
      return

    # inactive
    if self.state == SpeedLimitControlState.inactive:
      # If the limit speed offset is negative (i.e. reduce speed) and lower than threshold
      # we go to adapting state to quickly reduce speed, otherwise we go directly to active
      if self._v_offset < _SPEED_OFFSET_TH:
        self.state = SpeedLimitControlState.adapting
      else:
        self.state = SpeedLimitControlState.active
    # tempInactive
    elif self.state == SpeedLimitControlState.tempInactive:
      # if speed limit changes, transition to inactive,
      # proper active state will be set on next iteration.
      if self._speed_limit_changed:
        self.state = SpeedLimitControlState.inactive
    # adapting
    elif self.state == SpeedLimitControlState.adapting:
      self._adapting_cycles += 1
      # If user changes the cruise speed, deactivate temporarely
      if self._v_cruise_setpoint_changed:
        self.state = SpeedLimitControlState.tempInactive
      # Go to active once the speed offset is over threshold.
      elif self._v_offset >= _SPEED_OFFSET_TH:
        self.state = SpeedLimitControlState.active
    # active
    elif self.state == SpeedLimitControlState.active:
      # If user changes the cruise speed, deactivate temporarely
      if self._v_cruise_setpoint_changed:
        self.state = SpeedLimitControlState.tempInactive
      # Go to adapting if the speed offset goes below threshold.
      elif self._v_offset < _SPEED_OFFSET_TH:
        self.state = SpeedLimitControlState.adapting

  def _update_solution(self):
    # inactive
    if self.state == SpeedLimitControlState.inactive:
      # Preserve values
      self.v_limit = self._v_ego
      self.a_limit = self._a_ego
      self.v_limit_future = self._v_ego
    # adapting
    elif self.state == SpeedLimitControlState.adapting:
      # Calculate to adapt speed on target time.
      adapting_time = max(_LIMIT_ADAPT_TIME - self._adapting_cycles * _LON_MPC_STEP, 1.0)  # min adapt time 1 sec.
      a_target = (self.speed_limit - self._v_ego) / adapting_time
      # smooth out acceleration using jerk limits.
      j_limits = np.array(self._adapting_jerk_limits)
      a_limits = self._a_ego + j_limits * _LON_MPC_STEP
      a_target = max(min(a_target, a_limits[1]), a_limits[0])
      # calculate the solution values
      self.a_limit = max(a_target, _MIN_ADAPTING_BRAKE_ACC)  # acceleration in next Longitudinal control step.
      self.v_limit = self._v_ego + self.a_limit * _LON_MPC_STEP  # speed in next Longitudinal control step.
      self.v_limit_future = self._v_ego + self.a_limit * 4.  # speed in 4 seconds.
    # active
    elif self.state == SpeedLimitControlState.active:
      # Calculate following same cruise logic in planner.py
      self.v_limit, self.a_limit = speed_smoother(self._v_ego, self._a_ego, self.speed_limit,
                                                  self._active_accel_limits[1], self._active_accel_limits[0],
                                                  self._active_jerk_limits[1], self._active_jerk_limits[0],
                                                  _LON_MPC_STEP)
      self.v_limit = max(self.v_limit, 0.)
      self.v_limit_future = self._speed_limit

  def _update_events(self, events):
    if not self.is_active:
      # no event while inactive or deactivating
      return

    if self._state_prev <= SpeedLimitControlState.tempInactive:
      events.add(EventName.speedLimitActive)
    elif self._speed_limit_set_change > 0:
      events.add(EventName.speedLimitIncrease)
    elif self._speed_limit_set_change < 0:
      events.add(EventName.speedLimitDecrease)

  def update(self, enabled, v_ego, a_ego, CS, v_cruise_setpoint, accel_limits, jerk_limits, events=Events()):
    self._op_enabled = enabled
    self._v_ego = v_ego
    self._a_ego = a_ego
    self._speed_limit_set = CS.cruiseState.speedLimit
    self._v_cruise_setpoint = v_cruise_setpoint
    self._active_accel_limits = accel_limits
    self._active_jerk_limits = jerk_limits

    self._update_params()
    self._update_calculations()
    self._state_transition()
    self._update_solution()
    self._update_events(events)

  def deactivate(self):
    self.state = SpeedLimitControlState.inactive
