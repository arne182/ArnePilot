#!/usr/bin/env python3
import os
import json
from common.travis_checker import travis
from common.colors import opParams_error as error
from common.colors import opParams_warning as warning
try:
  from common.realtime import sec_since_boot
except ImportError:
  import time
  sec_since_boot = time.time
  warning("Using python time.time() instead of faster sec_since_boot")


class ValueTypes:
  number = [float, int]
  none_or_number = [type(None), float, int]


class Param:
  def __init__(self, default=None, allowed_types=[], description=None, live=False, hidden=False):
    self.default = default
    if not isinstance(allowed_types, list):
      allowed_types = [allowed_types]
    self.allowed_types = allowed_types
    self.description = description
    self.hidden = hidden
    self.live = live
    self._create_attrs()

  def is_valid(self, value):
    if not self.has_allowed_types:
      return True
    return type(value) in self.allowed_types

  def _create_attrs(self):  # Create attributes and check Param is valid
    self.has_allowed_types = isinstance(self.allowed_types, list) and len(self.allowed_types) > 0
    self.has_description = self.description is not None
    self.is_list = list in self.allowed_types
    if self.has_allowed_types:
      assert type(self.default) in self.allowed_types, 'Default value type must be in specified allowed_types!'
    if self.is_list:
      self.allowed_types.remove(list)


class opParams:
  def __init__(self):
    """
      To add your own parameter to opParams in your fork, simply add a new entry in self.fork_params, instancing a new Param class with at minimum a default value.
      The allowed_types and description args are not required but highly recommended to help users edit their parameters with opEdit safely.
        - The description value will be shown to users when they use opEdit to change the value of the parameter.
        - The allowed_types arg is used to restrict what kinds of values can be entered with opEdit so that users can't crash openpilot with unintended behavior.
              (setting a param intended to be a number with a boolean, or viceversa for example)
          Limiting the range of floats or integers is still recommended when `.get`ting the parameter.
          When a None value is allowed, use `type(None)` instead of None, as opEdit checks the type against the values in the arg with `isinstance()`.
        - Finally, the live arg tells both opParams and opEdit that it's a live parameter that will change. Therefore, you must place the `op_params.get()` call in the update function so that it can update.

      Here's an example of a good fork_param entry:
      self.fork_params = {'camera_offset': Param(default=0.06, allowed_types=VT.number)}  # VT.number allows both floats and ints
    """

    VT = ValueTypes()
    self.fork_params = {'awareness_factor': Param(10., VT.number, 'Multiplier for the awareness times'),
                        'alca_min_speed': Param(20, VT.number, 'Speed limit to start ALC in MPH'),
                        'alca_nudge_required': Param(False, bool, "Require nudge to start ALC"),
                        'camera_offset': Param(0.06, VT.number, 'Your camera offset to use in lane_planner.py', live=True),
                        'curvature_factor': Param(1.2, VT.number, 'Multiplier for the curvature slowdown. Increase for less braking.'),
                        'cloak': Param(True, bool, "make comma believe you are on their fork"),
                        'default_brake_distance': Param(250.0, VT.number, 'Distance in m to start braking for mapped speeds.'),
                        'dynamic_follow': Param('normal', str, "Can be: ('close', 'normal', 'far'): Left to right increases in following distance.\n"
                                                               "All profiles support dynamic follow so you'll get your preferred distance while\n"
                                                               "retaining the smoothness and safety of dynamic follow!", live=True),
                        'force_pedal': Param(False, bool, "If openpilot isn't recognizing your comma pedal, set this to True"),
                        'global_df_mod': Param(None, VT.none_or_number, 'The multiplier for the current distance used by dynamic follow. The range is limited from 0.85 to 1.2\n'
                                                                        'Smaller values will get you closer, larger will get you farther\n'
                                                                        'This is multiplied by any profile that\'s active. Set to None to disable', live=True),
                        'hide_auto_df_alerts': Param(True, bool, 'Hides the alert that shows what profile the model has chosen'),
                        'keep_openpilot_engaged': Param(True, bool, 'True is stock behavior in this fork. False lets you use the brake and cruise control stalk to disengage as usual'),
                        'limit_rsa': Param(False, bool, "Switch off RSA above rsa_max_speed"),
                        'mpc_offset': Param(0.0, VT.number, 'Offset model braking by how many m/s. Lower numbers equals more model braking', live=True),
                        'offset_limit': Param(0, VT.number, 'Speed at which apk percent offset will work in m/s'),
                        'osm': Param(True, bool, 'Whether to use OSM for drives'),
                        'rolling_stop': Param(False, bool, 'If you do not want stop signs to go down to 0 kph enable this for 9kph slow down'),
                        'rsa_max_speed': Param(24.5, VT.number, 'Speed limit to ignore RSA in m/s'),
                        'smart_speed': Param(True, bool, 'Whether to use Smart Speed for drives above smart_speed_max_vego'),
                        'smart_speed_max_vego': Param(26.8, VT.number, 'Speed limit to ignore Smartspeed in m/s'),
                        'spairrowtuning': Param(False, bool, 'INDI Tuning for Corolla'),
                        'speed_offset': Param(0, VT.number, 'Speed limit offset in m/s', live=True),
                        'traffic_light_alerts': Param(False, bool, "Switch off the traffic light alerts"),
                        'traffic_lights': Param(False, bool, "Should Openpilot stop for traffic lights"),
                        'traffic_lights_without_direction': Param(False, bool, "Should Openpilot stop for traffic lights without a direction specified"),
                        'use_car_caching': Param(True, bool, 'Whether to use fingerprint caching'),
                        'min_TR': Param(None, VT.none_or_number, 'The minimum allowed following distance in seconds. Default is 0.9 seconds.\n'
                                                                 'The range is limited from 0.85 to 1.3. Set to None to disable', live=True),
                        'use_virtual_middle_line': Param(False, bool, 'For roads over 4m wide, hug right. For roads under 2m wide, hug left.'),
                        'uniqueID': Param(None, [type(None), str], 'User\'s unique ID'),
                        'autoUpdate': Param(True, bool, 'Whether to auto-update'),
                        'corolla_tss2_d_tuning': Param(False, bool, 'lateral tuning using PID w/ true derivative'),
                        #'prius_use_pid': Param(False, bool, 'This enables the PID lateral controller with new a experimental derivative tune\nFalse: stock INDI, True: TSS2-tuned PID'),
                        'lat_d': Param(9.0, VT.number, 'The lateral derivative gain, default is 9.0 for TSS2 Corolla. This is active at all speeds', live=True),
                        'ludicrous_mode': Param(False, bool, 'Double overall acceleration!')}

    self._params_file = '/data/op_params.json'
    self._backup_file = '/data/op_params_corrupt.json'
    self._last_read_time = sec_since_boot()
    self.read_frequency = 2.5  # max frequency to read with self.get(...) (sec)
    self._to_delete = ['reset_integral', 'log_data']  # a list of params you want to delete (unused)
    self._run_init()  # restores, reads, and updates params

  def _run_init(self):  # does first time initializing of default params
    # Two required parameters for opEdit
    self.fork_params['username'] = Param(None, [type(None), str, bool], 'Your identifier provided with any crash logs sent to Sentry.\nHelps the developer reach out to you if anything goes wrong')
    self.fork_params['op_edit_live_mode'] = Param(False, bool, 'This parameter controls which mode opEdit starts in', hidden=True)
    self.params = self._get_all_params(default=True)  # in case file is corrupted
    if travis:
      return

    if os.path.isfile(self._params_file):
      if self._read():
        to_write = self._add_default_params()  # if new default data has been added
        to_write |= self._delete_old()  # or if old params have been deleted
      else:  # backup and re-create params file
        error("Can't read op_params.json file, backing up to /data/op_params_corrupt.json and re-creating file!")
        to_write = True
        if os.path.isfile(self._backup_file):
          os.remove(self._backup_file)
        os.rename(self._params_file, self._backup_file)
    else:
      to_write = True  # user's first time running a fork with op_params, write default params

    if to_write:
      self._write()
      os.chmod(self._params_file, 0o764)

  def get(self, key=None, force_live=False):  # any params you try to get MUST be in fork_params
    param_info = self.param_info(key)
    self._update_params(param_info, force_live)

    if key is None:
      return self._get_all_params()

    self._check_key_exists(key, 'get')
    value = self.params[key]
    if param_info.is_valid(value):  # always valid if no allowed types, otherwise checks to make sure
      return value  # all good, returning user's value

    warning('User\'s value type is not valid! Returning default')  # somehow... it should always be valid
    return param_info.default  # return default value because user's value of key is not in allowed_types to avoid crashing openpilot

  def put(self, key, value):
    self._check_key_exists(key, 'put')
    if not self.param_info(key).is_valid(value):
      raise Exception('opParams: Tried to put a value of invalid type!')
    self.params.update({key: value})
    self._write()

  def delete(self, key):  # todo: might be obsolete. remove?
    if key in self.params:
      del self.params[key]
      self._write()

  def param_info(self, key):
    if key in self.fork_params:
      return self.fork_params[key]
    return Param()

  def _check_key_exists(self, key, met):
    if key not in self.fork_params or key not in self.params:
      raise Exception('opParams: Tried to {} an unknown parameter! Key not in fork_params: {}'.format(met, key))

  def _add_default_params(self):
    added = False
    for key, param in self.fork_params.items():
      if key not in self.params:
        self.params[key] = param.default
        added = True
      elif not param.is_valid(self.params[key]):
        warning('Value type of user\'s {} param not in allowed types, replacing with default!'.format(key))
        self.params[key] = param.default
        added = True
    return added

  def _delete_old(self):
    deleted = False
    for param in self._to_delete:
      if param in self.params:
        del self.params[param]
        deleted = True
    return deleted

  def _get_all_params(self, default=False, return_hidden=False):
    if default:
      return {k: p.default for k, p in self.fork_params.items()}
    return {k: self.params[k] for k, p in self.fork_params.items() if k in self.params and (not p.hidden or return_hidden)}

  def _update_params(self, param_info, force_live):
    if force_live or param_info.live:  # if is a live param, we want to get updates while openpilot is running
      if not travis and sec_since_boot() - self._last_read_time >= self.read_frequency:  # make sure we aren't reading file too often
        if self._read():
          self._last_read_time = sec_since_boot()

  def _read(self):
    try:
      with open(self._params_file, "r") as f:
        self.params = json.loads(f.read())
      return True
    except Exception as e:
      error(e)
      return False

  def _write(self):
    if not travis:
      with open(self._params_file, "w") as f:
        f.write(json.dumps(self.params, indent=2))  # can further speed it up by remove indentation but makes file hard to read
