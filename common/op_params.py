import os
import json
import time
import string
import random
from common.travis_checker import travis


def write_params(params, params_file):
  if not travis:
    with open(params_file, "w") as f:
      json.dump(params, f, indent=2, sort_keys=True)
    os.chmod(params_file, 0o764)


def read_params(params_file, default_params):
  try:
    with open(params_file, "r") as f:
      params = json.load(f)
    return params, True
  except:
    params = default_params
    return params, False


class opParams:
  def __init__(self):
    self.params = {}
    self.params_file = "/data/op_params.json"
    self.kegman_file = "/data/kegman.json"
    self.last_read_time = time.time()
    self.read_timeout = 1.0  # max frequency to read with self.get(...) (sec)
    self.default_params = {'camera_offset': 0.06, 'awareness_factor': 2.0, 'lane_hug_direction': None,
                           'lane_hug_mod': 1.2, 'lane_hug_angle': 10, 'use_car_caching': True, 'osm': True,
                           'speed_offset': 0, 'keep_openpilot_engaged': True, 'force_pedal': False}
    self.force_update = False  # replaces values with default params if True, not just add add missing key/value pairs
    self.run_init()  # restores, reads, and updates params

  def create_id(self):  # creates unique identifier to send with sentry errors. please update uniqueID with op_edit.py to your username!
    need_id = False
    if "uniqueID" not in self.params:
      need_id = True
    if "uniqueID" in self.params and self.params["uniqueID"] is None:
      need_id = True
    if need_id:
      random_id = ''.join([random.choice(string.ascii_lowercase + string.ascii_uppercase + string.digits) for i in range(15)])
      self.params["uniqueID"] = random_id

  def add_default_params(self):
    prev_params = dict(self.params)
    if not travis:
      self.create_id()
      for i in self.default_params:
        if self.force_update:
          self.params.update({i: self.default_params[i]})
        elif i not in self.params:
          self.params.update({i: self.default_params[i]})
    return prev_params == self.params

  def run_init(self):  # does first time initializing of default params, and/or restoring from kegman.json
    if travis:
      print("Travis detected...")
      self.params = self.default_params
      self.add_default_params()
      return
    self.params = self.default_params  # in case any file is corrupted
    to_write = False
    no_params = False
    if os.path.isfile(self.params_file):
      self.params, read_status = read_params(self.params_file, self.default_params)
      if read_status:
        to_write = not self.add_default_params()  # if new default data has been added
      else:  # don't overwrite corrupted params, just print to screen
        print("ERROR: Can't read op_params.json file")
    elif os.path.isfile(self.kegman_file):
      to_write = True  # write no matter what
      try:
        with open(self.kegman_file, "r") as f:  # restore params from kegman
          self.params = json.load(f)
          self.add_default_params()
      except:
        print("ERROR: Can't read kegman.json file")
    else:
      no_params = True  # user's first time running a fork with kegman_conf or op_params
    if to_write or no_params:
      write_params(self.params, self.params_file)

  def put(self, key, value):
    self.params.update({key: value})
    write_params(self.params, self.params_file)

  def get(self, key=None, default=None):  # can specify a default value if key doesn't exist
    if (time.time() - self.last_read_time) >= self.read_timeout and not travis:  # make sure we aren't reading file too often
      self.params, read_status = read_params(self.params_file, self.default_params)
      self.last_read_time = time.time()
    if key is None:  # get all
      return self.params
    else:
      return self.params[key] if key in self.params else default

  def delete(self, key):
    if key in self.params:
      del self.params[key]
      write_params(self.params, self.params_file)
