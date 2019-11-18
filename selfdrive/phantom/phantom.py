import selfdrive.messaging_arne as messaging_arne
from common.op_params import opParams
import subprocess
from common.travis_checker import travis
import time


class Phantom:
  def __init__(self, timeout=1.):
    self.op_params = opParams()
    self.sm = messaging_arne.SubMaster('phantomData')
    self.data = {"status": False, "speed": 0.0}
    self.lost_connection = False
    self.last_receive_time = time.time()
    self.timeout = timeout  # in seconds
    if not travis and not self.op_params.get("UseDNS", None):  # ensure we only run once
      self.mod_sshd_config()

  def update(self):
    self.sm.update(0)
    phantom_data = self.sm['phantomData']
    self.data = {"status": phantom_data.status, "speed": phantom_data.speed, "angle": phantom_data.angle}

    if self.sm.updated['phantomData']:
      self.last_receive_time = time.time()

    if time.time() - self.last_receive_time >= self.timeout and self.data['status']:
      self.data = {"status": True, "speed": 0.0, "angle": 0.0}
      self.lost_connection = True
    else:
      self.lost_connection = False

  def __getitem__(self, s):
    return self.data[s]

  def mod_sshd_config(self):
    # this disables dns lookup when connecting to EON to speed up commands from phantom app, reboot required
    sshd_config_file = "/system/comma/usr/etc/ssh/sshd_config"
    try:
      # mount /system as rw so we can modify sshd_config file
      result = subprocess.check_call(["mount", "-o", "remount,rw", "/system"])
    except:
      result = 1
    if result == 0:
      with open(sshd_config_file, "r") as f:
        sshd_config = f.read()
      if "UseDNS no" not in sshd_config:
        use_dns = "{}UseDNS no\n"
        use_dns = use_dns.format("") if sshd_config[-1] == "\n" else use_dns.format("\n")
        with open(sshd_config_file, "w") as f:
          f.write(sshd_config + use_dns)
      self.op_params.put("UseDNS", True)
      try:
        subprocess.call(["mount", "-o", "remount,ro", "/system"])  # remount system as read only
      except:
        pass
    else:
      self.op_params.put("UseDNS", False)
