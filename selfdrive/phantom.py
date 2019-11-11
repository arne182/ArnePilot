import zmq
import selfdrive.messaging_arne as messaging_arne
from selfdrive.services import service_list
from common.op_params import opParams
import subprocess
from common.travis_checker import travis
import time


class Phantom:
  def __init__(self, timeout=1.):
    context = zmq.Context()
    self.op_params = opParams()
    self.phantom_Data_sock = messaging_arne.sub_sock('phantomData', conflate=True)
    self.data = {"status": False, "speed": 0.0}
    self.last_receive_time = time.time()
    self.last_phantom_data = {"status": False, "speed": 0.0}
    self.timeout = timeout  # in seconds
    self.to_disable = True
    if not travis and not self.op_params.get("UseDNS", None):  # ensure we only run once
      self.mod_sshd_config()

  def update(self):
    phantom_data = messaging_arne.recv_one_or_none(self.phantom_Data_sock)
    if phantom_data is not None:
      self.data = {"status": phantom_data.phantomData.status, "speed": phantom_data.phantomData.speed,
                   "angle": phantom_data.phantomData.angle, "time": phantom_data.phantomData.time}
      self.last_phantom_data = dict(self.data)
      self.last_receive_time = time.time()
      self.to_disable = not self.data["status"]
    else:
      if self.to_disable:  # if last message is status: False, disable phantom mode, also disable by default
        self.data = {"status": False, "speed": 0.0}
      elif time.time() - self.last_receive_time >= self.timeout:  # lost connection, don't disable. keep phantom on but set speed to 0
        self.data = {"status": True, "speed": 0.0, "angle": 0.0, "time": 0.0, "lost_connection": True}
      else:  # if waiting between messages from app, message becomes none, this uses the data from last message
        self.data = dict(self.last_phantom_data)

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
