#!/usr/bin/env python3

# simple service that waits for network access and tries to update every hour

import datetime
import subprocess
import time
import os

from common.params import Params
from selfdrive.swaglog import cloudlog
from common.op_params import opParams
import selfdrive.messaging as messaging

op_params = opParams()

autoUpdate = op_params.get('autoUpdate', True)

NICE_LOW_PRIORITY = ["nice", "-n", "19"]
def main(gctx=None):
  params = Params()
  NEED_REBOOT = False
  sm = messaging.SubMaster(['thermal', 'health'])  # comma messaging still needs list even if it's just 1 service
  while True:
    # try network
    ping_failed = subprocess.call(["ping", "-W", "4", "-c", "1", "8.8.8.8"])
    if ping_failed:
      time.sleep(60)
      continue

    # download application update
    try:
      r = subprocess.check_output(NICE_LOW_PRIORITY + ["git", "fetch"], stderr=subprocess.STDOUT).decode('utf8')
    except subprocess.CalledProcessError as e:
      cloudlog.event("git fetch failed",
        cmd=e.cmd,
        output=e.output,
        returncode=e.returncode)
      time.sleep(60)
      continue
    cloudlog.info("git fetch success: %s", r)

    # Write update available param
    try:
      cur_hash = subprocess.check_output(["git", "rev-parse", "HEAD"]).rstrip()
      upstream_hash = subprocess.check_output(["git", "rev-parse", "@{u}"]).rstrip()
      params.put("UpdateAvailable", str(int(cur_hash != upstream_hash)))
      try:
        if autoUpdate and not os.path.isfile("/data/no_ota_updates") and cur_hash != upstream_hash:
          r = subprocess.check_output(NICE_LOW_PRIORITY + ["git", "pull"], stderr=subprocess.STDOUT)
          NEED_REBOOT = True        
      except subprocess.CalledProcessError as e:
        cloudlog.event("git pull failed",
          cmd=e.cmd,
          output=e.output,
          returncode=e.returncode)
        time.sleep(60)
        continue
      if NEED_REBOOT:
        cloudlog.info("git pull success: %s", r)
    except:
      params.put("UpdateAvailable", "0")

    # Write latest release notes to param
    try:
      r = subprocess.check_output(["git", "--no-pager", "show", "@{u}:RELEASES.md"])
      r = r[:r.find(b'\n\n')] # Slice latest release notes
      params.put("ReleaseNotes", r + b"\n")
    except:
      params.put("ReleaseNotes", "")

    t = datetime.datetime.now().isoformat()
    params.put("LastUpdateTime", t.encode('utf8'))
    
    if NEED_REBOOT:
      sm.update(1)
      if not sm['thermal'].started and not (sm['health'].ignitionLine or sm['health'].ignitionCan):
        os.system('reboot')

    time.sleep(30*60)

if __name__ == "__main__":
  main()
