import time
import cereal.messaging_arne as messaging_arne

sm = messaging_arne.SubMaster(['dynamicFollowButton'])
while True:
  sm.update(0)
  print(sm['dynamicFollowButton'].status)
  time.sleep(1)