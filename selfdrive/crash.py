"""Install exception handler for process crash."""
import os
import sys
import capnp
import requests
from cereal import car
from common.params import Params
from selfdrive.version import version, dirty, origin, branch

from selfdrive.hardware import PC
from selfdrive.swaglog import cloudlog

def save_exception(exc_text):
  i = 0
  log_file = '{}/{}'.format(CRASHES_DIR, datetime.now().strftime('%Y-%m-%d--%H-%M-%S.%f.log')[:-3])
  if os.path.exists(log_file):
    while os.path.exists(log_file + str(i)):
      i += 1
    log_file += str(i)
  with open(log_file, 'w') as f:
    f.write(exc_text)
  print('Logged current crash to {}'.format(log_file))

if os.getenv("NOLOG") or os.getenv("NOCRASH") or PC:
  def capture_exception(*args, **kwargs):
    pass

  def bind_user(**kwargs):
    pass

  def bind_extra(**kwargs):
    pass

else:
  import sentry_sdk
  from sentry_sdk.integrations.threading import ThreadingIntegration
  from common.op_params import opParams
  from datetime import datetime

  ret = car.CarParams.new_message()
  candidate = ret.carFingerprint

  COMMUNITY_DIR = '/data/community'
  CRASHES_DIR = '{}/crashes'.format(COMMUNITY_DIR)

  if not os.path.exists(COMMUNITY_DIR):
    os.mkdir(COMMUNITY_DIR)
  if not os.path.exists(CRASHES_DIR):
    os.mkdir(CRASHES_DIR)

  params = Params()
  op_params = opParams()
  awareness_factor = op_params.get('awareness_factor')
  alca_min_speed = op_params.get('alca_min_speed')
  alca_nudge_required = op_params.get('alca_nudge_required')
  ArizonaMode = op_params.get('ArizonaMode')
  dynamic_gas_mod = op_params.get('dynamic_gas_mod')
  keep_openpilot_engaged = op_params.get('keep_openpilot_engaged')
  set_speed_offset = op_params.get('set_speed_offset')
  username = op_params.get('username')
  #uniqueID = op_params.get('uniqueID')
  try:
    dongle_id = params.get("DongleId").decode('utf8')
  except AttributeError:
    dongle_id = "None"
  try:
    ip = requests.get('https://checkip.amazonaws.com/').text.strip()
  except Exception:
    ip = "255.255.255.255"
  error_tags = {'dirty': dirty, 'dongle_id': dongle_id, 'branch': branch, 'remote': origin,
                'awareness_factor': awareness_factor, 'alca_min_speed': alca_min_speed,
                'alca_nudge_required': alca_nudge_required, 'ArizonaMode': ArizonaMode,
                'dynamic_gas_mod': dynamic_gas_mod, 'keep_openpilot_engaged': keep_openpilot_engaged,
                'set_speed_offset': set_speed_offset, 'fingerprintedAs': candidate}
  if username is None or not isinstance(username, str):
    username = 'undefined'
    #error_tags['uniqueID'] = uniqueID
  error_tags['username'] = username

  u_tag = []
  if isinstance(username, str):
    u_tag.append(username)
  if len(u_tag) > 0:
    error_tags['username'] = ''.join(u_tag)
  for k, v in error_tags.items():
    sentry_sdk.set_tag(k, v)
  def capture_exception(*args, **kwargs):
    exc_info = sys.exc_info()
    if not exc_info[0] is capnp.lib.capnp.KjException:
      sentry_sdk.capture_exception(*args, **kwargs)
      sentry_sdk.flush()  # https://github.com/getsentry/sentry-python/issues/291
    cloudlog.error("crash", exc_info=kwargs.get('exc_info', 1))

  def bind_user(**kwargs):
    sentry_sdk.set_user(kwargs)

  def capture_warning(warning_string):
    bind_user(id=dongle_id, ip_address=ip, username=username)
    sentry_sdk.capture_message(warning_string, level='warning')

  def capture_info(info_string):
    bind_user(id=dongle_id, ip_address=ip, username=username)
    sentry_sdk.capture_message(info_string, level='info')

  def bind_extra(**kwargs):
    for k, v in kwargs.items():
      sentry_sdk.set_tag(k, v)

  sentry_sdk.init("https://137e8e621f114f858f4c392c52e18c6d:8aba82f49af040c8aac45e95a8484970@sentry.io/1404547",
                  default_integrations=False, integrations=[ThreadingIntegration(propagate_hub=True)],
                  release=version)
