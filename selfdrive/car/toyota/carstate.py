import numpy as np
from common.numpy_fast import interp
import math
import time
from math import floor
from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.toyota.values import CAR, DBC, STEER_THRESHOLD, TSS2_CAR, NO_STOP_TIMER_CAR
from common.params import Params, put_nonblocking
import cereal.messaging as messaging
from common.travis_checker import travis
from common.op_params import opParams

op_params = opParams()
rsa_max_speed = op_params.get('rsa_max_speed')
limit_rsa = op_params.get('limit_rsa')
set_speed_offset = op_params.get('set_speed_offset')
physical_buttons_AP = op_params.get('physical_buttons_AP')
physical_buttons_DF = op_params.get('physical_buttons_DF')
physical_buttons_LKAS = op_params.get('physical_buttons_LKAS')

# dp
#DP_OFF = 0
DP_ECO = 1
DP_NORMAL = 2
DP_SPORT = 3

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["GEAR_PACKET"]['GEAR']

    # dp
    self.dp_toyota_zss = Params().get('dp_toyota_zss') == b'1'

    # All TSS2 car have the accurate sensor
    self.accurate_steer_angle_seen = CP.carFingerprint in TSS2_CAR or CP.carFingerprint in [CAR.LEXUS_ISH] or self.dp_toyota_zss
    self.setspeedcounter = 0
    self.pcm_acc_active = False
    self.lkas = 1
    self.main_on = False
    self.gas_pressed = False
    self.smartspeed = 0
    self.leftblindspot = False
    self.leftblindspotD1 = 0
    self.leftblindspotD2 = 0
    self.rightblindspot = False
    self.rightblindspotD1 = 0
    self.rightblindspotD2 = 0
    self.rightblindspotcounter = 0
    self.leftblindspotcounter = 0
    self.Angles = np.zeros(250)
    self.Angles_later = np.zeros(250)
    self.Angle_counter = 0
    self.Angle = [0, 5, 10, 15,20,25,30,35,60,100,180,270,500]
    self.Angle_Speed = [255,160,100,80,70,60,55,50,40,33,27,17,12]
    self.v_cruise_pcmactivated = False
    self.v_cruise_pcmlast = 0
    self.setspeedoffset = 34
    self.setspeedcounter = 0
    self.spdval1 = 0
    self.distance = 0
    self.engineRPM = 0
    self.read_distance_lines = 0
    if not travis:
      self.pm = messaging.PubMaster(['liveTrafficData'])
      self.sm = messaging.SubMaster(['liveMapData','dragonConf','latControl'])#',latControl',])
    # On NO_DSU cars but not TSS2 cars the cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']
    # is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.needs_angle_offset = True #CP.carFingerprint not in TSS2_CAR or CP.carFingerprint in [CAR.LEXUS_ISH] or self.dp_toyota_zss
    self.angle_offset = 0.

  def update(self, cp, cp_cam, frame):
    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["SEATS_DOORS"]['DOOR_OPEN_FL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_FR'],
                        cp.vl["SEATS_DOORS"]['DOOR_OPEN_RL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_RR']])
    ret.seatbeltUnlatched = cp.vl["SEATS_DOORS"]['SEATBELT_DRIVER_UNLATCHED'] != 0

    ret.brakePressed = (cp.vl["BRAKE_MODULE"]['BRAKE_PRESSED'] != 0) or not bool(cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE'])
    ret.brakeLights = bool(cp.vl["ESP_CONTROL"]['BRAKE_LIGHTS_ACC'] or ret.brakePressed)
    if self.CP.enableGasInterceptor:
      ret.gas = (cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS'] + cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS2']) / 2.
      ret.gasPressed = ret.gas > 15
    elif self.CP.carFingerprint in [CAR.LEXUS_ISH, CAR.LEXUS_GSH]:
      ret.gas = cp.vl["GAS_PEDAL_ALT"]['GAS_PEDAL']
      ret.gasPressed = ret.gas > 1e-5
    else:
      ret.gas = cp.vl["GAS_PEDAL"]['GAS_PEDAL']
      ret.gasPressed = cp.vl["PCM_CRUISE"]['GAS_RELEASED'] == 0

    ret.wheelSpeeds.fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.001

    # Some newer models have a more accurate angle measurement in the TORQUE_SENSOR message. Use if non-zero
    if abs(cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']) > 1e-3:
      self.accurate_steer_angle_seen = True

    if self.accurate_steer_angle_seen:
      if self.dp_toyota_zss:
        ret.steeringAngleDeg = cp.vl["SECONDARY_STEER_ANGLE"]['ZORRO_STEER'] - self.angle_offset
      else:
        ret.steeringAngleDeg = cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE'] - self.angle_offset

      if self.needs_angle_offset:
        angle_wheel = cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE'] + cp.vl["STEER_ANGLE_SENSOR"]['STEER_FRACTION']
        if (abs(angle_wheel) > 1e-3 and abs(ret.steeringAngle) > 1e-3) or self.dp_toyota_zss:
          self.needs_angle_offset = False
          self.angle_offset = ret.steeringAngleDeg - angle_wheel
    else:
      ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE'] + cp.vl["STEER_ANGLE_SENSOR"]['STEER_FRACTION']

    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]['STEER_RATE']

    can_gear = int(cp.vl["GEAR_PACKET"]['GEAR'])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    dp_profile = 0

    if not travis:
      self.sm.update(0)
      self.smartspeed = self.sm['liveMapData'].speedLimit
      dp_profile = self.sm['dragonConf'].dpAccelProfile
    if not travis and self.sm.updated['latControl'] and ret.vEgo > 11.0:
      angle_later = self.sm['latControl'].anglelater
    else:
      angle_later = 0
    if self.CP.carFingerprint in [CAR.COROLLAH_TSS2, CAR.LEXUS_ESH_TSS2, CAR.RAV4H_TSS2, CAR.CHRH, CAR.PRIUS_TSS2, CAR.HIGHLANDERH_TSS2]:
      sport_on = cp.vl["GEAR_PACKET2"]['SPORT_ON']
      econ_on = cp.vl["GEAR_PACKET2"]['ECON_ON']
    else:
      try:
        econ_on = cp.vl["GEAR_PACKET"]['ECON_ON']
      except KeyError:
        econ_on = 0
      if self.CP.carFingerprint == CAR.RAV4_TSS2:
        sport_on = cp.vl["GEAR_PACKET"]['SPORT_ON_2']
      else:
        try:
          sport_on = cp.vl["GEAR_PACKET"]['SPORT_ON']
        except KeyError:
          sport_on = 0
    if physical_buttons_LKAS:
      self.lkas = cp_cam.vl["LKAS_HUD"]['SET_ME_X01']
    if not travis and physical_buttons_AP:
      if econ_on == 1 and dp_profile !=  DP_ECO:
        if int(Params().get('dp_accel_profile')) != DP_ECO:
          put_nonblocking('dp_accel_profile',str(DP_ECO))
          put_nonblocking('dp_last_modified',str(floor(time.time())))
      if sport_on == 1 and dp_profile !=  DP_SPORT:
        if int(Params().get('dp_accel_profile')) != DP_SPORT:
          put_nonblocking('dp_accel_profile',str(DP_SPORT))
          put_nonblocking('dp_last_modified',str(floor(time.time())))
      if sport_on == 0 and econ_on == 0 and dp_profile !=  DP_NORMAL:
        if int(Params().get('dp_accel_profile')) != DP_NORMAL:
          put_nonblocking('dp_accel_profile',str(DP_NORMAL))
          put_nonblocking('dp_last_modified',str(floor(time.time())))
    #Arne Blindspot code.
    if frame > 999 and self.CP.carFingerprint in [CAR.RAV4H, CAR.HIGHLANDER]:#not (self.CP.carFingerprint in TSS2_CAR or self.CP.carFingerprint == CAR.CAMRY or self.CP.carFingerprint == CAR.CAMRYH):
      if cp.vl["DEBUG"]['BLINDSPOTSIDE']==65: #Left
        if cp.vl["DEBUG"]['BLINDSPOTD1'] != self.leftblindspotD1:
          self.leftblindspotD1 = cp.vl["DEBUG"]['BLINDSPOTD1']
          self.leftblindspotcounter = 21
        if cp.vl["DEBUG"]['BLINDSPOTD2'] != self.leftblindspotD2:
          self.leftblindspotD2 = cp.vl["DEBUG"]['BLINDSPOTD2']
          self.leftblindspotcounter = 21
        if (self.leftblindspotD1 > 10) or (self.leftblindspotD2 > 10):
          self.leftblindspot = bool(1)
          #print("Left Blindspot Detected")
      elif  cp.vl["DEBUG"]['BLINDSPOTSIDE']==66: #Right
        if cp.vl["DEBUG"]['BLINDSPOTD1'] != self.rightblindspotD1:
          self.rightblindspotD1 = cp.vl["DEBUG"]['BLINDSPOTD1']
          self.rightblindspotcounter = 21
        if cp.vl["DEBUG"]['BLINDSPOTD2'] != self.rightblindspotD2:
          self.rightblindspotD2 = cp.vl["DEBUG"]['BLINDSPOTD2']
          self.rightblindspotcounter = 21
        if (self.rightblindspotD1 > 10) or (self.rightblindspotD2 > 10):
          self.rightblindspot = bool(1)
          #print("Right Blindspot Detected")
      self.rightblindspotcounter = self.rightblindspotcounter -1 if self.rightblindspotcounter > 0 else 0
      self.leftblindspotcounter = self.leftblindspotcounter -1 if self.leftblindspotcounter > 0 else 0
      if self.leftblindspotcounter == 0:
        self.leftblindspot = False
        self.leftblindspotD1 = 0
        self.leftblindspotD2 = 0
      if self.rightblindspotcounter == 0:
        self.rightblindspot = False
        self.rightblindspotD1 = 0
        self.rightblindspotD2 = 0
    elif frame > 999 and self.CP.carFingerprint in TSS2_CAR or self.CP.carFingerprint == CAR.AVALON_2021:
      self.leftblindspot = cp.vl["BSM"]['L_ADJACENT'] == 1
      self.leftblindspotD1 = 10.1
      self.leftblindspotD2 = 10.1
      self.rightblindspot = cp.vl["BSM"]['R_ADJACENT'] == 1
      self.rightblindspotD1 = 10.1
      self.rightblindspotD2 = 10.1
    #Arne Distance button read and write code.
    if self.read_distance_lines != cp.vl["PCM_CRUISE_SM"]['DISTANCE_LINES'] and physical_buttons_DF:
      self.read_distance_lines = cp.vl["PCM_CRUISE_SM"]['DISTANCE_LINES']
      put_nonblocking('dp_dynamic_follow', str(int(max(self.read_distance_lines, 0))))
      put_nonblocking('dp_last_modified',str(floor(time.time())))

    ret.leftBlindspot = self.leftblindspot
    ret.rightBlindspot = self.rightblindspot

    ret.leftBlinker = cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 1
    ret.rightBlinker = cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 2
    self.engineRPM = cp.vl["ENGINE_RPM"]['RPM']

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_DRIVER']
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_EPS']
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steerWarning = cp.vl["EPS_STATUS"]['LKA_STATE'] not in [1, 5]

    if self.CP.carFingerprint in [CAR.LEXUS_IS, CAR.LEXUS_NXT]:
      self.main_on = cp.vl["DSU_CRUISE"]['MAIN_ON'] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]['SET_SPEED']
      self.low_speed_lockout = False
    elif self.CP.carFingerprint in [CAR.LEXUS_ISH, CAR.LEXUS_GSH]:
      self.main_on = cp.vl["PCM_CRUISE_ALT"]['MAIN_ON'] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_ALT"]['SET_SPEED']
      self.low_speed_lockout = False
    else:
      self.main_on = cp.vl["PCM_CRUISE_2"]['MAIN_ON'] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]['SET_SPEED']
      self.low_speed_lockout = cp.vl["PCM_CRUISE_2"]['LOW_SPEED_LOCKOUT'] == 2
      ret.cruiseState.available = self.main_on
    #print("ret.cruiseState.speed =" + str(ret.cruiseState.speed))
    if self.CP.carFingerprint in TSS2_CAR:
      minimum_set_speed = 27
    elif self.CP.carFingerprint == CAR.RAV4:
      minimum_set_speed = 44
    else:
      minimum_set_speed = 41
    maximum_set_speed = 169
    if self.CP.carFingerprint == CAR.LEXUS_RXH:
      maximum_set_speed = 177
    v_cruise_pcm_max = ret.cruiseState.speed
    if v_cruise_pcm_max < minimum_set_speed and self.pcm_acc_active:
      #print("Min set speed changed. Was " + str(minimum_set_speed) + ", now " + str(v_cruise_pcm_max))
      minimum_set_speed = v_cruise_pcm_max
    if v_cruise_pcm_max > maximum_set_speed and self.pcm_acc_active:
      #print("Max set speed changed. Was " + str(maximum_set_speed) + ", now " + str(v_cruise_pcm_max))
      maximum_set_speed = v_cruise_pcm_max
    speed_range = maximum_set_speed - minimum_set_speed
    #if self.v_cruise_pcmactivated:
      #print("self.v_cruise_pcmlast after activated = " + str(self.v_cruise_pcmlast))
      #print("ret.cruiseState.speed  after activated = " + str(ret.cruiseState.speed))
    if (self.v_cruise_pcmactivated or (bool(cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE']) and not
                                       self.pcm_acc_active)) and self.v_cruise_pcmlast != ret.cruiseState.speed:
      #print("Engage with different speed than before")
      if ret.vEgo * CV.MS_TO_KPH < minimum_set_speed:
        #print("speed lower than min_set_speed")
        self.setspeedoffset = max(min(int(minimum_set_speed - ret.vEgo * CV.MS_TO_KPH),(minimum_set_speed-7.0)),0.0)
        #print("self.setspeedoffset = " + str (self.setspeedoffset))
        self.v_cruise_pcmlast = ret.cruiseState.speed
      else:
        #print("speed is higher than min_set_speed")
        self.setspeedoffset = 0
        #print("self.setspeedoffset = " + str (self.setspeedoffset))
        self.v_cruise_pcmlast = ret.cruiseState.speed
    if ret.cruiseState.speed < self.v_cruise_pcmlast and (bool(cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE']) and self.pcm_acc_active):
      #print("Speed lowered")
      if self.setspeedcounter > 0 and ret.cruiseState.speed > minimum_set_speed:
        self.setspeedoffset = self.setspeedoffset + 4
        #print("Speed lowered by 5")
        #print("self.setspeedoffset = " + str (self.setspeedoffset))
        #print("ret.cruiseState.speed = " + str(ret.cruiseState.speed) + " kph or " +  str(ret.cruiseState.speed - self.setspeedoffset) + " kph")
      else:
        if math.floor((int((-ret.cruiseState.speed)*(minimum_set_speed-7.0)/speed_range
                           + maximum_set_speed * (minimum_set_speed - 7.0)/speed_range)
                       - self.setspeedoffset)/(ret.cruiseState.speed - (minimum_set_speed-1.0))) > 0:
          self.setspeedoffset = self.setspeedoffset + math.floor((int((-ret.cruiseState.speed)*(minimum_set_speed - 7.0)/speed_range
                                                                      + maximum_set_speed * (minimum_set_speed - 7.0)/speed_range)
                                                                  - self.setspeedoffset)/(ret.cruiseState.speed - (minimum_set_speed - 1.0)))
          #print("Speed lowered, self.setspeedoffset is now " + str(self.setspeedoffset))
        #print("ret.cruiseState.speed = " + str(ret.cruiseState.speed) + " kph or " +  str(ret.cruiseState.speed - self.setspeedoffset) + " kph")
      self.setspeedcounter = 50
    if self.v_cruise_pcmlast < ret.cruiseState.speed and (bool(cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE']) and self.pcm_acc_active):
      #print("Speed raised")
      if self.setspeedcounter > 0 and (self.setspeedoffset - 4) > 0:
        #print("Speed raised by 5")
        #print("self.setspeedoffset = " + str (self.setspeedoffset))
        #print("ret.cruiseState.speed = " + str(ret.cruiseState.speed) + " kph or " +  str(ret.cruiseState.speed - self.setspeedoffset) + " kph")
        self.setspeedoffset = self.setspeedoffset - 4
      else:
        self.setspeedoffset = self.setspeedoffset + math.floor((int((-ret.cruiseState.speed) * (minimum_set_speed - 7.0)/speed_range
                                                                    + maximum_set_speed * (minimum_set_speed - 7.0)/speed_range)
                                                                - self.setspeedoffset)/(maximum_set_speed + 1.0 - ret.cruiseState.speed))
        #print("Speed raised, self.setspeedoffset is now " + str(self.setspeedoffset))
        #print("ret.cruiseState.speed = " + str(ret.cruiseState.speed) + " kph or " +  str(ret.cruiseState.speed - self.setspeedoffset) + " kph")
      self.setspeedcounter = 50
    if self.setspeedcounter > 0:
      self.setspeedcounter = self.setspeedcounter - 1
    if bool(cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE']) and not self.pcm_acc_active:
      #print("self.v_cruise_pcmlast on activated = " + str(self.v_cruise_pcmlast))
      #print("ret.cruiseState.speed  on activated = " + str(ret.cruiseState.speed))
      self.v_cruise_pcmactivated = True
    else:
      self.v_cruise_pcmactivated = False
    self.v_cruise_pcmlast = ret.cruiseState.speed
    if ret.cruiseState.speed - self.setspeedoffset < 7:
      #print("Set speed lower than 7 kph.")
      self.setspeedoffset = ret.cruiseState.speed - 7
      #print("self.setspeedoffset = " + str (self.setspeedoffset))
    if ret.cruiseState.speed - self.setspeedoffset > maximum_set_speed:
      #print("Set speed higher than max_set_speed")
      self.setspeedoffset = ret.cruiseState.speed - maximum_set_speed
      #print("self.setspeedoffset = " + str (self.setspeedoffset))

    if set_speed_offset or travis:
      self.setspeedoffset = 0.0

    #print("ret.cruiseState.speed before = " + str (ret.cruiseState.speed))
    ret.cruiseState.speed = min(max(7, ret.cruiseState.speed - self.setspeedoffset),v_cruise_pcm_max) * CV.KPH_TO_MS
    #print("ret.cruiseState.speed after = " + str(ret.cruiseState.speed) + " m/s or " +  str(round(ret.cruiseState.speed * CV.MS_TO_KPH)) + " kph")
    if not ret.leftBlinker and not ret.rightBlinker:
      self.Angles[self.Angle_counter] = abs(ret.steeringAngle)
      self.Angles_later[self.Angle_counter] = abs(angle_later)
    else:
      self.Angles[self.Angle_counter] = abs(ret.steeringAngle) * 0.8
      if ret.vEgo > 11.0:
        self.Angles_later[self.Angle_counter] = abs(angle_later) * 0.8
      else:
        self.Angles_later[self.Angle_counter] = 0.0

    if dp_profile == DP_ECO:
      factor = 1.0
    elif dp_profile == DP_SPORT:
      factor = 1.6
    else:
      factor = 1.3
    if not travis:
      ret.cruiseState.speed = float(min(ret.cruiseState.speed * CV.MS_TO_KPH, factor * interp(np.max(self.Angles), self.Angle, self.Angle_Speed)))* CV.KPH_TO_MS
      ret.cruiseState.speed = float(min(ret.cruiseState.speed * CV.MS_TO_KPH, factor * interp(np.max(self.Angles_later), self.Angle, self.Angle_Speed)))* CV.KPH_TO_MS
    self.Angle_counter = (self.Angle_counter + 1 ) % 250
    if self.CP.carFingerprint in [CAR.LEXUS_ISH, CAR.LEXUS_GSH]:
      # Lexus ISH does not have CRUISE_STATUS value (always 0), so we use CRUISE_ACTIVE value instead
      self.pcm_acc_status = cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE']
    else:
      self.pcm_acc_status = cp.vl["PCM_CRUISE"]['CRUISE_STATE']
    if self.CP.carFingerprint in NO_STOP_TIMER_CAR or self.CP.enableGasInterceptor:
      # ignore standstill in hybrid vehicles, since pcm allows to restart without
      # receiving any special command. Also if interceptor is detected
      ret.cruiseState.standstill = False
    else:
      ret.cruiseState.standstill = self.pcm_acc_status == 7

    self.pcm_acc_active = bool(cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE'])
    ret.cruiseState.enabled = self.pcm_acc_active and ret.cruiseState.speed > 0.0
    ret.cruiseState.nonAdaptive = cp.vl["PCM_CRUISE"]['CRUISE_STATE'] in [1, 2, 3, 4, 5, 6]

    if self.CP.carFingerprint == CAR.PRIUS:
      ret.genericToggle = cp.vl["AUTOPARK_STATUS"]['STATE'] != 0
    else:
      ret.genericToggle = bool(cp.vl["LIGHT_STALK"]['AUTO_HIGH_BEAM'])
    ret.stockAeb = bool(cp_cam.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_cam.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    ret.espDisabled = cp.vl["ESP_CONTROL"]['TC_DISABLED'] != 0
    # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = cp.vl["EPS_STATUS"]['LKA_STATE']

    self.distance = cp_cam.vl["ACC_CONTROL"]['DISTANCE']
    if self.CP.carFingerprint in [CAR.RAV4H, CAR.HIGHLANDER]:
      self.distance = cp.vl["SDSU"]['FD_BUTTON']
    if self.CP.carFingerprint in TSS2_CAR:
      ret.leftBlindspot = (cp.vl["BSM"]['L_ADJACENT'] == 1) or (cp.vl["BSM"]['L_APPROACHING'] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]['R_ADJACENT'] == 1) or (cp.vl["BSM"]['R_APPROACHING'] == 1)


    self.tsgn1 = cp_cam.vl["RSA1"]['TSGN1']
    if self.spdval1 != cp_cam.vl["RSA1"]['SPDVAL1']:
      self.rsa_ignored_speed = 0
    self.spdval1 = cp_cam.vl["RSA1"]['SPDVAL1']

    self.splsgn1 = cp_cam.vl["RSA1"]['SPLSGN1']
    self.tsgnhlt1 = cp_cam.vl["RSA1"]['TSGNHLT1']
    self.tsgn2 = cp_cam.vl["RSA1"]['TSGN2']
    #self.spdval2 = cp_cam.vl["RSA1"]['SPDVAL2']

    self.splsgn2 = cp_cam.vl["RSA1"]['SPLSGN2']
    self.tsgn3 = cp_cam.vl["RSA2"]['TSGN3']
    self.splsgn3 = cp_cam.vl["RSA2"]['SPLSGN3']
    self.tsgn4 = cp_cam.vl["RSA2"]['TSGN4']
    self.splsgn4 = cp_cam.vl["RSA2"]['SPLSGN4']
    self.noovertake = self.tsgn1 == 65 or self.tsgn2 == 65 or self.tsgn3 == 65 or self.tsgn4 == 65 or self.tsgn1 == 66 or self.tsgn2 == 66 or self.tsgn3 == 66 or self.tsgn4 == 66
    if (self.spdval1 > 0) and not (self.spdval1 == 35 and self.tsgn1 == 1) and self.rsa_ignored_speed != self.spdval1:
      dat =  messaging.new_message('liveTrafficData')
      if self.spdval1 > 0:
        dat.liveTrafficData.speedLimitValid = True
        if self.tsgn1 == 36:
          dat.liveTrafficData.speedLimit = self.spdval1 * 1.60934
        elif self.tsgn1 == 1:
          dat.liveTrafficData.speedLimit = self.spdval1
        else:
          dat.liveTrafficData.speedLimit = 0
      else:
        dat.liveTrafficData.speedLimitValid = False
      #if self.spdval2 > 0:
      #  dat.liveTrafficData.speedAdvisoryValid = True
      #  dat.liveTrafficData.speedAdvisory = self.spdval2
      #else:
      dat.liveTrafficData.speedAdvisoryValid = False
      if limit_rsa and rsa_max_speed < ret.vEgo:
        dat.liveTrafficData.speedLimitValid = False
      if not travis:
        self.pm.send('liveTrafficData', dat)
    if ret.gasPressed and not self.gas_pressed:
      self.engaged_when_gas_was_pressed = self.pcm_acc_active
    if ((ret.gasPressed) or (self.gas_pressed and not ret.gasPressed)) and self.engaged_when_gas_was_pressed and ret.vEgo > self.smartspeed:
      self.rsa_ignored_speed = self.spdval1
      dat = messaging.new_message('liveTrafficData')
      dat.liveTrafficData.speedLimitValid = True
      dat.liveTrafficData.speedLimit = ret.vEgo * 3.6
      if not travis:
        self.pm.send('liveTrafficData', dat)
    self.gas_pressed = ret.gasPressed

    return ret

  @staticmethod
  def get_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("STEER_ANGLE", "STEER_ANGLE_SENSOR", 0),
      ("GEAR", "GEAR_PACKET", 0),
      ("BRAKE_PRESSED", "BRAKE_MODULE", 0),
      ("GAS_PEDAL", "GAS_PEDAL", 0),
      ("RPM", "ENGINE_RPM", 0),
      ("WHEEL_SPEED_FL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_FR", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RR", "WHEEL_SPEEDS", 0),
      ("DOOR_OPEN_FL", "SEATS_DOORS", 1),
      ("DOOR_OPEN_FR", "SEATS_DOORS", 1),
      ("DOOR_OPEN_RL", "SEATS_DOORS", 1),
      ("DOOR_OPEN_RR", "SEATS_DOORS", 1),
      ("SEATBELT_DRIVER_UNLATCHED", "SEATS_DOORS", 1),
      ("TC_DISABLED", "ESP_CONTROL", 1),
      ("STEER_FRACTION", "STEER_ANGLE_SENSOR", 0),
      ("STEER_RATE", "STEER_ANGLE_SENSOR", 0),
      ("CRUISE_ACTIVE", "PCM_CRUISE", 0),
      ("CRUISE_STATE", "PCM_CRUISE", 0),
      ("GAS_RELEASED", "PCM_CRUISE", 1),
      ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR", 0),
      ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR", 0),
      ("STEER_ANGLE", "STEER_TORQUE_SENSOR", 0),
      ("TURN_SIGNALS", "STEERING_LEVERS", 3),   # 3 is no blinkers
      ("LKA_STATE", "EPS_STATUS", 0),
      ("BRAKE_LIGHTS_ACC", "ESP_CONTROL", 0),
      ("AUTO_HIGH_BEAM", "LIGHT_STALK", 0),
      ("SPORT_ON", "GEAR_PACKET", 0),
      ("ECON_ON", "GEAR_PACKET", 0),
      ("DISTANCE_LINES", "PCM_CRUISE_SM", 0),
    ]

    checks = [
      #("BRAKE_MODULE", 40),
      #("GAS_PEDAL", 33),
      ("WHEEL_SPEEDS", 80),
      ("STEER_ANGLE_SENSOR", 80),
      ("PCM_CRUISE", 33),
      ("STEER_TORQUE_SENSOR", 50),
      ("EPS_STATUS", 25),
    ]
    if CP.carFingerprint == CAR.RAV4_TSS2:
      signals.append(("SPORT_ON_2", "GEAR_PACKET", 0))

    if CP.carFingerprint in [CAR.COROLLAH_TSS2, CAR.LEXUS_ESH_TSS2, CAR.RAV4H_TSS2, CAR.CHRH, CAR.PRIUS_TSS2, CAR.HIGHLANDERH_TSS2]:
      signals.append(("SPORT_ON", "GEAR_PACKET2", 0))
      signals.append(("ECON_ON", "GEAR_PACKET2", 0))

    if CP.carFingerprint in [CAR.LEXUS_ISH, CAR.LEXUS_GSH]:
      signals.append(("GAS_PEDAL", "GAS_PEDAL_ALT", 0))
      signals.append(("MAIN_ON", "PCM_CRUISE_ALT", 0))
      signals.append(("SET_SPEED", "PCM_CRUISE_ALT", 0))
      signals.append(("AUTO_HIGH_BEAM", "LIGHT_STALK_ISH", 0))
      checks += [
        ("BRAKE_MODULE", 50),
        ("GAS_PEDAL_ALT", 50),
        ("PCM_CRUISE_ALT", 1),
      ]
    else:
      signals += [
        ("AUTO_HIGH_BEAM", "LIGHT_STALK", 0),
        ("GAS_PEDAL", "GAS_PEDAL", 0),
      ]
      checks += [
        ("BRAKE_MODULE", 40),
        ("GAS_PEDAL", 33),
      ]

    if CP.carFingerprint in [CAR.LEXUS_IS, CAR.LEXUS_NXT]:
      signals.append(("MAIN_ON", "DSU_CRUISE", 0))
      signals.append(("SET_SPEED", "DSU_CRUISE", 0))
      checks.append(("DSU_CRUISE", 5))
    else:
      signals.append(("MAIN_ON", "PCM_CRUISE_2", 0))
      signals.append(("SET_SPEED", "PCM_CRUISE_2", 0))
      signals.append(("LOW_SPEED_LOCKOUT", "PCM_CRUISE_2", 0))
      checks.append(("PCM_CRUISE_2", 33))

    if CP.carFingerprint == CAR.PRIUS:
      signals.append(("STATE", "AUTOPARK_STATUS", 0))

    if CP.carFingerprint in [CAR.RAV4H, CAR.HIGHLANDER]:
      signals.append(("FD_BUTTON", "SDSU", 0))
      signals.append(("BLINDSPOT","DEBUG", 0))
      signals.append(("BLINDSPOTSIDE","DEBUG",65))
      signals.append(("BLINDSPOTD1","DEBUG", 0))
      signals.append(("BLINDSPOTD2","DEBUG", 0))

    # add gas interceptor reading if we are using it
    if CP.enableGasInterceptor:
      signals.append(("INTERCEPTOR_GAS", "GAS_SENSOR", 0))
      signals.append(("INTERCEPTOR_GAS2", "GAS_SENSOR", 0))
      checks.append(("GAS_SENSOR", 50))

    if CP.carFingerprint in TSS2_CAR  or CP.carFingerprint == CAR.AVALON_2021:
      signals += [("L_ADJACENT", "BSM", 0)]
      signals += [("L_APPROACHING", "BSM", 0)]
      signals += [("R_ADJACENT", "BSM", 0)]
      signals += [("R_APPROACHING", "BSM", 0)]

    if Params().get('dp_toyota_zss') == b'1':
      signals += [("ZORRO_STEER", "SECONDARY_STEER_ANGLE", 0)]

    checks = []
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):

    signals = [
      ("FORCE", "PRE_COLLISION", 0),
      ("PRECOLLISION_ACTIVE", "PRE_COLLISION", 0),
      ("TSGN1", "RSA1", 0),
      ("SPDVAL1", "RSA1", 0),
      ("SPLSGN1", "RSA1", 0),
      ("TSGNHLT1", "RSA1", 0),
      ("TSGN2", "RSA1", 0),
      #("SPDVAL2", "RSA1", 0),
      ("SPLSGN2", "RSA1", 0),
      ("TSGN3", "RSA2", 0),
      ("SPLSGN3", "RSA2", 0),
      ("TSGN4", "RSA2", 0),
      ("SPLSGN4", "RSA2", 0),
      ("DISTANCE", "ACC_CONTROL", 0),
      ("SET_ME_X01", "LKAS_HUD", 0),
    ]

    # use steering message to check if panda is connected to frc
    checks = [
      ("STEERING_LKA", 42)
    ]
    checks = []
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)
