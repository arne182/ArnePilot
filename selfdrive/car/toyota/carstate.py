import numpy as np
from common.numpy_fast import interp
import math
from cereal import car
from common.numpy_fast import mean
import cereal.messaging_arne as messaging_arne
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from common.travis_checker import travis
from selfdrive.car.toyota.values import CAR, DBC, STEER_THRESHOLD, TSS2_CAR, NO_STOP_TIMER_CAR

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["GEAR_PACKET"]['GEAR']

    # All TSS2 car have the accurate sensor
    self.accurate_steer_angle_seen = CP.carFingerprint in TSS2_CAR

    # On NO_DSU cars but not TSS2 cars the cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']
    # is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.needs_angle_offset = CP.carFingerprint not in TSS2_CAR
    self.angle_offset = 0.
    self.pcm_acc_active = False
    self.main_on = False
    self.v_cruise_pcmlast = 41.0
    self.setspeedoffset = 34.0
    self.setspeedcounter = 0
    self.leftblindspot = False
    self.leftblindspotD1 = 0
    self.leftblindspotD2 = 0
    self.rightblindspot = False
    self.rightblindspotD1 = 0
    self.rightblindspotD2 = 0
    self.rightblindspotcounter = 0
    self.leftblindspotcounter = 0
    self.Angles = np.zeros(250)
    #self.Angles_later = np.zeros(250)
    self.Angle_counter = 0
    self.Angle = [0, 5, 10, 15,20,25,30,35,60,100,180,270,500]
    self.Angle_Speed = [255,160,100,80,70,60,55,50,40,33,27,17,12]
    if not travis:
      self.arne_pm = messaging_arne.PubMaster(['liveTrafficData', 'arne182Status'])


  def update(self, cp, cp_cam, frame):
    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["SEATS_DOORS"]['DOOR_OPEN_FL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_FR'],
                        cp.vl["SEATS_DOORS"]['DOOR_OPEN_RL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_RR']])
    ret.seatbeltUnlatched = cp.vl["SEATS_DOORS"]['SEATBELT_DRIVER_UNLATCHED'] != 0

    ret.brakePressed = cp.vl["BRAKE_MODULE"]['BRAKE_PRESSED'] != 0
    ret.brakeLights = bool(cp.vl["ESP_CONTROL"]['BRAKE_LIGHTS_ACC'] or ret.brakePressed)
    if self.CP.enableGasInterceptor:
      ret.gas = (cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS'] + cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS2']) / 2.
      ret.gasPressed = ret.gas > 15
    else:
      ret.gas = cp.vl["GAS_PEDAL"]['GAS_PEDAL']
      ret.gasPressed = ret.gas > 1e-5

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
      ret.steeringAngle = cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE'] - self.angle_offset

      if self.needs_angle_offset:
        angle_wheel = cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE'] + cp.vl["STEER_ANGLE_SENSOR"]['STEER_FRACTION']
        if abs(angle_wheel) > 1e-3 and abs(ret.steeringAngle) > 1e-3:
          self.needs_angle_offset = False
          self.angle_offset = ret.steeringAngle - angle_wheel
    else:
      ret.steeringAngle = cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE'] + cp.vl["STEER_ANGLE_SENSOR"]['STEER_FRACTION']

    ret.steeringRate = cp.vl["STEER_ANGLE_SENSOR"]['STEER_RATE']
    can_gear = int(cp.vl["GEAR_PACKET"]['GEAR'])

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    try:
      self.econ_on = cp.vl["GEAR_PACKET"]['ECON_ON']
    except:
      self.econ_on = 0
    try:
      self.sport_on = cp.vl["GEAR_PACKET"]['SPORT_ON']
    except:
      self.sport_on = 0
    if self.sport_on == 1:
      self.gasbuttonstatus = 1
    if self.econ_on == 1:
      self.gasbuttonstatus = 2
    if self.sport_on == 0 and self.econ_on == 0:
      self.gasbuttonstatus = 0
    msg = messaging_arne.new_message()
    msg.init('arne182Status')
    if frame > 999 and not (self.CP.carFingerprint in TSS2_CAR):
      if cp.vl["DEBUG"]['BLINDSPOTSIDE']==65: #Left
        if cp.vl["DEBUG"]['BLINDSPOTD1'] != self.leftblindspotD1:
          self.leftblindspotD1 = cp.vl["DEBUG"]['BLINDSPOTD1']
          self.leftblindspotcounter = 21
        if cp.vl["DEBUG"]['BLINDSPOTD2'] != self.leftblindspotD2:
          self.leftblindspotD2 = cp.vl["DEBUG"]['BLINDSPOTD2']
          self.leftblindspotcounter = 21
        if (self.leftblindspotD1 > 10) or (self.leftblindspotD2 > 10):
          self.leftblindspot = bool(1)
          print("Left Blindspot Detected")
      elif  cp.vl["DEBUG"]['BLINDSPOTSIDE']==66: #Right
        if cp.vl["DEBUG"]['BLINDSPOTD1'] != self.rightblindspotD1:
          self.rightblindspotD1 = cp.vl["DEBUG"]['BLINDSPOTD1']
          self.rightblindspotcounter = 21
        if cp.vl["DEBUG"]['BLINDSPOTD2'] != self.rightblindspotD2:
          self.rightblindspotD2 = cp.vl["DEBUG"]['BLINDSPOTD2']
          self.rightblindspotcounter = 21
        if (self.rightblindspotD1 > 10) or (self.rightblindspotD2 > 10):
          self.rightblindspot = bool(1)
          print("Right Blindspot Detected")
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
    elif frame > 999 and self.CP.carFingerprint in TSS2_CAR:
      self.leftblindspot = cp.vl["BSM"]['L_ADJACENT'] == 1
      self.leftblindspotD1 = 10.1
      self.leftblindspotD2 = 10.1
      self.rightblindspot = cp.vl["BSM"]['R_ADJACENT'] == 1
      self.rightblindspotD1 = 10.1
      self.rightblindspotD2 = 10.1
   
    msg.arne182Status.leftBlindspot = self.leftblindspot
    msg.arne182Status.rightBlindspot = self.rightblindspot
    msg.arne182Status.rightBlindspotD1 = self.rightblindspotD1
    msg.arne182Status.rightBlindspotD2 = self.rightblindspotD2
    msg.arne182Status.leftBlindspotD1 = self.leftblindspotD1
    msg.arne182Status.leftBlindspotD2 = self.leftblindspotD2
    msg.arne182Status.gasbuttonstatus = self.gasbuttonstatus
    if not travis:
      self.arne_pm.send('arne182Status', msg)
    self.left_blinker_on = cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 1
    self.right_blinker_on = cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 2

    ret.leftBlinker = cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 1
    ret.rightBlinker = cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 2

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_DRIVER']
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_EPS']
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    if self.CP.carFingerprint == CAR.LEXUS_IS:
      self.main_on = cp.vl["DSU_CRUISE"]['MAIN_ON'] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]['SET_SPEED'] * CV.KPH_TO_MS
      self.low_speed_lockout = False
    else:
      self.main_on = cp.vl["PCM_CRUISE_2"]['MAIN_ON'] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]['SET_SPEED'] * CV.KPH_TO_MS
      self.low_speed_lockout = cp.vl["PCM_CRUISE_2"]['LOW_SPEED_LOCKOUT'] == 2
    ret.cruiseState.available = self.main_on
    v_cruise_pcm_max = ret.cruiseState.speed
    if self.CP.carFingerprint in TSS2_CAR:
      minimum_set_speed = 27.0
    elif self.CP.carFingerprint == CAR.RAV4:
      minimum_set_speed = 44.0
    else:
      minimum_set_speed = 41.0
    if bool(cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE']) and not self.pcm_acc_active:
      if ret.vEgo < 12.5:
        self.setspeedoffset = max(min(int(minimum_set_speed-ret.vEgo*3.6),(minimum_set_speed-7.0)),0.0)
        self.v_cruise_pcmlast = ret.cruiseState.speed
      else:
        self.setspeedoffset = 0
        self.v_cruise_pcmlast = ret.cruiseState.speed
    if ret.cruiseState.speed < self.v_cruise_pcmlast:
      if self.setspeedcounter > 0 and ret.cruiseState.speed > minimum_set_speed:
        self.setspeedoffset = self.setspeedoffset + 4
      else:
        if math.floor((int((-ret.cruiseState.speed)*(minimum_set_speed-7.0)/(169.0-minimum_set_speed)  + 169.0*(minimum_set_speed-7.0)/(169.0-minimum_set_speed))-self.setspeedoffset)/(ret.cruiseState.speed-(minimum_set_speed-1.0))) > 0:
          self.setspeedoffset = self.setspeedoffset + math.floor((int((-ret.cruiseState.speed)*(minimum_set_speed-7.0)/(169.0-minimum_set_speed)  + 169*(minimum_set_speed-7.0)/(169.0-minimum_set_speed))-self.setspeedoffset)/(ret.cruiseState.speed-(minimum_set_speed-1.0)))
      self.setspeedcounter = 50
    if self.v_cruise_pcmlast < ret.cruiseState.speed:
      if self.setspeedcounter > 0 and (self.setspeedoffset - 4) > 0:
        self.setspeedoffset = self.setspeedoffset - 4
      else:
        self.setspeedoffset = self.setspeedoffset + math.floor((int((-ret.cruiseState.speed)*(minimum_set_speed-7.0)/(169.0-minimum_set_speed)  + 169*(minimum_set_speed-7.0)/(169.0-minimum_set_speed))-self.setspeedoffset)/(170-ret.cruiseState.speed))
      self.setspeedcounter = 50
    if self.setspeedcounter > 0:
      self.setspeedcounter = self.setspeedcounter - 1
    self.v_cruise_pcmlast = ret.cruiseState.speed
    if int(ret.cruiseState.speed) - self.setspeedoffset < 7:
      self.setspeedoffset = int(ret.cruiseState.speed) - 7
    if int(ret.cruiseState.speed) - self.setspeedoffset > 169:
      self.setspeedoffset = int(ret.cruiseState.speed) - 169


    ret.cruiseState.speed = min(max(7, int(ret.cruiseState.speed) - self.setspeedoffset),v_cruise_pcm_max)

    if not self.left_blinker_on and not self.right_blinker_on:
      self.Angles[self.Angle_counter] = abs(ret.steeringAngle)
      #self.Angles_later[self.Angle_counter] = abs(angle_later)
      if self.gasbuttonstatus ==1:
        factor = 1.6
      elif self.gasbuttonstatus == 2:
        factor = 1.0
      else:
        factor = 1.3
      ret.cruiseState.speed = int(min(ret.cruiseState.speed, factor * interp(np.max(self.Angles), self.Angle, self.Angle_Speed)))
      #ret.cruiseState.speed = int(min(ret.cruiseState.speed, self.brakefactor * interp(np.max(self.Angles_later), self.Angle, self.Angle_Speed)))
    else:
      self.Angles[self.Angle_counter] = 0
      #self.Angles_later[self.Angle_counter] = 0
    self.Angle_counter = (self.Angle_counter + 1 ) % 250

    self.pcm_acc_status = cp.vl["PCM_CRUISE"]['CRUISE_STATE']
    if self.CP.carFingerprint in NO_STOP_TIMER_CAR or self.CP.enableGasInterceptor:
      # ignore standstill in hybrid vehicles, since pcm allows to restart without
      # receiving any special command. Also if interceptor is detected
      ret.cruiseState.standstill = False
    else:
      ret.cruiseState.standstill = self.pcm_acc_status == 7
    self.pcm_acc_active = bool(cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE'])
    ret.cruiseState.enabled = self.pcm_acc_active

    if self.CP.carFingerprint == CAR.PRIUS:
      ret.genericToggle = cp.vl["AUTOPARK_STATUS"]['STATE'] != 0
    else:
      ret.genericToggle = bool(cp.vl["LIGHT_STALK"]['AUTO_HIGH_BEAM'])
    ret.stockAeb = bool(cp_cam.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_cam.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    ret.espDisabled = cp.vl["ESP_CONTROL"]['TC_DISABLED'] != 0
    # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = cp.vl["EPS_STATUS"]['LKA_STATE']
    self.steer_warning = cp.vl["EPS_STATUS"]['LKA_STATE'] not in [1, 5]
    
    self.barriers = cp_cam.vl["LKAS_HUD"]['BARRIERS']
    self.rightline = cp_cam.vl["LKAS_HUD"]['RIGHT_LINE']
    self.leftline = cp_cam.vl["LKAS_HUD"]['LEFT_LINE']

    self.tsgn1 = cp_cam.vl["RSA1"]['TSGN1']
    self.spdval1 = cp_cam.vl["RSA1"]['SPDVAL1']

    self.splsgn1 = cp_cam.vl["RSA1"]['SPLSGN1']
    self.tsgn2 = cp_cam.vl["RSA1"]['TSGN2']
    self.spdval2 = cp_cam.vl["RSA1"]['SPDVAL2']

    self.splsgn2 = cp_cam.vl["RSA1"]['SPLSGN2']
    self.tsgn3 = cp_cam.vl["RSA2"]['TSGN3']
    self.splsgn3 = cp_cam.vl["RSA2"]['SPLSGN3']
    self.tsgn4 = cp_cam.vl["RSA2"]['TSGN4']
    self.splsgn4 = cp_cam.vl["RSA2"]['SPLSGN4']
    self.noovertake = self.tsgn1 == 65 or self.tsgn2 == 65 or self.tsgn3 == 65 or self.tsgn4 == 65 or self.tsgn1 == 66 or self.tsgn2 == 66 or self.tsgn3 == 66 or self.tsgn4 == 66
    if self.spdval1 > 0 or self.spdval2 > 0:
      dat = messaging_arne.new_message()
      dat.init('liveTrafficData')
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
      if self.spdval2 > 0:
        dat.liveTrafficData.speedAdvisoryValid = True
        dat.liveTrafficData.speedAdvisory = self.spdval2
      else:
        dat.liveTrafficData.speedAdvisoryValid = False
      if not travis:
        self.arne_pm.send('liveTrafficData', dat)
        
    return ret
  
  @staticmethod
  def get_can_parser_init(CP):

    signals = [
      # sig_name, sig_address, default
      ("STEER_ANGLE", "STEER_ANGLE_SENSOR", 0),
      ("GEAR", "GEAR_PACKET", 0),
      ("SPORT_ON", "GEAR_PACKET", 0),
      ("ECON_ON", "GEAR_PACKET", 0),
      ("BRAKE_PRESSED", "BRAKE_MODULE", 0),
      ("GAS_PEDAL", "GAS_PEDAL", 0),
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
      ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR", 0),
      ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR", 0),
      ("STEER_ANGLE", "STEER_TORQUE_SENSOR", 0),
      ("TURN_SIGNALS", "STEERING_LEVERS", 3),   # 3 is no blinkers
      ("LKA_STATE", "EPS_STATUS", 0),
      ("BRAKE_LIGHTS_ACC", "ESP_CONTROL", 0),
      ("AUTO_HIGH_BEAM", "LIGHT_STALK", 0),
    ]

    checks = [
      ("BRAKE_MODULE", 40),
      ("GAS_PEDAL", 33),
      ("WHEEL_SPEEDS", 80),
      ("STEER_ANGLE_SENSOR", 80),
      ("PCM_CRUISE", 33),
      ("STEER_TORQUE_SENSOR", 50),
      ("EPS_STATUS", 25),
    ]

    if CP.carFingerprint == CAR.LEXUS_IS:
      signals.append(("MAIN_ON", "DSU_CRUISE", 0))
      signals.append(("SET_SPEED", "DSU_CRUISE", 0))
      checks.append(("DSU_CRUISE", 5))
    else:
      signals.append(("MAIN_ON", "PCM_CRUISE_2", 0))
      signals.append(("SET_SPEED", "PCM_CRUISE_2", 0))
      signals.append(("LOW_SPEED_LOCKOUT", "PCM_CRUISE_2", 0))
      checks.append(("PCM_CRUISE_2", 33))


    if CP.carFingerprint == CAR.PRIUS:
      signals += [("STATE", "AUTOPARK_STATUS", 0)]

    # add gas interceptor reading if we are using it
    if CP.enableGasInterceptor:
      signals.append(("INTERCEPTOR_GAS", "GAS_SENSOR", 0))
      signals.append(("INTERCEPTOR_GAS2", "GAS_SENSOR", 0))
      checks.append(("GAS_SENSOR", 50))

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("STEER_ANGLE", "STEER_ANGLE_SENSOR", 0),
      ("GEAR", "GEAR_PACKET", 0),
      ("BRAKE_PRESSED", "BRAKE_MODULE", 0),
      ("GAS_PEDAL", "GAS_PEDAL", 0),
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
      ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR", 0),
      ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR", 0),
      ("STEER_ANGLE", "STEER_TORQUE_SENSOR", 0),
      ("TURN_SIGNALS", "STEERING_LEVERS", 3),   # 3 is no blinkers
      ("LKA_STATE", "EPS_STATUS", 0),
      ("BRAKE_LIGHTS_ACC", "ESP_CONTROL", 0),
      ("AUTO_HIGH_BEAM", "LIGHT_STALK", 0),
      ("SPORT_ON", "GEAR_PACKET", 0),
      ("ECON_ON", "GEAR_PACKET", 0),
      ("BLINDSPOT","DEBUG", 0),
      ("BLINDSPOTSIDE","DEBUG",65),
      ("BLINDSPOTD1","DEBUG", 0),
      ("BLINDSPOTD2","DEBUG", 0),
      ("ACC_DISTANCE", "JOEL_ID", 2),
      ("LANE_WARNING", "JOEL_ID", 1),
      ("ACC_SLOW", "JOEL_ID", 0),
      ("DISTANCE_LINES", "PCM_CRUISE_SM", 0),
    ]

    checks = [
      ("BRAKE_MODULE", 40),
      ("GAS_PEDAL", 33),
      ("WHEEL_SPEEDS", 80),
      ("STEER_ANGLE_SENSOR", 80),
      ("PCM_CRUISE", 33),
      ("STEER_TORQUE_SENSOR", 50),
      ("EPS_STATUS", 25),
    ]

    if CP.carFingerprint == CAR.LEXUS_IS:
      signals.append(("MAIN_ON", "DSU_CRUISE", 0))
      signals.append(("SET_SPEED", "DSU_CRUISE", 0))
      checks.append(("DSU_CRUISE", 5))
    else:
      signals.append(("MAIN_ON", "PCM_CRUISE_2", 0))
      signals.append(("SET_SPEED", "PCM_CRUISE_2", 0))
      signals.append(("LOW_SPEED_LOCKOUT", "PCM_CRUISE_2", 0))
      checks.append(("PCM_CRUISE_2", 33))


    if CP.carFingerprint == CAR.PRIUS:
      signals += [("STATE", "AUTOPARK_STATUS", 0)]

    # add gas interceptor reading if we are using it
    if CP.enableGasInterceptor:
      signals.append(("INTERCEPTOR_GAS", "GAS_SENSOR", 0))
      signals.append(("INTERCEPTOR_GAS2", "GAS_SENSOR", 0))
      checks.append(("GAS_SENSOR", 50))

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):

    signals = [("FORCE", "PRE_COLLISION", 0),
               ("PRECOLLISION_ACTIVE", "PRE_COLLISION", 0),
               ("TSGN1", "RSA1", 0),
               ("SPDVAL1", "RSA1", 0),
               ("SPLSGN1", "RSA1", 0),
               ("TSGN2", "RSA1", 0),
               ("SPDVAL2", "RSA1", 0),
               ("SPLSGN2", "RSA1", 0),
               ("TSGN3", "RSA2", 0),
               ("SPLSGN3", "RSA2", 0),
               ("TSGN4", "RSA2", 0),
               ("SPLSGN4", "RSA2", 0),
               ("BARRIERS", "LKAS_HUD", 0),
               ("RIGHT_LINE", "LKAS_HUD", 0),
               ("LEFT_LINE", "LKAS_HUD", 0),]

    # use steering message to check if panda is connected to frc
    checks = [("STEERING_LKA", 42)]

       
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)
