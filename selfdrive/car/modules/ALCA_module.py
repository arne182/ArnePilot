"""
Copyright 2018-2019 BB Solutions, LLC. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
    
  * Neither the name of Google nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.
    
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

HISTORY
-------
v4.1 - OP 0.6.5 operating model
v4.0 - integrated into model_parser.py
v3.6 - moved parameters to carstate.py
v3.5 - changing the start angle to keep turning until we reach MAX_ANGLE_DELTA
v3.4 - read steerRatio from each car parameters file
v3.3 - re-entry logic changed for smoothness
v3.2 - angle adjustment to compesate for road curvature change
v3.1 - new angle logic for a smoother re-entry
v3.0 - better lane dettection logic
v2.0 - detection of lane crossing 
v1.0 - fixed angle move
"""

from common.numpy_fast import interp,clip
from selfdrive.controls.lib.pid import PIController
from common.realtime import sec_since_boot
import selfdrive.messaging as messaging
import numpy as np
from cereal import arne182

#wait time after turn complete before enabling smoother
WAIT_TIME_AFTER_TURN = 2.0

#ALCA
ALCA_line_min_prob = 0.01
ALCA_release_distance = 0.3
ALCA_line_prob_low = 0.2
ALCA_line_prob_high = 0.4
ALCA_distance_jump = 1.1
ALCA_lane_change_coefficient = 0.7
ITERATIONS_AHEAD_TO_ESTIMATE = 2
ALCA_duration_seconds = 5.
ALCA_right_lane_multiplier = 1.
ALCA_distance_left_min = 0.7

ALCA_DEBUG = False
DEBUG_INFO = "step {step} of {total_steps}: direction = {ALCA_direction} | using visual = {ALCA_use_visual} | over line = {ALCA_over_line} | lane width = {ALCA_lane_width} | left to move = {left_to_move} | from center = {from_center} | C2 offset = {ALCA_OFFSET_C2} | C1 offset = {ALCA_OFFSET_C1} | Prob Low = {prob_low} | Prob High = {prob_high}"


class ALCAController():
  def __init__(self,carcontroller,alcaEnabled,steerByAngle):
    #import settings
    self.frame = 0
    self.CC = carcontroller  # added to start, will see if we need it actually
    # variables for lane change
    self.angle_offset = 0. #added when one needs to compensate for missalignment
    self.alcaEnabled = alcaEnabled
    self.laneChange_over_the_line = 0 # did we cross the line?
    self.laneChange_enabled = 1 # set to zero for no lane change
    self.laneChange_counter = 0 # used to count frames during lane change
    self.laneChange_wait = 2 # how many seconds to wait before it starts the change
    self.laneChange_direction = 0 # direction of the lane change 
    self.laneChange_cancelled = False
    self.laneChange_cancelled_counter = 0
    self.alcaStatusSocket = messaging.pub_sock('alcaStatus')

  def debug_alca(self,message):
    if ALCA_DEBUG:
      print (message)

  def send_status(self,CS):
    CS.ALCA_enabled = (self.laneChange_enabled > 1) and self.alcaEnabled
    CS.ALCA_total_steps = int(20 * ALCA_duration_seconds)
    if self.laneChange_enabled == 3:
      CS.ALCA_direction = -self.laneChange_direction
    else:
      CS.ALCA_direction = 0
    if not (self.frame % 20 == 0):
        return
    alca_status = arne182.ALCAStatus.new_message()
    alca_status.alcaEnabled = bool(CS.ALCA_enabled)
    alca_status.alcaTotalSteps = int(CS.ALCA_total_steps)
    alca_status.alcaDirection = int(CS.ALCA_direction)
    alca_status.alcaError = bool(CS.ALCA_error)
    self.alcaStatusSocket.send(alca_status.to_bytes())


  def update_status(self,alcaEnabled):
    self.alcaEnabled = alcaEnabled


  def stop_ALCA(self, CS, isDone):
    # something is not right; ALCAModelParser is not engaged; cancel
    self.debug_alca("ALCA canceled: stop_ALCA called")
    if not isDone:
      #CS.UE.custom_alert_message(3,"Auto Lane Change Canceled! (d)",200,5)
      self.laneChange_cancelled = True
      self.laneChange_cancelled_counter = 200
    else:
      self.laneChange_cancelled = False
      self.laneChange_cancelled_counter = 0
    self.laneChange_enabled = 1
    self.laneChange_counter = 0
    self.laneChange_direction = 0
    #CS.cstm_btns.set_button_status("alca",1)
    self.send_status(CS)


  def update(self,enabled,CS,actuators,alcaStateData,frame):
    cl_min_v = CS.CL_MIN_V
    cl_max_a = CS.CL_MAX_A
    self.frame = frame

    if (alcaStateData is not None) and (((self.laneChange_direction != 0) and alcaStateData.alcaError) or (alcaStateData.alcaDirection == 100)):
      self.debug_alca("ALCA canceled: stop_ALCA called (1)")
      self.stop_ALCA(CS,alcaStateData.alcaDirection == 100)
      return 0, False

    if self.laneChange_cancelled_counter > 0:
      self.laneChange_cancelled_counter -= 1
      if self.laneChange_cancelled_counter == 0:
        self.laneChange_cancelled = False

    # Basic highway lane change logic
    turn_signal_needed = 0 # send 1 for left, 2 for right 0 for not needed

    if CS.turn_signal_stalk_state == 0 and \
    (self.laneChange_enabled == 4):
        self.debug_alca("ALCA reset: resetting 4 -> 1")
        self.laneChange_enabled =1
        self.laneChange_counter =0
        self.laneChange_direction =0
        #CS.UE.custom_alert_message(-1,"",0)

    if CS.turn_signal_stalk_state == 0 and \
      (self.laneChange_enabled > 1):
      # no blinkers on but we are still changing lane, so we need to send blinker command
      if self.laneChange_direction == -1:
        turn_signal_needed = 1
      elif self.laneChange_direction == 1:
        turn_signal_needed = 2

    #if self.alcaEnabled and (self.laneChange_enabled == 1): #(CS.cstm_btns.get_button_status("alca") > 0) and
      #if ((CS.v_ego < cl_min_v) or (abs(actuators.steerAngle) >= cl_max_a) or \
      #(abs(CS.angle_steers)>= cl_max_a)  or (not enabled)): 
        #CS.cstm_btns.set_button_status("alca",9)
      #else:
        #CS.cstm_btns.set_button_status("alca",1)

    if self.alcaEnabled and enabled and (self.laneChange_enabled > 1) and \
      ((CS.v_ego < cl_min_v) or (abs(actuators.steerAngle) >= cl_max_a) or (abs(CS.angle_steers) >=cl_max_a)):
      # something is not right, the speed or angle is limitting
      self.debug_alca("ALCA Unavailable (2)")
      #CS.UE.custom_alert_message(3,"Auto Lane Change Unavailable!",200,3)
      #CS.cstm_btns.set_button_status("alca",9)
      self.stop_ALCA(CS, False)
      return 0, False

    if self.alcaEnabled and enabled and CS.prev_turn_signal_stalk_state == 0 and CS.turn_signal_stalk_state > 0 and \
      (CS.v_ego >= cl_min_v) and (abs(actuators.steerAngle) < cl_max_a) and (self.laneChange_enabled == 1):
      # start blinker, speed and angle is within limits, let's go
      laneChange_direction = -1 if CS.turn_signal_stalk_state == 1 else 1 # left -1, right 1
      self.debug_alca("ALCA blinker on detected")

      #CS.UE.custom_alert_message(2,"Auto Lane Change Engaged!",100)
      self.debug_alca("ALCA engaged")
      self.laneChange_enabled = 2
      self.laneChange_counter = 1
      self.laneChange_direction = laneChange_direction
      #CS.cstm_btns.set_button_status("alca",2)

    if (not self.alcaEnabled) and self.laneChange_enabled > 1:
      self.debug_alca("ALCA canceled: not enabled")
      self.laneChange_enabled = 1
      self.laneChange_counter = 0
      self.laneChange_direction = 0
      self.stop_ALCA(CS, False)
      return 0, False

    # lane change in progress
    if self.laneChange_enabled > 1:
      if (CS.steer_override or (CS.v_ego < cl_min_v)):
        #CS.UE.custom_alert_message(4,"Auto Lane Change Canceled! (u)",200,3)
        self.debug_alca("ALCA canceled: steer override")
        self.laneChange_cancelled = True
        self.laneChange_cancelled_counter = 200
        # if any steer override cancel process or if speed less than min speed
        self.laneChange_counter = 0
        self.laneChange_enabled = 1
        self.laneChange_direction = 0
        #CS.cstm_btns.set_button_status("alca",1)
        self.stop_ALCA(CS, False)
        return 0, False
      if self.laneChange_enabled == 2:
        #if self.laneChange_counter == 1:
          #CS.UE.custom_alert_message(2,"Auto Lane Change Engaged! (1)",self.laneChange_wait * 100)
        self.laneChange_counter += 1
        self.debug_alca("ALCA phase 2: " + str(self.laneChange_counter))
        if self.laneChange_counter == self.laneChange_wait * 100:
          self.laneChange_enabled = 3
          self.laneChange_counter = 0
      if self.laneChange_enabled ==3:
        #if self.laneChange_counter == 1:
          #CS.UE.custom_alert_message(2,"Auto Lane Change Engaged! (2)",int(ALCA_duration_seconds * 100))
        self.laneChange_counter += 1
        self.debug_alca("ALCA phase 3: " + str(self.laneChange_counter))
        if self.laneChange_counter >= (ALCA_duration_seconds + 2) * 100.:
          self.debug_alca("ALCA phase 3: Canceled due to time restriction")
          self.laneChange_enabled = 4
          self.laneChange_counter = 0
      if self.laneChange_enabled == 4:
        self.debug_alca("ALCA phase 4: " +str(self.laneChange_counter))
        if self.laneChange_counter == 1:
          #CS.UE.custom_alert_message(2,"Auto Lane Change Complete!",100)
          self.laneChange_enabled = 1
          self.laneChange_counter = 0
          self.stop_ALCA(CS, True)
          return 0, False

    self.send_status(CS)
    return turn_signal_needed, self.laneChange_enabled > 1

class ALCAModelParser():
  def __init__(self):
    #ALCA params
    self.ALCA_error = False
    self.ALCA_lane_width = 3.0
    self.ALCA_direction = 100  #none 0, left 1, right -1,reset 100
    self.ALCA_step = 0
    self.ALCA_total_steps = 20 * ALCA_duration_seconds #20 Hz, 5 seconds, wifey mode
    self.ALCA_cancelling = False
    self.ALCA_enabled = False
    self.ALCA_OFFSET_C3 = 0.
    self.ALCA_OFFSET_C2 = 0.
    self.ALCA_OFFSET_C1 = 0.
    self.ALCA_over_line = False
    self.prev_CS_ALCA_error = False
    self.ALCA_use_visual = True
    self.ALCA_vego = 0.
    self.ALCA_vego_prev = 0.
    self.alcaStatus = messaging.sub_sock('alcaStatus', conflate=True)
    self.alcaState = messaging.pub_sock('alcaState')
    self.alcas = None
    self.hit_prob_low = False
    self.hit_prob_high = False
    self.distance_to_line_L = 100.
    self.prev_distance_to_line_L = 100.
    self.distance_to_line_R = 100.
    self.prev_distance_to_line_R = 100.


  def reset_alca (self,v_ego):
    self.ALCA_step = 0
    self.ALCA_direction = 100
    self.ALCA_cancelling = False
    self.ALCA_error = False
    self.ALCA_enabled = False
    self.ALCA_OFFSET_C3 = 0.
    self.ALCA_OFFSET_C2 = 0.
    self.ALCA_OFFSET_C1 = 0.
    self.ALCA_over_line = False
    self.ALCA_use_visual = True
    self.ALCA_vego_prev = v_ego
    self.alcas = None
    self.hit_prob_low = False
    self.hit_prob_high = False
    self.distance_to_line_L = 100.
    self.prev_distance_to_line_L = 100.
    self.distance_to_line_R = 100.
    self.prev_distance_to_line_R = 100.
    self.send_state()

  def debug_alca(self,message):
    if ALCA_DEBUG:
      print (message)

  def send_state(self):
    alca_state = arne182.ALCAState.new_message()
    #ALCA params
    alca_state.alcaDirection = int(self.ALCA_direction)
    alca_state.alcaError = bool(self.ALCA_error)
    alca_state.alcaCancelling = bool(self.ALCA_cancelling)
    alca_state.alcaEnabled = bool(self.ALCA_enabled)
    alca_state.alcaLaneWidth = float(self.ALCA_lane_width)
    alca_state.alcaStep = int(self.ALCA_step)
    alca_state.alcaTotalSteps = int(self.ALCA_total_steps)
    self.alcaState.send(alca_state.to_bytes())

  def update(self, v_ego, md, r_poly, l_poly, r_prob, l_prob, lane_width, p_poly):

    alcaStatusMsg = self.alcaStatus.receive(non_blocking=True)
    if alcaStatusMsg is not None:
      self.alcas = arne182.ALCAStatus.from_bytes(alcaStatusMsg)

    #if we don't have yet ALCA status, return same values
    if self.alcas is None:
      self.send_state()
      return np.array(r_poly),np.array(l_poly),r_prob, l_prob, lane_width, p_poly

    if self.alcas.alcaDirection == 0:
      self.ALCA_direction = 0

    if self.ALCA_direction < 100:
      self.ALCA_direction = self.alcas.alcaDirection

      self.ALCA_enabled = self.alcas.alcaEnabled

      self.ALCA_total_steps = self.alcas.alcaTotalSteps
      self.ALCA_error = self.ALCA_error or (self.alcas.alcaError and not self.prev_CS_ALCA_error)
      self.prev_CS_ALCA_error = self.alcas.alcaError

    if self.ALCA_enabled:
      if self.ALCA_direction == 0:
        self.ALCA_lane_width = lane_width
      else:
        lane_width = self.ALCA_lane_width

    #if error but no direction, the carcontroller component is fine and we need to reset
    if self.ALCA_error and (self.ALCA_direction == 0):
      self.ALCA_error = False

    if (not self.ALCA_enabled) or (self.ALCA_direction == 100) or (self.ALCA_direction == 0):
      self.ALCA_vego_prev = v_ego
      self.send_state()
      return np.array(r_poly),np.array(l_poly),r_prob, l_prob, lane_width,np.array(p_poly)

    self.hit_prob_low = self.hit_prob_low and ((self.ALCA_direction != 0) and (self.hit_prob_low or ((self.ALCA_direction == 1) and (l_prob < ALCA_line_prob_low)) or ((self.ALCA_direction == -1) and (r_prob < ALCA_line_prob_low))))
    self.hit_prob_high = (self.ALCA_direction != 0) and (self.hit_prob_low) and (self.hit_prob_high or ((self.ALCA_direction == 1) and (r_prob > ALCA_line_prob_high)) or ((self.ALCA_direction == -1) and (l_prob < ALCA_line_prob_high)))

    if self.hit_prob_high:
      self.debug_alca("Hit high probability for line, ALCA done, releasing...")
      self.reset_alca(v_ego)
      return np.array(r_poly),np.array(l_poly),r_prob, l_prob, lane_width,np.array(p_poly)

    #where are we in alca as %
    ALCA_perc_complete = float(self.ALCA_step) / float(self.ALCA_total_steps)
    if self.ALCA_error and self.ALCA_cancelling:
      self.debug_alca(" Error and Cancelling -> resetting...")
      self.reset_alca(v_ego)
      return np.array(r_poly),np.array(l_poly),r_prob, l_prob, lane_width,np.array(p_poly)
    if self.ALCA_error and not self.ALCA_cancelling:
      if (ALCA_perc_complete < 0.1) or (ALCA_perc_complete > 0.9):
        self.debug_alca(" Error and less than 10% -> resetting...")
        self.reset_alca(v_ego)
        return np.array(r_poly),np.array(l_poly),r_prob, l_prob, lane_width,np.array(p_poly)
      else:
        self.debug_alca(" Error and not Cancelling -> rewinding...")
        self.ALCA_cancelling = True
        self.ALCA_error = False

    self.ALCA_step += 1 #ALCA_increment


    if (self.ALCA_step < 0) or (self.ALCA_step >= self.ALCA_total_steps):
      #done so end ALCA
      self.debug_alca(" step out of bounds -> resetting...")
      self.reset_alca(v_ego)
      return np.array(r_poly),np.array(l_poly),r_prob, l_prob, lane_width,np.array(p_poly)

    #compute offset
    from_center = 0.
    left_to_move = 0.

    #compute distances to lines
    self.distance_to_line_L = abs(l_poly[3]) 
    self.distance_to_line_R = abs(r_poly[3]) 
    percent_completed = float(self.ALCA_total_steps - self.ALCA_step) / float(self.ALCA_total_steps)
    percent_completed = clip(percent_completed, ALCA_distance_left_min, 1.0)
    distance_left = float((self.ALCA_total_steps) * 0.05 * percent_completed * (self.ALCA_vego_prev + v_ego) / 2.) #5m + distance left
    distance_estimate = float(ITERATIONS_AHEAD_TO_ESTIMATE * 0.05 * (self.ALCA_vego_prev + v_ego) / 2.) 
    estimate_curv_at = distance_estimate / distance_left
    left_to_move = self.ALCA_lane_width * estimate_curv_at
    # if ((self.ALCA_direction == 1) and ((self.distance_to_line_L > ALCA_distance_jump * self.prev_distance_to_line_L) or (self.distance_to_line_R < self.ALCA_lane_width / 3.))) or \
    #   ((self.ALCA_direction == -1) and ((self.distance_to_line_R > ALCA_distance_jump * self.prev_distance_to_line_R) or (self.distance_to_line_L < self.ALCA_lane_width / 3.))):
    #   self.ALCA_over_line = True
    if ((self.ALCA_direction == 1) and (self.distance_to_line_R < self.ALCA_lane_width / 3.)) or \
      ((self.ALCA_direction == -1) and (self.distance_to_line_L < self.ALCA_lane_width / 3.)):
      self.ALCA_over_line = True
    left_to_move = self.ALCA_lane_width * estimate_curv_at 
    if self.ALCA_over_line:
      left_to_move2 = (self.distance_to_line_L if self.ALCA_direction == 1 else self.distance_to_line_R) - self.ALCA_lane_width / 2. 
      if left_to_move2 < ALCA_release_distance:
        self.reset_alca(v_ego)
        return np.array(r_poly),np.array(l_poly),r_prob, l_prob, lane_width, p_poly

    d1 = np.polyval(p_poly,distance_estimate -1)
    d2 = np.polyval(p_poly,distance_estimate + 1)
    cos = 0.
    turn_mult = 0.
    if abs(d2 - d1) > 0.001:
      cos = abs(np.cos(np.arctan(2/abs(d2-d1))))
      # turn mult is used to detect if we move in the same direction as the turn or oposite direction
      # should be + if we move in the same direction and - if opposite
      # it will increase or decrease the turn angle by cos(angle)
      turn_mult = max (0, self.ALCA_direction * (d2-d1)/abs(d2-d1))
    d0 = (d2 + d1) / 2.0 - np.polyval(p_poly,distance_estimate)
    ltm = left_to_move  * (1 +  cos * turn_mult)
    #compute offsets
    self.ALCA_OFFSET_C1 = 0.
    lane_multiplier = 1. if self.ALCA_direction == 1 else ALCA_right_lane_multiplier
    self.ALCA_OFFSET_C2 = lane_multiplier * ALCA_lane_change_coefficient * float(self.ALCA_direction * ltm) / (distance_estimate )
    self.prev_distance_to_line_R = self.distance_to_line_R
    self.prev_distance_to_line_L = self.distance_to_line_L
    if ALCA_DEBUG:
      debug_string = DEBUG_INFO.format(step=self.ALCA_step,total_steps=self.ALCA_total_steps,ALCA_direction=self.ALCA_direction,ALCA_use_visual=self.ALCA_use_visual,ALCA_over_line=self.ALCA_over_line,ALCA_lane_width=self.ALCA_lane_width, left_to_move=left_to_move/estimate_curv_at, from_center=from_center, ALCA_OFFSET_C2=self.ALCA_OFFSET_C2, ALCA_OFFSET_C1=self.ALCA_OFFSET_C1,prob_low=self.hit_prob_low,prob_high=self.hit_prob_high)
      self.debug_alca(debug_string)

    if (not self.ALCA_error) and self.ALCA_use_visual:
      if self.ALCA_over_line:
        if (self.ALCA_total_steps - self.ALCA_step <= 1) or (self.ALCA_over_line and ((self.ALCA_direction == 1) and ((r_poly[3] < -ALCA_release_distance) or (l_poly[3] < self.ALCA_lane_width / 2. - ALCA_release_distance))) or ((self.ALCA_direction == -1) and ((l_poly[3] > ALCA_release_distance) or (r_poly[3] > -(self.ALCA_lane_width / 2. - ALCA_release_distance))))):
          self.ALCA_error = False
          self.reset_alca(v_ego)
          return np.array(r_poly),np.array(l_poly),r_prob, l_prob, lane_width, p_poly

    if l_prob > r_prob:
      r_poly = np.array(l_poly)
      if l_prob > ALCA_line_prob_low:
        l_prob = 1
      r_prob = l_prob
    else:
      l_poly = np.array(r_poly)
      if r_prob > ALCA_line_prob_low:
        r_prob = 1
      l_prob = r_prob     
    l_poly[3] = self.ALCA_lane_width / 2
    r_poly[3] = -self.ALCA_lane_width / 2
    l_poly[2] += self.ALCA_OFFSET_C2
    r_poly[2] += self.ALCA_OFFSET_C2
    l_poly[1] += self.ALCA_OFFSET_C1
    r_poly[1] += self.ALCA_OFFSET_C1
    p_poly[3] = 0
    p_poly[2] += self.ALCA_OFFSET_C2
    p_poly[1] += self.ALCA_OFFSET_C1

    self.ALCA_vego_prev = v_ego	
    self.send_state()	
    return np.array(r_poly),np.array(l_poly),r_prob, l_prob, self.ALCA_lane_width, np.array(p_poly)
