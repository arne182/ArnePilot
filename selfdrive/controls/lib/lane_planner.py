from common.numpy_fast import interp
import numpy as np
from selfdrive.hardware import EON, TICI
from cereal import log
from common.dp_common import get_last_modified, param_get_if_updated
from common.dp_time import LAST_MODIFIED_LANE_PLANNER
from common.op_params import opParams
from common.travis_checker import travis

use_virtual_middle_line = opParams().get('use_virtual_middle_line')


TRAJECTORY_SIZE = 33
# camera offset is meters from center car to camera
if EON:
  CAMERA_OFFSET = 0.06
elif TICI:
  CAMERA_OFFSET = -0.04
else:
  CAMERA_OFFSET = 0.0



class LanePlanner:
  def __init__(self):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.lane_width_estimate = 2.85
    self.lane_width_certainty = 1.0
    self.lane_width = 2.85

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.


    # dp
    self.dp_camera_offset = CAMERA_OFFSET * 100
    self.last_modified_dp_camera_offset = None
    self.modified = None
    self.last_modified = None
    self.last_modified_check = None

  def parse_model(self, md):
    if len(md.laneLines) == 4 and len(md.laneLines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(md.laneLines[1].t) + np.array(md.laneLines[2].t))/2
      # left and right ll x is the same
      self.ll_x = md.laneLines[1].x
      # only offset left and right lane lines; offsetting path does not make sense
      self.lll_y = np.array(md.laneLines[1].y) - CAMERA_OFFSET
      self.rll_y = np.array(md.laneLines[2].y) - CAMERA_OFFSET
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    if len(md.meta.desireState):

      self.l_lane_change_prob = md.meta.desireState[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = md.meta.desireState[log.LateralPlan.Desire.laneChangeRight]

  def update_d_poly(self, v_ego):
    # only offset left and right lane lines; offsetting p_poly does not make sense
    if travis:
      offset = 0
    else:
      self.last_modified_check, self.modified = get_last_modified(LAST_MODIFIED_LANE_PLANNER, self.last_modified_check, self.modified)
      if self.last_modified != self.modified:
        self.dp_camera_offset, self.last_modified_dp_camera_offset = param_get_if_updated("dp_camera_offset", "int", self.dp_camera_offset, self.last_modified_dp_camera_offset)
        self.last_modified = self.modified
      offset = self.dp_camera_offset * 0.01 if self.dp_camera_offset != 0 else 0
    self.l_poly[3] += offset
    self.r_poly[3] += offset
    self.p_poly[3] += offset

  def get_d_path(self, v_ego, path_t, path_xyz):
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in [0.0, 1.5, 3.0]:
      width_at_t = interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    # Find current lanewidth
    self.lane_width_certainty += 0.05 * (l_prob * r_prob - self.lane_width_certainty)
    current_lane_width = abs(self.rll_y[3] - self.lll_y[3])
    #if use_virtual_middle_line and v_ego < 14.15:
    #  #lane_width = self.lane_width
    #  print(current_lane_width)
    #  if current_lane_width < 2.0:
    #    self.r_poly[3] -= 2.0 - current_lane_width # TODO: this should be l_poly if isRHD
    #    current_lane_width = 2.0
    #  elif current_lane_width > 4.0:
    #    factor = min(current_lane_width - 4.0, 1.0)
    #    self.l_poly[3] -= current_lane_width/2 * factor # TODO: this should be r_poly if isRHD
    #    current_lane_width -= current_lane_width/2 * factor
    self.lane_width_estimate += 0.005 * (current_lane_width - self.lane_width_estimate)
    speed_lane_width = interp(v_ego, [0., 14., 20.], [2.5, 3., 3.5]) # German Standards
    self.lane_width = self.lane_width_certainty * self.lane_width_estimate + \
                      (1 - self.lane_width_certainty) * speed_lane_width
    #path_from_left_lane = self.l_poly.copy()
    #path_from_right_lane = self.r_poly.copy()
    #if use_virtual_middle_line and v_ego < 14.15:
      #if current_lane_width > 4.0:
        #print(current_lane_width)
        #factor = min(current_lane_width - 4.0, 1.0)
        #clipped_lane_width = min(4.0, self.lane_width)
        #path_from_left_lane[3] -= current_lane_width/2 * factor
        #l_prob = max(factor, l_prob)
        #r_prob = max(factor, r_prob)
      #else:
        #clipped_lane_width = min(4.0, self.lane_width)
        #path_from_left_lane[3] -= clipped_lane_width / 2.0
        #path_from_right_lane[3] += clipped_lane_width / 2.0
    #else:
      #lipped_lane_width = min(4.0, self.lane_width)
      #path_from_left_lane[3] -= clipped_lane_width / 2.0
      #path_from_right_lane[3] += clipped_lane_width / 2.0

    #lr_prob = l_prob + r_prob - l_prob * r_prob

    #d_poly_lane = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
    #elf.d_poly = lr_prob * d_poly_lane + (1.0 - lr_prob) * self.p_poly.copy()

    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    self.d_prob = l_prob + r_prob - l_prob * r_prob
    lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
    lane_path_y_interp = np.interp(path_t, self.ll_t, lane_path_y)
    path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]
    return path_xyz
