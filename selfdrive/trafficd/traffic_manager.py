#!/usr/bin/env python3

from common.numpy_fast import clip
from common.basedir import BASEDIR
from common.realtime import sec_since_boot
import cereal.messaging as messaging
import numpy as np
import time
import subprocess
import os
import signal


class TrafficdThread:
  def __init__(self):
    self.proc_path = 'selfdrive/trafficd/trafficd'
    self.proc = None
    self.started = False

  def start(self):
    if not self.started:
      self.proc = subprocess.Popen(os.path.join(BASEDIR, self.proc_path), cwd=os.path.join(BASEDIR, os.path.dirname(self.proc_path)))
      self.started = True
    else:
      print('trafficd already started')

  def stop(self):
    if self.proc is not None and self.started:
      self.proc.send_signal(signal.SIGINT)
      self.started = False
    else:
      print('trafficd not started, can\'t stop')



class Traffic:
  def __init__(self):
    self.pm = messaging.PubMaster(['trafficModelEvent'])
    self.sm = messaging.SubMaster(['trafficModelRaw'])

    self.labels = ['SLOW', 'GREEN', 'NONE']
    self.model_rate = 1 / 5.
    self.recurrent_length = 1.  # in seconds, how far back to factor into current prediction
    self.min_preds = int(round(self.recurrent_length / self.model_rate))
    self.last_pred_weight = 5.  # places nx weight on most recent prediction
    self.trafficd_timeout = 5.  # in seconds, how long to wait before realizing trafficd is dead

    self.past_preds = []
    self.weights = np.linspace(1, self.last_pred_weight, self.min_preds)
    self.weight_sum = sum(self.weights)
    self.last_log = {'log': 0, 'time': sec_since_boot()}
    self.shown_dead_warning = False

  def start(self):
    self.traffic_loop()

  def traffic_loop(self):
    while True:
      while not self.is_new_msg():  # uses rate keeper from traffic.cc, waits for new message
        time.sleep(self.model_rate)
        self.sm.update(0)
        if self.is_dead:
          break

      if not self.is_dead:
        self.shown_dead_warning = False
        self.past_preds.append(list(self.sm['trafficModelRaw'].prediction))
        pred, confidence = self.get_prediction()  # uses most common prediction from weighted past second list (1 / model_rate), NONE until car is started for min time
        #print('{}, confidence: {}'.format(pred, confidence))
        self.send_prediction(pred, confidence)
      else:
        if not self.shown_dead_warning and self.last_log['log'] != 0:
          self.send_prediction('DEAD', 1.0)  # only show once
          self.shown_dead_warning = True
        print("No response from trafficd in {} seconds. Is it dead?".format(round(sec_since_boot() - self.last_log['time'], 2)))
        time.sleep(0.5)

  def is_new_msg(self):
    log_time = self.sm.logMonoTime['trafficModelRaw']
    is_new = log_time != self.last_log['log']
    if is_new:
      self.last_log['log'] = log_time
      self.last_log['time'] = sec_since_boot()
    return is_new

  def get_prediction(self):
    while len(self.past_preds) > self.min_preds:
      del self.past_preds[0]
    if len(self.past_preds) != self.min_preds:  # not enough predictions yet for recurrent algorithm
      return 'NONE', 1

    # below is a weighted average, the further back in time we go, the less we care (and vice versa)
    time_weighted_preds = [[label * self.weights[idx] for label in pred] for idx, pred in enumerate(self.past_preds)]
    time_weighted_preds = [sum(label) / self.weight_sum for label in np.array(time_weighted_preds).T]  # pylint: disable=E1133

    prediction = np.argmax(time_weighted_preds)  # get most confident prediction
    confidence = clip(time_weighted_preds[prediction], 0, 1)
    return self.labels[prediction], confidence

  def send_prediction(self, pred, confidence):
    traffic_send = messaging.new_message('trafficModelEvent')
    traffic_send.trafficModelEvent.status = pred
    traffic_send.trafficModelEvent.confidence = float(confidence)
    self.pm.send('trafficModelEvent', traffic_send)

  @property
  def is_dead(self):
    return sec_since_boot() - self.last_log['time'] > self.trafficd_timeout

  def rate_keeper(self, loop_time):
    time.sleep(max(self.model_rate - loop_time, 0))


def main():
  #time.sleep(5)
  traffic = Traffic()
  traffic.start()

if __name__ == "__main__":
  main()
