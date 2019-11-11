import selfdrive.messaging_arne as messaging_arne


class PhantomReceiver:
  def __init__(self):
    self.phantom_data_sock = None

  def receive_data(self, speed, angle, time):
    self.broadcast_data(True, speed, angle, time)

  def broadcast_data(self, status, speed, angle, time):
    with open('/data/bd.test', 'a') as f:
      f.write('{}\n'.format([status, speed, angle, time]))
    phantom_data = messaging_arne.new_message()
    phantom_data.init('phantomData')
    phantom_data.phantomData.status = status
    phantom_data.phantomData.speed = speed
    phantom_data.phantomData.angle = angle
    phantom_data.phantomData.time = time
    self.phantom_data_sock.send(phantom_data.to_bytes())

  def enable_phantom(self):
    self.phantom_data_sock = messaging_arne.pub_sock('phantomData')
    return "ENABLED"  # needed to tell the app we're all done with this function

  def disable_phantom(self):
    self.broadcast_data(False, 0.0, 0.0, 0.0)
    self.phantom_data_sock.close()
    return "DISABLED"
