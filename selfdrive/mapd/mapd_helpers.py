import math
import json
import numpy as np
from datetime import datetime
from common.basedir import BASEDIR
from selfdrive.config import Conversions as CV
from common.transformations.coordinates import LocalCoord, geodetic2ecef
Traffic_Debug = False # if traffic signals do not have a direction stop for them anyway
LOOKAHEAD_TIME = 10.
MAPS_LOOKAHEAD_DISTANCE = 50 * LOOKAHEAD_TIME

DEFAULT_SPEEDS_JSON_FILE = BASEDIR + "/selfdrive/mapd/default_speeds.json"
DEFAULT_SPEEDS = {}
with open(DEFAULT_SPEEDS_JSON_FILE, "rb") as f:
  DEFAULT_SPEEDS = json.loads(f.read())

DEFAULT_SPEEDS_BY_REGION_JSON_FILE = BASEDIR + "/selfdrive/mapd/default_speeds_by_region.json"
DEFAULT_SPEEDS_BY_REGION = {}
with open(DEFAULT_SPEEDS_BY_REGION_JSON_FILE, "rb") as f:
  DEFAULT_SPEEDS_BY_REGION = json.loads(f.read())

def rate_curvature_points(p2,p3,curvature2,curvature3):
  x2, y2, _ = p2
  x3, y3, _ = p3
  if abs(curvature3) > abs(curvature2):
    return abs((curvature3-curvature2)/(np.sqrt((x3-x2)**2+(y3-y2)**2)))
  else:
    return 0

def circle_through_points(p1, p2, p3, force=False, direction=False):
  """Fits a circle through three points
  Formulas from: http://www.ambrsoft.com/trigocalc/circle3d.htm"""
  x1, y1, _ = p1
  x2, y2, _ = p2
  x3, y3, _ = p3

  A = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2
  B = (x1**2 + y1**2) * (y3 - y2) + (x2**2 + y2**2) * (y1 - y3) + (x3**2 + y3**2) * (y2 - y1)
  C = (x1**2 + y1**2) * (x2 - x3) + (x2**2 + y2**2) * (x3 - x1) + (x3**2 + y3**2) * (x1 - x2)
  D = (x1**2 + y1**2) * (x3 * y2 - x2 * y3) + (x2**2 + y2**2) * (x1 * y3 - x3 * y1) + (x3**2 + y3**2) * (x2 * y1 - x1 * y2)
  try:
    if abs((y3-y1)*x2-(x3-x1)*y2+x3*y1-y3*x1)/np.sqrt((y3-y1)**2+(x3-x1)**2) > 0.1 or force:
      if direction:
        if (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1)>0:
          return (-B / (2 * A), - C / (2 * A), np.sqrt((B**2 + C**2 - 4 * A * D) / (4 * A**2)))
        else:
          return (-B / (2 * A), - C / (2 * A), -np.sqrt((B**2 + C**2 - 4 * A * D) / (4 * A**2)))
      else:
        return (-B / (2 * A), - C / (2 * A), np.sqrt((B**2 + C**2 - 4 * A * D) / (4 * A**2)))
    else:
      return (-B / (2 * A), - C / (2 * A), 10000)
  except RuntimeWarning:
    return x2, y2, 10000

def parse_speed_unit(max_speed):
  """Converts a maxspeed string to m/s based on the unit present in the input.
  OpenStreetMap defaults to kph if no unit is present. """

  if not max_speed:
    return None

  conversion = CV.KPH_TO_MS
  if 'mph' in max_speed:
    max_speed = max_speed.replace(' mph', '')
    conversion = CV.MPH_TO_MS
  try:
    return float(max_speed) * conversion
  except ValueError:
    return None

def parse_speed_tags(tags):
  """Parses tags on a way to find the maxspeed string"""
  max_speed = None

  if 'maxspeed' in tags:
    max_speed = tags['maxspeed']

  if 'maxspeed:conditional' in tags:
    try:
      weekday = True
      max_speed_cond, cond = tags['maxspeed:conditional'].split(' @ ')
      if cond.find('wet') > -0.5:
        cond = cond.replace('wet','')
        cond = cond.replace(' ','')
        weekday = False
        #TODO Check if road is wet waybe if wipers are on.
      cond = cond[1:-1]
      
      now = datetime.now()  # TODO: Get time and timezone from gps fix so this will work correctly on replays
      if cond.find('Mo-Fr') > -0.5:
        cond = cond.replace('Mo-Fr','')
        cond = cond.replace(' ','')
        if now.weekday() > 4:
          weekday = False
      if cond.find('Mo-Su') > -0.5:
        cond = cond.replace('Mo-Su','')
        cond = cond.replace(' ','')
      if cond.find('; SH off') > -0.5:
        cond = cond.replace('; SH off','')
        cond = cond.replace(' ','')
      if cond.find('Oct-Apr') > -0.5:
        if 4 > now.month > 10:
          weekday = False
        else:
          max_speed = max_speed_cond
      else:
        start, end = cond.split('-')
        starthour, startminute = start.split(':')
        endhour, endminute = end.split(':')
        start = datetime.strptime(start, "%H:%M").replace(year=now.year, month=now.month, day=now.day)
        midnight = datetime.strptime("00:00", "%H:%M").replace(year=now.year, month=now.month, day=now.day)
        end1 = datetime.strptime(end, "%H:%M").replace(year=now.year, month=now.month, day=now.day)
        if int(endhour) + int(endminute)/60 < int(starthour) + int(startminute)/60:
          end2 = datetime.strptime(end, "%H:%M").replace(year=now.year, month=now.month, day=now.day+1)
          if start <= now <= end2 or midnight <= now <= end1 and weekday:
            max_speed = max_speed_cond
        else:
          if start <= now <= end1 and weekday:
            max_speed = max_speed_cond
    except ValueError:
      pass

  if not max_speed and 'source:maxspeed' in tags:
    max_speed = DEFAULT_SPEEDS.get(tags['source:maxspeed'], None)
  if not max_speed and 'maxspeed:type' in tags:
    max_speed = DEFAULT_SPEEDS.get(tags['maxspeed:type'], None)

  max_speed = parse_speed_unit(max_speed)
  return max_speed

def geocode_maxspeed(tags, location_info):
  max_speed = None
  try:
    geocode_country = location_info.get('country', '')
    geocode_region = location_info.get('region', '')

    country_rules = DEFAULT_SPEEDS_BY_REGION.get(geocode_country, {})
    country_defaults = country_rules.get('Default', [])
    for rule in country_defaults:
      rule_valid = all(
        tag_name in tags
        and tags[tag_name] == value
        for tag_name, value in rule['tags'].items()
      )
      if rule_valid:
        max_speed = rule['speed']
        break #stop searching country

    region_rules = country_rules.get(geocode_region, [])
    for rule in region_rules:
      rule_valid = all(
        tag_name in tags
        and tags[tag_name] == value
        for tag_name, value in rule['tags'].items()
      )
      if rule_valid:
        max_speed = rule['speed']
        break #stop searching region
  except KeyError:
    pass
  max_speed = parse_speed_unit(max_speed)
  return max_speed

class Way:
  def __init__(self, way, query_results):
    self.id = way.id
    self.way = way
    self.query_results = query_results

    points = list()

    for node in self.way.get_nodes(resolve_missing=False):
      points.append((float(node.lat), float(node.lon), 0.))

    self.points = np.asarray(points)

  @classmethod
  def closest(cls, query_results, lat, lon, heading, prev_way=None):
    if query_results is None:
      return None
    else:
      results, tree, real_nodes, node_to_way, location_info = query_results

    cur_pos = geodetic2ecef((lat, lon, 0))
    nodes = tree.query_ball_point(cur_pos, 500)

    # If no nodes within 500m, choose closest one
    if not nodes:
      nodes = [tree.query(cur_pos)[1]]

    ways = []
    for n in nodes:
      real_node = real_nodes[n]
      ways += node_to_way[real_node.id]
    ways = set(ways)

    closest_way = None
    best_score = None
    for way in ways:
      way = Way(way, query_results)
      points = way.points_in_car_frame(lat, lon, heading, True)

      on_way = way.on_way(lat, lon, heading, points)
      if not on_way:
        continue

      # Create mask of points in front and behind
      x = points[:, 0]
      y = points[:, 1]
      angles = np.arctan2(y, x)
      front = np.logical_and((-np.pi / 2) < angles,
                                angles < (np.pi / 2))
      behind = np.logical_not(front)

      dists = np.linalg.norm(points, axis=1)

      # Get closest point behind the car
      dists_behind = np.copy(dists)
      dists_behind[front] = np.NaN
      closest_behind = points[np.nanargmin(dists_behind)]

      # Get closest point in front of the car
      dists_front = np.copy(dists)
      dists_front[behind] = np.NaN
      closest_front = points[np.nanargmin(dists_front)]

      # fit line: y = a*x + b
      x1, y1, _ = closest_behind
      x2, y2, _ = closest_front
      a = (y2 - y1) / max((x2 - x1), 1e-5)
      b = y1 - a * x1

      # With a factor of 60 a 20m offset causes the same error as a 20 degree heading error
      # (A 20 degree heading offset results in an a of about 1/3)
      score = abs(a) * 60. + abs(b)

      # Prefer same type of road
      if prev_way is not None:
        if way.way.tags.get('highway', '') == prev_way.way.tags.get('highway', ''):
          score *= 0.5

      if closest_way is None or score < best_score:
        closest_way = way
        best_score = score
        
    if best_score is None:
      return None
    
    # Normal score is < 5
    if best_score > 50:
      return None

    return closest_way

  def __str__(self):
    return "%s %s" % (self.id, self.way.tags)

  def max_speed(self, heading):
    """Extracts the (conditional) speed limit from a way"""
    if not self.way:
      return None
    angle=heading - math.atan2(self.way.nodes[0].lon-self.way.nodes[-1].lon,self.way.nodes[0].lat-self.way.nodes[-1].lat)*180/3.14159265358979 - 180
    if angle < -180:
      angle = angle + 360
    if angle > 180:
      angle = angle - 360
    backwards = abs(angle) > 90
    if backwards:
      if 'maxspeed:backward' in self.way.tags:
        max_speed = self.way.tags['maxspeed:backward']
        max_speed = parse_speed_unit(max_speed)
        return max_speed
    else:
      if 'maxspeed:forward' in self.way.tags:
        max_speed = self.way.tags['maxspeed:forward']
        max_speed = parse_speed_unit(max_speed)
        return max_speed

    max_speed = parse_speed_tags(self.way.tags)
    if not max_speed:
      location_info = self.query_results[4]
      max_speed = geocode_maxspeed(self.way.tags, location_info)

    return max_speed

  def max_speed_ahead(self, current_speed_limit, lat, lon, heading, lookahead):
    """Look ahead for a max speed"""
    if not self.way:
      return None

    speed_ahead = None
    speed_ahead_dist = None
    lookahead_ways = 5
    way = self
    for i in range(lookahead_ways):
      way_pts = way.points_in_car_frame(lat, lon, heading, True)
      #print way_pts
      # Check current lookahead distance
      if way_pts[0,0] < 0 and way_pts[-1,0] < 0:
        break
      elif way_pts[0,0] < 0:
        max_dist = np.linalg.norm(way_pts[-1, :])
      elif way_pts[-1,0] < 0:
        max_dist = np.linalg.norm(way_pts[0, :])
      else:
        max_dist = min(np.linalg.norm(way_pts[1, :]),np.linalg.norm(way_pts[0, :]),np.linalg.norm(way_pts[-1, :]))
         

      if max_dist > 2 * lookahead:
        #print "max_dist break"
        break
      try:
        if way.way.tags['junction']=='roundabout' or way.way.tags['junction']=='circular':
          latmin = 181
          lonmin = 181
          latmax = -181
          lonmax = -181
          for n in way.way.nodes:
            lonmax = max(n.lon,lonmax)
            lonmin = min(n.lon,lonmin)
            latmax = max(n.lat,latmax)
            latmin = min(n.lat,latmin)
          if way.way.nodes[0].id == way.way.nodes[-1].id:
            a = 111132.954*math.cos(float(latmax+latmin)/360*3.141592)*float(lonmax-lonmin)
          else:
            if way.way.nodes[1].id == way.way.nodes[-1].id:
              circle = [0,0,30]
            else:
              circle = circle_through_points([way.way.nodes[0].lat,way.way.nodes[0].lon,1], [way.way.nodes[1].lat,way.way.nodes[1].lon,1], [way.way.nodes[-1].lat,way.way.nodes[-1].lon,1],True)
            a = 111132.954*math.cos(float(latmax+latmin)/360*3.141592)*float(circle[2])*2
          speed_ahead = np.sqrt(1.6075*a)
          min_dist = 999.9
          for w in way_pts:
            min_dist = min(min_dist, float(np.linalg.norm(w)))
          speed_ahead_dist = min_dist
          break
      except KeyError:
        pass
      angle=heading - math.atan2(way.way.nodes[0].lon-way.way.nodes[-1].lon,way.way.nodes[0].lat-way.way.nodes[-1].lat)*180/3.14159265358979 - 180
      if angle < -180:
        angle = angle + 360
      if angle > 180:
        angle = angle - 360
      backwards = abs(angle) > 90
      if backwards:
        if 'maxspeed:backward' in way.way.tags:
          spd = way.way.tags['maxspeed:backward']
          spd = parse_speed_unit(spd)
          if spd is not None and spd < current_speed_limit:
            speed_ahead = spd
            min_dist = min(np.linalg.norm(way_pts[1, :]),np.linalg.norm(way_pts[0, :]),np.linalg.norm(way_pts[-1, :]))
            speed_ahead_dist = min_dist
            break
      else:
        if 'maxspeed:forward' in way.way.tags:
          spd = way.way.tags['maxspeed:forward']
          spd = parse_speed_unit(spd)
          if spd is not None and spd < current_speed_limit:
            speed_ahead = spd
            min_dist = min(np.linalg.norm(way_pts[1, :]),np.linalg.norm(way_pts[0, :]),np.linalg.norm(way_pts[-1, :]))
            speed_ahead_dist = min_dist
            break
      if 'maxspeed' in way.way.tags:
        spd = parse_speed_tags(way.way.tags)
        #print "spd found"
        #print spd
        if not spd:
          location_info = self.query_results[4]
          spd = geocode_maxspeed(way.way.tags, location_info)
          #print "spd is actually"
          #print spd
        if spd is not None and spd < current_speed_limit:
          speed_ahead = spd
          min_dist = min(np.linalg.norm(way_pts[1, :]),np.linalg.norm(way_pts[0, :]),np.linalg.norm(way_pts[-1, :]))
          speed_ahead_dist = min_dist
          #print "slower speed found"
          #print min_dist
          
          break
      way_pts = way.points_in_car_frame(lat, lon, heading, False)
      #print(way_pts)

      try:
        count = 0
        loop_must_break = False
        for n in way.way.nodes:
          if 'highway' in n.tags and (n.tags['highway']=='stop' or n.tags['highway']=='give_way' or n.tags['highway']=='mini_roundabout' or n.tags['highway']=='traffic_signals') and way_pts[count,0] > 0:
            if 'direction' in n.tags:
              if backwards and (n.tags['direction']=='backward' or n.tags['direction']=='both'):
                print("backward")
                if way_pts[count, 0] > 0:
                  speed_ahead_dist = max(0. , way_pts[count, 0] - 1.0)
                  print(speed_ahead_dist)
                  speed_ahead = 5/3.6
                  if n.tags['highway']=='stop':
                    speed_ahead = 0
                  loop_must_break = True
                  break
              elif not backwards and (n.tags['direction']=='forward' or n.tags['direction']=='both'):
                print("forward")
                if way_pts[count, 0] > 0:
                  speed_ahead_dist = max(0. , way_pts[count, 0] - 1.0)
                  print(speed_ahead_dist)
                  speed_ahead = 5/3.6
                  if n.tags['highway']=='stop':
                    speed_ahead = 0
                  loop_must_break = True
                  break
              try:
                if int(n.tags['direction']) > -0.1 and int(n.tags['direction']) < 360.1:
                  print(int(n.tags['direction']))
                  direction = int(n.tags['direction']) - heading
                  if direction < -180:
                    direction = direction + 360
                  if direction > 180:
                    direction = direction - 360
                  if abs(direction) > 135:
                    speed_ahead_dist = max(0. , way_pts[count, 0] - 1.0)
                    print(speed_ahead_dist)
                    speed_ahead = 5/3.6
                    if n.tags['highway']=='stop':
                      speed_ahead = 0
                    loop_must_break = True
                    break
              except (KeyError, ValueError):
                pass
            elif 'traffic_signals:direction' in n.tags:
              if backwards and (n.tags['traffic_signals:direction']=='backward' or n.tags['traffic_signals:direction']=='both'):
                print("backward")
                if way_pts[count, 0] > 0:
                  speed_ahead_dist = max(0. , way_pts[count, 0] - 4.0)
                  print(speed_ahead_dist)
                  speed_ahead = 5/3.6
                  if n.tags['highway']=='traffic_signals':
                    speed_ahead = 0
                  loop_must_break = True
                  break
              elif not backwards and (n.tags['traffic_signals:direction']=='forward' or n.tags['traffic_signals:direction']=='both'):
                print("forward")
                if way_pts[count, 0] > 0:
                  speed_ahead_dist = max(0. , way_pts[count, 0] - 4.0)
                  print(speed_ahead_dist)
                  speed_ahead = 5/3.6
                  if n.tags['highway']=='traffic_signals':
                    speed_ahead = 0
                  loop_must_break = True
                  break
              try:
                if int(n.tags['traffic_signals:direction']) > -0.1 and int(n.tags['traffic_signals:direction']) < 360.1:
                  print(int(n.tags['traffic_signals:direction']))
                  direction = int(n.tags['traffic_signals:direction']) - heading
                  if direction < -180:
                    direction = direction + 360
                  if direction > 180:
                    direction = direction - 360
                  if abs(direction) > 135:
                    speed_ahead_dist = max(0. , way_pts[count, 0] - 4.0)
                    print(speed_ahead_dist)
                    speed_ahead = 5/3.6
                    if n.tags['highway']=='traffic_signals':
                      speed_ahead = 0
                    loop_must_break = True
                    break
              except (KeyError, ValueError):
                pass
            else:
              if n.tags['highway']=='mini_roundabout':
                if way_pts[count, 0] > 0:
                  speed_ahead_dist = max(0. , way_pts[count, 0] - 5.0)
                  print(speed_ahead_dist)
                  speed_ahead = 15/3.6
                  loop_must_break = True
                  break
              if way_pts[count, 0] > 0 and Traffic_Debug:
                print("no direction")
                speed_ahead_dist = max(0. , way_pts[count, 0] - 10.0)
                print(speed_ahead_dist)
                speed_ahead = 5/3.6
                if n.tags['highway']=='stop':
                  speed_ahead = 0
                loop_must_break = True
                break
          if 'railway' in n.tags and n.tags['railway']=='level_crossing':
            if way_pts[count, 0] > 0:
              speed_ahead = 0
              speed_ahead_dist = max(0. , way_pts[count, 0] - 10.0)
              loop_must_break = True
              break
          if 'traffic_calming' in n.tags:
            if way_pts[count, 0] > 0:
              if n.tags['traffic_calming']=='bump' or n.tags['traffic_calming']=='hump':
                speed_ahead = 2.24
                speed_ahead_dist = way_pts[count, 0]
                loop_must_break = True
                break
              elif n.tags['traffic_calming']=='chicane' or n.tags['traffic_calming']=='choker':
                speed_ahead = 20/3.6
                speed_ahead_dist = way_pts[count, 0]
                loop_must_break = True
                break
          count += 1
        if loop_must_break: break
      except (KeyError, IndexError, ValueError):
        pass
      # Find next way
      way = way.next_way(heading)
      if not way:
        #print "no way break"
        break

    return speed_ahead, speed_ahead_dist

  def advisory_max_speed(self):
    if not self.way:
      return None

    tags = self.way.tags
    adv_speed = None

    if 'maxspeed:advisory' in tags:
      adv_speed = tags['maxspeed:advisory']
      adv_speed = parse_speed_unit(adv_speed)
    return adv_speed

  def on_way(self, lat, lon, heading, points=None):
    if points is None:
      points = self.points_in_car_frame(lat, lon, heading, True)
    x = points[:, 0]
    return np.min(x) < 0. and np.max(x) > 0.

  def closest_point(self, lat, lon, heading, points=None):
    if points is None:
      points = self.points_in_car_frame(lat, lon, heading, True)
    i = np.argmin(np.linalg.norm(points, axis=1))
    return points[i]

  def distance_to_closest_node(self, lat, lon, heading, points=None):
    if points is None:
      points = self.points_in_car_frame(lat, lon, heading, True)
    return np.min(np.linalg.norm(points, axis=1))

  def points_in_car_frame(self, lat, lon, heading, flip):
    lc = LocalCoord.from_geodetic([lat, lon, 0.])

    # Build rotation matrix
    heading = math.radians(-heading + 90)
    c, s = np.cos(heading), np.sin(heading)
    rot = np.array([[c, s, 0.], [-s, c, 0.], [0., 0., 1.]])

    # Convert to local coordinates
    points_carframe = lc.geodetic2ned(self.points).T

    # Rotate with heading of car
    points_carframe = np.dot(rot, points_carframe[(1, 0, 2), :]).T
    
    if points_carframe[-1,0] < points_carframe[0,0] and flip:
      points_carframe = np.flipud(points_carframe)
      
    return points_carframe

  def next_way(self, heading):
    results, tree, real_nodes, node_to_way, location_info = self.query_results
    #print "way.id"
    #print self.id
    #print "node0.id"
    #print self.way.nodes[0].id
    #print "node-1.id"
    #print self.way.nodes[-1].id
    #print "heading"
    #print heading
    angle=heading - math.atan2(self.way.nodes[0].lon-self.way.nodes[-1].lon,self.way.nodes[0].lat-self.way.nodes[-1].lat)*180/3.14159265358979 - 180
    #print "angle before"
    #print angle
    if angle < -180:
      angle = angle + 360
    if angle > 180:
      angle = angle - 360
    #print "angle"
    #print angle
    backwards = abs(angle) > 90
    #print "backwards"
    #print backwards
    if backwards:
      node = self.way.nodes[0]
    else:
      node = self.way.nodes[-1]

    ways = node_to_way[node.id]

    way = None
    try:
      # Simple heuristic to find next way
      ways = [w for w in ways if w.id != self.id]
      if len(ways) == 1:
        way = Way(ways[0], self.query_results)
        #print "only one way found"
        return way
      if len(ways) == 2:
        try:
          if ways[0].tags['junction']=='roundabout' or ways[0].tags['junction']=='circular':
            #print ("roundabout found")
            way = Way(ways[0], self.query_results)
            return way
        except (KeyError, IndexError):
          pass
        try:
          if (ways[0].tags['oneway'] == 'yes') and (ways[1].tags['oneway'] == 'yes'):
            if (ways[0].nodes[0].id == node.id and ways[1].nodes[0].id != node.id) and not (ways[0].nodes[0].id != node.id and ways[1].nodes[0].id == node.id):
              way = Way(ways[0], self.query_results)
              return way
            elif (ways[0].nodes[0].id != node.id and ways[1].nodes[0].id == node.id) and not (ways[0].nodes[0].id == node.id and ways[1].nodes[0].id != node.id):
              way = Way(ways[1], self.query_results)
              return way
        except (KeyError, IndexError):
          pass
      ways = [w for w in ways if (w.nodes[0] == node or w.nodes[-1] == node)]
      if len(ways) == 1:
        way = Way(ways[0], self.query_results)
        #print "only one way found"
        return way
      # Filter on highway tag
      acceptable_tags = list()
      cur_tag = self.way.tags['highway']
      acceptable_tags.append(cur_tag)
      if cur_tag == 'motorway_link':
        acceptable_tags.append('motorway')
        acceptable_tags.append('trunk')
        acceptable_tags.append('primary')
      ways = [w for w in ways if w.tags['highway'] in acceptable_tags]
      if len(ways) == 1:
        way = Way(ways[0], self.query_results)
        #print "only one way found"
        return way
      if len(ways) == 2:
        try:
          if ways[0].tags['junction']=='roundabout' or ways[0].tags['junction']=='circular':
            #print ("roundabout found")
            way = Way(ways[0], self.query_results)
            return way
        except (KeyError, IndexError):
          pass
        try:
          if (ways[0].tags['oneway'] == 'yes') and (ways[1].tags['oneway'] == 'yes'):
            if (ways[0].nodes[0].id == node.id and ways[1].nodes[0].id != node.id) and not (ways[0].nodes[0].id != node.id and ways[1].nodes[0].id == node.id):
              way = Way(ways[0], self.query_results)
              return way
            elif (ways[0].nodes[0].id != node.id and ways[1].nodes[0].id == node.id) and not (ways[0].nodes[0].id == node.id and ways[1].nodes[0].id != node.id):
              way = Way(ways[1], self.query_results)
              return way
        except (KeyError, IndexError):
          pass
      # Filter on number of lanes
      cur_num_lanes = int(self.way.tags['lanes'])
      if len(ways) > 1:
        ways_same_lanes = [w for w in ways if int(w.tags['lanes']) == cur_num_lanes]
        if len(ways_same_lanes) == 1:
          ways = ways_same_lanes
      if len(ways) > 1:
        ways = [w for w in ways if int(w.tags['lanes']) > cur_num_lanes]
      if len(ways) == 1:
        way = Way(ways[0], self.query_results)

    except (KeyError, ValueError):
      pass

    return way

  def get_lookahead(self, lat, lon, heading, lookahead):
    pnts = None
    way = self
    valid = False
    
    for i in range(5):
      # Get new points and append to list
      new_pnts = way.points_in_car_frame(lat, lon, heading, True)

      try:
        if way.way.tags['junction']=='roundabout' or way.way.tags['junction']=='circular':
          break
      except KeyError:
        pass
      if pnts is None:
        pnts = new_pnts
        valid = True
      else:
        new_pnts = np.delete(new_pnts,[0,0,0], axis=0)
        pnts = np.vstack([pnts, new_pnts])

      # Check current lookahead distance
      max_dist = np.linalg.norm(pnts[-1, :])

      if max_dist > 2 * lookahead:
        break

      # Find next way
      startid = way.way.nodes[0].id
      endid = way.way.nodes[-1].id
      way = way.next_way(heading)
      if not way:
        break
      if not (way.way.nodes[0].id == startid or way.way.nodes[0].id == endid or way.way.nodes[-1].id == startid or way.way.nodes[-1].id == endid):
        break
    return pnts, valid
