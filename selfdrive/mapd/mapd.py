#!/usr/bin/env python3

# Add phonelibs openblas to LD_LIBRARY_PATH if import fails
from scipy import spatial
import selfdrive.crash as crash

#DEFAULT_SPEEDS_BY_REGION_JSON_FILE = BASEDIR + "/selfdrive/mapd/default_speeds_by_region.json"
#from selfdrive.mapd import default_speeds_generator
#default_speeds_generator.main(DEFAULT_SPEEDS_BY_REGION_JSON_FILE)

import time
import zmq
import requests
import threading
import numpy as np
import overpy
from cereal import arne182
from common.params import Params
from collections import defaultdict
from selfdrive.version import version, dirty

from common.transformations.coordinates import geodetic2ecef
import selfdrive.mapd.messaging as messaging
from selfdrive.mapd.mapd_helpers import MAPS_LOOKAHEAD_DISTANCE, Way, circle_through_points, rate_curvature_points

OVERPASS_API_URL = "https://z.overpass-api.de/api/interpreter"
OVERPASS_API_URL2 = "https://lz4.overpass-api.de/api/interpreter"
OVERPASS_HEADERS = {
    'User-Agent': 'NEOS (comma.ai)',
    'Accept-Encoding': 'gzip'
}

last_gps = None
query_lock = threading.Lock()
last_query_result = None
last_query_pos = None
cache_valid = False

def connected_to_internet(url='https://z.overpass-api.de/api/interpreter', timeout=5):
    try:
        requests.get(url, timeout=timeout)
        return True
    except:
        print("No internet connection available.")
    return False

def connected_to_internet2(url='https://lz4.overpass-api.de/api/interpreter', timeout=5):
    try:
        requests.get(url, timeout=timeout)
        return True
    except:
        print("No internet connection available.")
    return False

def build_way_query(lat, lon, radius=50):
  """Builds a query to find all highways within a given radius around a point"""
  pos = "  (around:%f,%f,%f)" % (radius, lat, lon)
  lat_lon = "(%f,%f)" % (lat, lon)
  q = """(
  way
  """ + pos + """
  [highway][highway!~"^(footway|path|bridleway|steps|cycleway|construction|bus_guideway|escape)$"];
  >;);out;""" + """is_in""" + lat_lon + """;area._[admin_level~"[24]"];
  convert area ::id = id(), admin_level = t['admin_level'],
  name = t['name'], "ISO3166-1:alpha2" = t['ISO3166-1:alpha2'];out;
  """
  return q


def query_thread():
  global last_query_result, last_query_pos, cache_valid
  #api = overpy.Overpass(url=OVERPASS_API_URL, headers=OVERPASS_HEADERS, timeout=20.)
  api = overpy.Overpass(url=OVERPASS_API_URL)
  
  while True:
    time.sleep(1)
    if last_gps is not None:
      fix_ok = last_gps.flags & 1
      if not fix_ok:
        continue

      if last_query_pos is not None:
        cur_ecef = geodetic2ecef((last_gps.latitude, last_gps.longitude, last_gps.altitude))
        prev_ecef = geodetic2ecef((last_query_pos.latitude, last_query_pos.longitude, last_query_pos.altitude))
        dist = np.linalg.norm(cur_ecef - prev_ecef)
        if dist < 3000: #updated when we are 1km from the edge of the downloaded circle
          continue

        if dist > 4000:
          cache_valid = False

      q = build_way_query(last_gps.latitude, last_gps.longitude, radius=4000)
      if connected_to_internet() or connected_to_internet2():
        try:
          try:
            new_result = api.query(q)
          except:
            api2 = overpy.Overpass(url=OVERPASS_API_URL2)
            print("Using backup Server")
            new_result = api2.query(q)
  
          # Build kd-tree
          nodes = []
          real_nodes = []
          node_to_way = defaultdict(list)
          location_info = {}
  
          for n in new_result.nodes:
            nodes.append((float(n.lat), float(n.lon), 0))
            real_nodes.append(n)
  
          for way in new_result.ways:
            for n in way.nodes:
              node_to_way[n.id].append(way)
  
          for area in new_result.areas:
            if area.tags.get('admin_level', '') == "2":
              location_info['country'] = area.tags.get('ISO3166-1:alpha2', '')
            if area.tags.get('admin_level', '') == "4":
              location_info['region'] = area.tags.get('name', '')
  
          nodes = np.asarray(nodes)
          nodes = geodetic2ecef(nodes)
          tree = spatial.cKDTree(nodes)
  
          query_lock.acquire()
          last_query_result = new_result, tree, real_nodes, node_to_way, location_info
          last_query_pos = last_gps
          cache_valid = True
          query_lock.release()

        except Exception as e:
          print(e)
          crash.capture_warning(e)
          query_lock.acquire()
          last_query_result = None
          query_lock.release()
      else:
        query_lock.acquire()
        last_query_result = None
        query_lock.release()

def save_gps_data(gps):
  try:
    location = [gps.speed, gps.bearing, gps.latitude, gps.longitude, gps.altitude, gps.accuracy, time.time()]
    with open("/data/openpilot/selfdrive/data_collection/gps-data", "a") as f:
      f.write("{}\n".format(location))
  except:
    pass

def mapsd_thread():
  global last_gps
  context = zmq.Context()
  poller = zmq.Poller()
  gps_external_sock = messaging.sub_sock(context, 8032, poller, conflate=True)
  map_data_sock = messaging.pub_sock(context, 8065)
  traffic_data_sock = messaging.sub_sock(context, 8208, poller, conflate=True)

  cur_way = None
  curvature_valid = False
  curvature = None
  upcoming_curvature = 0.
  dist_to_turn = 0.
  road_points = None
  speedLimittraffic = 0
  speedLimittraffic_prev = 0
  max_speed = None
  max_speed_ahead = None
  max_speed_ahead_dist = None
  
  max_speed_prev = 0
  speedLimittrafficvalid = False
  
  while True:
    gps_ext = None
    traffic = None
    for socket, event in poller.poll(0):
      if socket is gps_external_sock:
        gps_ext = messaging.recv_one(socket)
      elif socket is traffic_data_sock:
        traffic = arne182.LiveTrafficData.from_bytes(socket.recv())
    if traffic is not None:
      if traffic.speedLimitValid:
        speedLimittraffic = traffic.speedLimit
        if abs(speedLimittraffic_prev - speedLimittraffic) > 0.1:
          speedLimittrafficvalid = True
          speedLimittraffic_prev = speedLimittraffic
      else:
        speedLimittrafficvalid = False
      if traffic.speedAdvisoryValid:
        speedLimittrafficAdvisory = traffic.speedAdvisory
        speedLimittrafficAdvisoryvalid = True
      else:
        speedLimittrafficAdvisoryvalid = False
    else:
      speedLimittrafficAdvisoryvalid = False
      
    if gps_ext is not None:
      gps = gps_ext.gpsLocationExternal
    else:
      continue

    save_gps_data(gps)

    last_gps = gps
    
    fix_ok = gps.flags & 1
    
    if gps.accuracy > 2.0 and not speedLimittrafficvalid:
      fix_ok = False
    if not fix_ok or last_query_result is None or not cache_valid:
      cur_way = None
      curvature = None
      max_speed_ahead = None
      max_speed_ahead_dist = None
      curvature_valid = False
      upcoming_curvature = 0.
      dist_to_turn = 0.
      road_points = None
      map_valid = False
    else:
      map_valid = True
      lat = gps.latitude
      lon = gps.longitude
      heading = gps.bearing
      speed = gps.speed

      query_lock.acquire()
      cur_way = Way.closest(last_query_result, lat, lon, heading, cur_way)
      if cur_way is not None:
        pnts, curvature_valid = cur_way.get_lookahead(lat, lon, heading, MAPS_LOOKAHEAD_DISTANCE)
        if pnts is not None:
          xs = pnts[:, 0]
          ys = pnts[:, 1]
          road_points = [float(x) for x in xs], [float(y) for y in ys]

          if speed < 5:
            curvature_valid = False
          if curvature_valid and pnts.shape[0] <= 3:
            curvature_valid = False
        else:
          curvature_valid = False
          upcoming_curvature = 0.
          curvature = None
          dist_to_turn = 0.
        # The curvature is valid when at least MAPS_LOOKAHEAD_DISTANCE of road is found
        if curvature_valid:
          # Compute the curvature for each point
          with np.errstate(divide='ignore'):
            circles = [circle_through_points(*p, direction=True) for p in zip(pnts, pnts[1:], pnts[2:])]
            circles = np.asarray(circles)
            radii = np.nan_to_num(circles[:, 2])
            radii[radii < 15.] = 10000
            
            if cur_way.way.tags['highway'] == 'trunk':
              radii = radii*1.6 # https://media.springernature.com/lw785/springer-static/image/chp%3A10.1007%2F978-3-658-01689-0_21/MediaObjects/298553_35_De_21_Fig65_HTML.gif
            if cur_way.way.tags['highway'] == 'motorway' or  cur_way.way.tags['highway'] == 'motorway_link':
              radii = radii*2.8
         
            curvature = 1. / radii
          rate = [rate_curvature_points(*p) for p in zip(pnts[1:], pnts[2:],curvature[0:],curvature[1:])]
          rate = ([0] + rate)
          curvature = np.multiply(np.minimum(np.multiply(rate,4000)+0.7,1.1),curvature)
          # Index of closest point
          closest = np.argmin(np.linalg.norm(pnts, axis=1))
          dist_to_closest = pnts[closest, 0]  # We can use x distance here since it should be close

          # Compute distance along path
          dists = list()
          dists.append(0)
          for p, p_prev in zip(pnts, pnts[1:, :]):
            dists.append(dists[-1] + np.linalg.norm(p - p_prev))
          dists = np.asarray(dists)
          dists = dists - dists[closest] + dist_to_closest
          dists = dists[1:-1]

          close_idx = np.logical_and(dists > 0, dists < 500)
          dists = dists[close_idx]
          curvature = curvature[close_idx]

          if len(curvature):
            curvature = np.nan_to_num(curvature)

            
            
            upcoming_curvature = np.amax(curvature)
            dist_to_turn =np.amin(dists[np.logical_and(curvature >= np.amax(curvature), curvature <= np.amax(curvature))])

            
          else:
            upcoming_curvature = 0.
            dist_to_turn = 999

      query_lock.release()

    dat = messaging.new_message()
    dat.init('liveMapData')

    if last_gps is not None:
      dat.liveMapData.lastGps = last_gps

    if cur_way is not None:
      dat.liveMapData.wayId = cur_way.id

      # Speed limit
      max_speed = cur_way.max_speed(heading)
      max_speed_ahead = None
      max_speed_ahead_dist = None
      if max_speed is not None:
        max_speed_ahead, max_speed_ahead_dist = cur_way.max_speed_ahead(max_speed, lat, lon, heading, MAPS_LOOKAHEAD_DISTANCE)
      else:
        max_speed_ahead, max_speed_ahead_dist = cur_way.max_speed_ahead(speed*1.1, lat, lon, heading, MAPS_LOOKAHEAD_DISTANCE)
      # TODO: anticipate T junctions and right and left hand turns based on indicator
        
      if max_speed_ahead is not None and max_speed_ahead_dist is not None:
        dat.liveMapData.speedLimitAheadValid = True
        dat.liveMapData.speedLimitAhead = float(max_speed_ahead)
        dat.liveMapData.speedLimitAheadDistance = float(max_speed_ahead_dist)
      if max_speed is not None:
        if abs(max_speed - max_speed_prev) > 0.1:
          speedLimittrafficvalid = False
          max_speed_prev = max_speed
      advisory_max_speed = cur_way.advisory_max_speed()
      if speedLimittrafficAdvisoryvalid:
        dat.liveMapData.speedAdvisoryValid = True
        dat.liveMapData.speedAdvisory = speedLimittrafficAdvisory / 3.6
      else:
        if advisory_max_speed is not None:
          dat.liveMapData.speedAdvisoryValid = True
          dat.liveMapData.speedAdvisory = advisory_max_speed
      
      # Curvature
      dat.liveMapData.curvatureValid = curvature_valid
      dat.liveMapData.curvature = float(upcoming_curvature)
      dat.liveMapData.distToTurn = float(dist_to_turn)
      if road_points is not None:
        dat.liveMapData.roadX, dat.liveMapData.roadY = road_points
      if curvature is not None:
        dat.liveMapData.roadCurvatureX = [float(x) for x in dists]
        dat.liveMapData.roadCurvature = [float(x) for x in curvature]

    if speedLimittrafficvalid:
      if speedLimittraffic > 0.1:
        dat.liveMapData.speedLimitValid = True
        dat.liveMapData.speedLimit = speedLimittraffic / 3.6
        map_valid = False
      else:
        speedLimittrafficvalid = False
    else:
      if max_speed is not None and map_valid:
        dat.liveMapData.speedLimitValid = True
        dat.liveMapData.speedLimit = max_speed
        
    dat.liveMapData.mapValid = map_valid

    map_data_sock.send(dat.to_bytes())


def main(gctx=None):
  params = Params()
  dongle_id = params.get("DongleId")
  crash.bind_user(id=dongle_id)
  crash.bind_extra(version=version, dirty=dirty, is_eon=True)
  crash.install()

  main_thread = threading.Thread(target=mapsd_thread)
  main_thread.daemon = True
  main_thread.start()

  q_thread = threading.Thread(target=query_thread)
  q_thread.daemon = True
  q_thread.start()

  while True:
    time.sleep(0.1)


if __name__ == "__main__":
  main()
