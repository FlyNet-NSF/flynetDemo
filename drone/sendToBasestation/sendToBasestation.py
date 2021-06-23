#!/usr/bin/python3
import requests
import json
import geojson
import time
import sys
import os
import pika
import threading
import configparser
import random
from datetime import datetime
from argparse import ArgumentParser
from os import path
from geopy.distance import lonlat, distance, Distance
from geopy import Point
from geographiclib.geodesic import Geodesic
from distutils.util import strtobool

# constants
networks = ("verizon", "att", "sprint", "comcast")
worker_interface = "eth2"

def readConfig(file):
  config = configparser.ConfigParser()
  config.read(os.path.join(os.path.dirname(__file__), file))
  return config['properties']

def main(args):
  credentials = pika.PlainCredentials(args.rabbituser, args.rabbitpass)
  baseconnection = pika.BlockingConnection(pika.ConnectionParameters(host=args.basestation_host, virtual_host=args.basestation_vhost, credentials=credentials))
  basechannel = baseconnection.channel()
  basechannel.queue_declare(queue=args.basestation_queue, durable=True)

  currentLat = 32.0
  currentLon = -96.0
  currentAlt = 500  # ft
  currentTuple = [currentLon, currentLat, currentAlt]
  currentBattery = random.randint(50, 100)
  endless = 0

  cell_towers = {}
  droneData = {}
  droneData['type'] = "Feature"
  droneData['properties'] = {}
  droneData['properties']['eventName'] = "FlyNetDemo"
  droneData['properties']['classification'] = "ongoingFlight"
  droneData['properties']['userProperties'] = {}
  droneData['properties']['userProperties']['celltowers'] = {}
  droneData['geometry'] = {}
  droneData['geometry']['type'] = "LineString"
  droneData['geometry']['coordinates'] = []
  droneData['geometry']['coordinates'].append(currentTuple)

  while currentBattery > 10:

    # update drone data
    droneData['properties']['userProperties']['batterylife'] = currentBattery

    # move drone
    prevLat = currentLat
    prevLon = currentLon

    currentLat = currentLat + .01
    currentLon = currentLon - .01
    currentTuple = [currentLon, currentLat, currentAlt]
    
    # calculate heading
    drone_change = Geodesic.WGS84.Inverse(currentLat, currentLon, prevLat, prevLon)
    drone_heading = drone_change['azi1']

    droneData['properties']['userProperties']['heading'] = drone_heading

    drone_point = Point(prevLat, prevLon, currentAlt)

    cell_towers = generateCellTowers(drone_point, cell_towers)  # regenerate cell towers

    for id,tower in cell_towers.items():
      towerDistanceCalc = Geodesic.WGS84.Inverse(currentLat, currentLon, tower['geometry']['coordinates'][1], tower['geometry']['coordinates'][0])
      towerDistance = towerDistanceCalc['s12']
      rtt = towerDistance / 1000  # calculate RTT with some randomness (factor of distance)
      tower['properties']['rtt'] = rtt

      if 'bw' not in tower:
        bw = round(random.random() * 35)  # in mb/s
        tower['properties']['bandwidth'] = bw

    droneData['properties']['userProperties']['celltowers'] = cell_towers

    # battery simulation
    currentBattery = currentBattery - 0.1

    #print(droneData)
    #droneMessage = str(droneData, 'utf-8')
    #droneMessage = droneData
    submitToBasestation(args, basechannel, droneData)

    time.sleep(5)
  
  print("Flight is complete.  Exiting")
  sys.exit()
  
def generateCellTowers(location, existing = {}):
  mincount = 4
  distance_limit = 10000  # meters from drone to cell tower
  out = existing

  def getGenCount():
    amount = mincount + round(random.random() * 4 - 2) - len(existing)
    if amount < 0:
      return 0
    else:
      return amount

  if len(existing) == 0:
    # first tower generation
    for i in range(getGenCount()):
      rand_distance = distance_limit * random.random()  # generate a distance on the edge of the range of the drone
      rel_bearing = random.random() * 360 - 180  # calculate a random relative heading between 180 and -180 degrees from the drone

      new_tower_dist = distance(kilometers=rand_distance / 1000)
      new_tower = new_tower_dist.destination(location, rel_bearing)  # find coordinates of new tower

      longitude = new_tower.longitude
      latitude = new_tower.latitude
      key = "ct_" + str(longitude) + "_" + str(latitude)
      out[key] = {}
      out[key]['type'] = "Feature"
      out[key]['geometry'] = {}
      out[key]['geometry']['type'] = "Point"
      this_tuple = [longitude, latitude, 0]
      out[key]['geometry']['coordinates'] = this_tuple
      out[key]['properties'] = {}
      out[key]['properties']['classification'] = "celltower"
      out[key]['properties']['eventName'] = key
      out[key]['properties']['network'] = random.choice(networks)
      
  else:
    delList = []
    # remove out of range cell towers
    for id,tower in existing.items():
      tower_lon = tower['geometry']['coordinates'][0]
      tower_lat = tower['geometry']['coordinates'][1]

      drone_tower_distance = Geodesic.WGS84.Inverse(location.latitude, location.longitude, tower_lat, tower_lon)['s12']  # calculate distance

      if drone_tower_distance > distance_limit:
        delList.append(id)

    
    for id in delList:
      del existing[id]

    # add stations if needed
    for i in range(getGenCount()):
      rand_distance = distance_limit - random.random() * (distance_limit / 200)  # generate a distance on the edge of the range of the drone
      rel_bearing = random.random() * 180 - 90  # calculate a random relative heading between -90 and 90 degrees from the drone

      new_tower_dist = distance(kilometers=rand_distance / 1000)
      new_tower = new_tower_dist.destination(location, rel_bearing)  # find coordinates of new tower

      longitude = new_tower.longitude
      latitude = new_tower.latitude
      key = "ct_" + str(longitude) + "_" + str(latitude)
      out[key] = {}
      out[key]['type'] = "Feature"
      out[key]['geometry'] = {}
      out[key]['geometry']['type'] = "Point"
      this_tuple = [longitude, latitude, 0]
      out[key]['geometry']['coordinates'] = this_tuple
      out[key]['properties'] = {}
      out[key]['properties']['tower'] = key
      out[key]['properties']['network'] = random.choice(networks)

  return out

def handleArguments(properties):
  parser = ArgumentParser()
  parser.add_argument("-u", "--rabbituser", dest="rabbituser", default=properties['rabbituser'],
                      type=str, help="The username for RabbitMQ.  Default is in the config file.")
  parser.add_argument("-p", "--rabbitpass", dest="rabbitpass", default=properties['rabbitpass'],
                      type=str, help="The password for RabbitMQ.  Default is in the config file.")
  parser.add_argument("-b", "--basestation-host", dest="basestation_host", default=properties['basestation_host'],
                      type=str, help="The host/IP address for the basestation RabbitMQ.  Default is in the config file.")
  parser.add_argument("-v", "--basestation-vhost", dest="basestation_vhost", default=properties['basestation_vhost'],
                      type=str, help="The virtual host for the basestation RabbitMQ.  Default is in the config file.")
  parser.add_argument("-q", "--basestation-queue", dest="basestation_queue", default=properties['basestation_queue'],
                          type=str, help="The basestation RabbitMQ queue name.  Default is in the config file.")
  parser.add_argument("-e", "--basestation-exchange", dest="basestation_exchange", default=properties['basestation_exchange'],
                          type=str, help="The basestation RabbitMQ exchange name.  Default is in the config file.")
  parser.add_argument("-n", "--noisy", dest="noisy", action='store_true',
                      help="Enable noisy output.")
  return parser.parse_args()

def daemonize():
    try:
        pid = os.fork()
    except OSError as e:
        raise Exception("%s [%d]" % (e.strerror, e.errno))

    if (pid == 0):
        os.setsid()
        try:
            pid = os.fork()    # Fork a second child.
        except OSError as e:
            raise Exception("%s [%d]" % (e.strerror, e.errno))
        if (pid == 0):
            os.chdir("/")
            os.umask(0)
        else:
            os._exit(0)
    else:
        os._exit(0)

def submitToBasestation(args, channel, message):
  channel.basic_publish(
    exchange=args.basestation_exchange,
    routing_key=exchangeRoutingKey,
    body=json.dumps(message),
    properties=pika.BasicProperties( delivery_mode = 2 )
  )
  return

if __name__ == '__main__':
  # read the config file which is config.ini
  configProperties = readConfig("config.ini")
  exchangeRoutingKey = "#"; #may need to be read out of config
  args = handleArguments(configProperties)
  main(args)
  #daemonize()
  #t = threading.Thread(target=main, args=[args])
  #threads.append(t)
  #t.start()
  #print(threading.currentThread().getName())
