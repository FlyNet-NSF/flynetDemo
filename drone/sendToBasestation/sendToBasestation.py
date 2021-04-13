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
from geopy.distance import lonlat, distance
from geopy import Point
from geographiclib.geodesic import Geodesic
from distutils.util import strtobool

# constants
networks = ("verizon", "att", "sprint", "comcast")

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
  currentBattery = random.randint(50, 100)
  endless = 0

  cell_towers = []

  while currentBattery > 10:
    droneData = {}
    """
    networkConnections = []
    network_a = {'network': 'att', 'distanceM': random.randint(500, 4000) }
    network_b = {'network': 'verizon', 'distanceM': random.randint(500, 4000) }
    networkConnections.append(network_a)
    networkConnections.append(network_b)
    droneData['networkConnections'] = networkConnections
    """
    
    droneData['latitude'] = currentLat
    droneData['longitude'] = currentLon
    droneData['batterylife'] = currentBattery

    cell_towers = generateCellTowers({'latitude': currentLat, 'longitude': currentLon}, cell_towers)  # regenerate cell towers

    for tower in cell_towers:
      towerDistanceCalc = Geodesic.WGS84.Inverse(currentLat, currentLon, tower['latitude'], tower['longitude'])
      towerDistance = towerDistanceCalc['s12']
      rtt = towerDistance + random.randint(-20, 20)  # calculate RTT with some randomness
      tower['rtt'] = rtt

    droneData['celltowers'] = cell_towers

    # flight simulation
    currentLat = currentLat + .01
    currentLon = currentLon - .01

    currentBattery = currentBattery - 1

    #print(droneData)
    #droneMessage = str(droneData, 'utf-8')
    #droneMessage = droneData
    submitToBasestation(args, basechannel, droneData)

    time.sleep(5)
  
  print("Flight is complete.  Exiting")
  sys.exit()
  
def generateCellTowers(location, existing = []):
  count = 4
  location_delta = 0.01
  loc_lat = location['latitude']
  loc_long = location['longitude']
  min_long = loc_long - location_delta
  max_long = loc_long + location_delta
  min_lat = loc_lat - location_delta
  max_lat = loc_lat + location_delta

  # remove out of range cell towers
  for tower in existing:
    if tower['longitude'] < min_long or tower['longitude'] > max_long or tower['latitude'] < min_lat or tower['latitude'] > max_lat:
      existing.remove(tower)

  out = existing
  # add stations if needed
  if len(existing) < count:
    for i in range(count - len(existing)):
      longitude = min_long + random.random() * 2 * location_delta
      latitude = min_lat + random.random() * 2 * location_delta
      out.append({'id': "ct" + str(longitude) + "-" + str(latitude), 'longitude': longitude, 'latitude': latitude, 'network': random.choice(networks)})  # add in a new random ground station

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
