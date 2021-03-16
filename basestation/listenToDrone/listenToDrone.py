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

def readConfig(file):
  config = configparser.ConfigParser()
  config.read(os.path.join(os.path.dirname(__file__), file))
  return config['properties']

def main(args):
  credentials = pika.PlainCredentials(args.rabbituser, args.rabbitpass)
  #both currently using same credentials... 

  baseconnection = pika.BlockingConnection(pika.ConnectionParameters(host=args.basestation_host, virtual_host=args.basestation_vhost, credentials=credentials))
  basechannel = baseconnection.channel()
  basechannel.queue_declare(queue=args.basestation_queue, durable=True)
  
  droneconnection = pika.BlockingConnection(pika.ConnectionParameters(host=args.drone_host, virtual_host=args.drone_vhost, credentials=credentials))
  dronechannel = droneconnection.channel()
  dronechannel.queue_declare(queue=args.drone_queue, durable=True)

  def callback(ch, method, properties, body):
    droneData = json.loads(body)
    print(" [x] Received %s" % droneData)
    droneLL = (droneData['longitude'], droneData['latitude'])
    droneBatteryLife = droneData['batterylife']
    droneNetworks = droneData['networkConnections']
    maxCellUtility = 0
    bestNetwork = None
    bestTowerDistance = 0
    for droneNetwork in droneNetworks:
      #really we want to combine cell network info with processing station load info gathered from mobius
      #for short term testing we'll just make a decision based on the cell data
      thisNetworkName = droneNetwork['network']
      towerResult = getTowerInfo(thisNetworkName)
      if towerResult is not None:
        towerDistanceCalc = Geodesic.WGS84.Inverse(droneLL[1], droneLL[0], towerResult['latitude'], towerResult['longitude'])
        towerDistance = towerDistanceCalc['s12'] #https://geographiclib.sourceforge.io/html/python/code.html#geographiclib.geodesic.Geodesic.Inverse
        averageUtilization = towerResult['averageUtilization']
        thisCellUtility = getCellUtility(towerDistance, averageUtilization)
        if thisCellUtility > maxCellUtility:
          maxCellUtility = thisCellUtility
          bestNetwork = thisNetworkName
          bestTowerDistance = towerDistance
    basestationData = {}
    if bestNetwork is not None:
      basestationData['network'] = bestNetwork
      #send some arbitrary tuning params
      basestationData['latency'] = bestTowerDistance/10000
      basestationData['rate'] = random.randint(7, 10) #rate some random number in mbps assuming 10mbps was the max
      submitToDrone(args, dronechannel, basestationData)
    else:
      print("no networks to use... don't process this data")

  basechannel.basic_consume(queue=args.basestation_queue,
                        auto_ack=True,
                        on_message_callback=callback)

  print(' [*] Waiting for messages. ')
  basechannel.start_consuming()

def getTowerInfo(thisNetworkName):
  #not used at present, but if we wanted to do the simulated network overlay here on the basestation side instead of the web app, this could be useful
  networkLookup = {
        "att": { 'latitude': 32.4, 'longitude': -96.5, 'averageUtilization': random.randint(10, 90) },
        "verizon": { 'latitude': 32.3, 'longitude': -96.6, 'averageUtilization': random.randint(10, 90) }
  }
  networkParams = networkLookup.get(thisNetworkName);
  return networkParams
  
def getCellUtility(towerDistance, averageUtilization):
  #ultra simple utility function implementation... semi normalized assuming max dist between in normal comms is ~4km... 
  #presently not normalized, but doesn't matter because we don't require normalization, just take the max  
  if towerDistance == 0:
    towerDistance == .000001
  utility = (((100-averageUtilization)/100)*4000) / towerDistance
  return utility

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
  parser.add_argument("-d", "--drone-host", dest="drone_host", default=properties['drone_host'],
                      type=str, help="The host/IP address for the drone RabbitMQ.  Default is in the config file.")
  parser.add_argument("-r", "--drone-vhost", dest="drone_vhost", default=properties['drone_vhost'],
                      type=str, help="The virtual host for the drone RabbitMQ.  Default is in the config file.")
  parser.add_argument("-o", "--drone-queue", dest="drone_queue", default=properties['drone_queue'],
                          type=str, help="The drone RabbitMQ queue name.  Default is in the config file.")
  parser.add_argument("-x", "--drone-exchange", dest="drone_exchange", default=properties['drone_exchange'],
                          type=str, help="The drone RabbitMQ exchange name.  Default is in the config file.")
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

def submitToDrone(args, channel, message):
  channel.basic_publish(
    exchange=args.drone_exchange,
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
