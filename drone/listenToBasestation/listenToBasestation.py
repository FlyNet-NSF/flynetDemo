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
  droneconnection = pika.BlockingConnection(pika.ConnectionParameters(host=args.drone_host, virtual_host=args.drone_vhost, credentials=credentials))
  dronechannel = droneconnection.channel()
  dronechannel.queue_declare(queue=args.drone_queue, durable=True)
  #outgoingDevice = "eth0"
  
  def callback(ch, method, properties, body):
    basestationCommand = json.loads(body)
    print(" [x] Received %s" % basestationCommand)
    #addedLatency = basestationCommand['latency']
    #rateLimit = basestationCommand['rate']
    #network = basestationCommand['network']
    #networkModification = "sudo tc qdisc replace dev " + outgoingDevice + " root netem delay " + str(addedLatency) + "ms rate " + str(rateLimit) + "Mbit"
    #print(networkModification)
    #os.system(networkModification)
    path = basestationCommand['network']
    print("Using path " + str(path))

    #now send video to somewhere

  dronechannel.basic_consume(queue=args.drone_queue,
                        auto_ack=True,
                        on_message_callback=callback)

  print(' [*] Waiting for messages. ')
  dronechannel.start_consuming()

def handleArguments(properties):
  parser = ArgumentParser()
  parser.add_argument("-u", "--rabbituser", dest="rabbituser", default=properties['rabbituser'],
                      type=str, help="The username for RabbitMQ.  Default is in the config file.")
  parser.add_argument("-p", "--rabbitpass", dest="rabbitpass", default=properties['rabbitpass'],
                      type=str, help="The password for RabbitMQ.  Default is in the config file.")
  parser.add_argument("-d", "--drone-host", dest="drone_host", default=properties['drone_host'],
                      type=str, help="The host/IP address for the drone RabbitMQ.  Default is in the config file.")
  parser.add_argument("-v", "--drone-vhost", dest="drone_vhost", default=properties['drone_vhost'],
                      type=str, help="The virtual host for the drone RabbitMQ.  Default is in the config file.")
  parser.add_argument("-q", "--drone-queue", dest="drone_queue", default=properties['drone_queue'],
                          type=str, help="The drone RabbitMQ queue name.  Default is in the config file.")
  parser.add_argument("-e", "--drone-exchange", dest="drone_exchange", default=properties['drone_exchange'],
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
