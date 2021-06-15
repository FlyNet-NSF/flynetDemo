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
from collections import defaultdict

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

  ground_stations = {}

  def callback(ch, method, properties, body):
    droneData = json.loads(body)
    print(" [x] Received %s" % droneData)
    droneLL = (droneData['longitude'], droneData['latitude'])
    droneBatteryLife = droneData['batterylife']

    nonlocal ground_stations
    ground_stations = generateGroundStations(droneLL, ground_stations)

    towers = droneData['celltowers']
    graph = calculateWeights(towers, ground_stations)
    rtt, prevs = shortestPath(graph, "drone")

    max_val = float('inf')
    max_element = None
    for node in rtt:
      if node.startswith("gs_"):
        max_val = rtt[node]
        max_element = node
    
    station_to_use = max_element
    links = getPath(prevs, station_to_use)

    pathList = []
    for link in links:
      # loop through each link in the network (shortest path)
      source = link[0]
      dest = link[1]

      dest_node = None
      source_neighbors = graph[source]
      for neighbor in source_neighbors:
        if neighbor[0] == dest:
          # found neighbor
          dest_node = neighbor

      if dest_node is None:
        print("Couldn't find node?")
        exit(1)

      new_dict = {
        'link': link,
        'weight': dest_node[1],
        'latency': dest_node[2][0],
        'bw': dest_node[2][1],
        'load': dest_node[2][2]
      }

      pathList.append(new_dict)

    basestationData = {}
    basestationData['net_path'] = pathList
    submitToDrone(args, dronechannel, basestationData)

    if args.state is not None:
      # save to state
      stateFile = args.state

      jsonDict = {
        "drone": droneData,
        "stations": ground_stations,
        "weights": graph,
        "basestation": basestationData
      }

      json_dump = json.dumps(jsonDict)

      with open(stateFile, "w") as file: # Use file to refer to the file object
        data = file.write(json_dump)

  basechannel.basic_consume(queue=args.basestation_queue,
                        auto_ack=True,
                        on_message_callback=callback)

  print(' [*] Waiting for messages. ')
  basechannel.start_consuming()

def normalize(parameters):
  parameters[0] = parameters[0] / 1000  # RTT
  parameters[1] = parameters[1] / 1000  # BW
  parameters[2] = parameters[2] / 100  # Load

  return parameters

def calculateWeights(towers, stations):
  # weights for calculating overall weight
  weights = [20, 30, 50]  # weights (rtt, bw, load)

  graph = defaultdict(list)
  for t_id,tower in towers.items():
    for s_id,station in stations.items():
      # SIMULATION (weight calculation)
      tower_to_gs = Geodesic.WGS84.Inverse(tower['latitude'], tower['longitude'], station['latitude'], station['longitude'])
      tower_to_gs_distance = tower_to_gs['s12']
      rtt = tower_to_gs_distance / 1000  # calculate RTT with some randomness
      bw = random.random() * 1000  # up to 1000mb link bandwidth
      load = 20  # GS load (fixed value for now)

      parameters = [rtt, bw, load]
      param_norm = normalize(parameters)
      weighted_params = [a * b for a, b in zip(weights, param_norm)]
      total_weight = sum(weighted_params)
      # END SIMULATION

      graph[t_id].append([s_id, total_weight, parameters])  # add path to graph
      graph[s_id].append([t_id, total_weight, parameters])
    
    parameters = [tower['rtt'], tower['bw'], 0]
    param_norm = normalize(parameters)
    weighted_params = [a * b for a, b in zip(weights, param_norm)]
    total_weight = sum(weighted_params)

    graph['drone'].append([t_id, total_weight, parameters])  # add drone node
    graph[t_id].append(["drone", total_weight, parameters])  # add drone node

  return graph

def shortestPath(graph, startNode):
  # get unique nodes
  nodes = list(graph.keys())
  distance = dict.fromkeys(graph, float('inf'))  # values set to inf by default
  prev = dict.fromkeys(graph, "")

  distance[startNode] = 0  # set starting node

  while len(nodes) > 0:
    max_val = float('inf')
    max_element = None

    for node_name in distance:
      if distance[node_name] <= max_val and node_name in nodes:
        max_val = distance[node_name]
        max_element = node_name
    
    node = max_element

    nodes.remove(node)
    
    neighbors = graph[node]

    # find node paths
    for next_neighbor in neighbors:
      next_label = next_neighbor[0]
      weight = distance[node] + next_neighbor[1]

      if weight < distance[next_label] and next_label in nodes:
        distance[next_label] = weight
        prev[next_label] = node

  return distance, prev

def getPath(prev, destinationNode):
  out = []
  currentNode = destinationNode

  while prev[currentNode] != "":
    out.insert(0, [prev[currentNode], currentNode])
    currentNode = prev[currentNode]

  return out

def generateGroundStations(location, existing = {}):
  count = 2
  location_delta = 0.1
  loc_lat = location[1]
  loc_long = location[0]

  min_long = loc_long - location_delta
  max_long = loc_long + location_delta
  min_lat = loc_lat - location_delta
  max_lat = loc_lat + location_delta

  # remove out of range stations
  delList = []
  for id,station in existing.items():
    if station['longitude'] < min_long or station['longitude'] > max_long or station['latitude'] < min_lat or station['latitude'] > max_lat:
      delList.append(id)

  for id in delList:
    del existing[id]

  out = existing
  # add stations if needed
  if len(existing) < count:
    for i in range(count - len(existing)):
      longitude = min_long + random.random() * 2 * location_delta
      latitude = min_lat + random.random() * 2 * location_delta
      key = "gs_" + str(longitude) + "_" + str(latitude)
      out[key] = {'longitude': longitude, 'latitude': latitude}  # add in a new random ground station

  return out


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
  parser.add_argument("-s", "--state", dest="state", default=properties['state_file'], help="File path to state json")
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
