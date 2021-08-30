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
import time
#import socket
import iperf3
import csv
from datetime import datetime
from argparse import ArgumentParser
from os import path
from geopy.distance import lonlat, distance
from geopy import Point
from geographiclib.geodesic import Geodesic
from distutils.util import strtobool
from collections import defaultdict
from python_hosts import Hosts, HostsEntry

def readConfig(file):
  config = configparser.ConfigParser()
  config.read(os.path.join(os.path.dirname(__file__), file))
  return config['properties']

def main(args):

  #open a csv file to keep track of scores      
  csvfile = open('./scores.csv', 'w', newline='')
  rowwriter = csv.writer(csvfile, delimiter=',')
  rowwriter.writerow(['bestTower', 'bestStation', 'bestTotal', 'nearestTower', 'nearestStation', 'nearestTotal'])
  
  credentials = pika.PlainCredentials(args.rabbituser, args.rabbitpass)
  #both currently using same credentials... 

  baseconnection = pika.BlockingConnection(pika.ConnectionParameters(host=args.basestation_host, virtual_host=args.basestation_vhost, credentials=credentials))
  basechannel = baseconnection.channel()
  basechannel.queue_declare(queue=args.basestation_queue, durable=True)
  
  droneconnection = pika.BlockingConnection(pika.ConnectionParameters(host=args.drone_host, virtual_host=args.drone_vhost, credentials=credentials))
  dronechannel = droneconnection.channel()
  dronechannel.queue_declare(queue=args.drone_queue, durable=True)

  #ground_stations = []
  #drone_flights = []
  update_no = 0
  
  def callback(ch, method, properties, body):
    #nonlocal drone_flights
    nonlocal update_no
    update_no = update_no + 1
    
    drone_flights = []
    drones = json.loads(body)
    print(" [x] Received %s" % drones)
    for droneData in drones['features']:
      droneLL = (droneData['properties']['dynamicProperties']['location']['coordinates'][0], droneData['properties']['dynamicProperties']['location']['coordinates'][1])
    
      droneBatteryLife = droneData['properties']['userProperties']['batterylife']

      #nonlocal ground_stations
      #ground_stations = modulateGroundStationsLoad(25, ground_stations) #the integer represents maximum change in load from timeframe to timeframe
      #ground_stations = generateGroundStations(droneLL, ground_stations)
      
      towers = droneData['properties']['userProperties']['celltowers']['features']
      ground_stations = droneData['properties']['userProperties']['groundstations']['features']
      #graph = calculateWeights(towers, ground_stations)
      graphGeoJSON = calculateWeightsGeoJSON(droneData, towers, ground_stations)

      maxscore = 0
      besttower = ""
      beststation = ""
      bestbw = ""
      bestipaddr = ""
      bestrtt = ""
      for this_link in graphGeoJSON:
        thisscore = 0
        if this_link['properties']['startpoint'] == "drone":
          thistowerscore = this_link['properties']['weight']
          this_tower = this_link['properties']['endpoint']
          for next_link in graphGeoJSON:
            if next_link['properties']['startpoint'] == this_tower:
              this_station = next_link['properties']['endpoint']
              thisstationscore = next_link['properties']['weight']
              totalscore = thistowerscore + thisstationscore
              if totalscore > maxscore:
                maxscore = totalscore
                besttower = this_tower
                beststation = this_station
                bestbw = min(this_link['properties']['bandwidth'], next_link['properties']['bandwidth'])
                for station in ground_stations:
                  if station['properties']['name'] == beststation:
                    bestipaddr = station['properties']['ipaddress']
                    bestrtt = station['properties']['towerRTT'][this_tower]
                for tower in towers:
                  if tower['properties']['name'] == this_tower:
                    bestrtt = bestrtt + tower['properties']['rtt']

      output_json = {
        'ipaddress': bestipaddr,
        'bandwidth': bestbw,
        'rtt': bestrtt
      }

      submitToDrone(args, dronechannel, output_json) 
      #rtt, prevs = shortestPath(graph, "drone")
      
      #max_val = float('inf')
      #max_element = None
      #for node in rtt:
      #  if node.startswith("gs_"):
      #    max_val = rtt[node]
      #    max_element = node
          
      #station_to_use = max_element
      #links = getPath(prevs, station_to_use)

      #pathList = []
      
      #for link in links:
        # loop through each link in the network (shortest path)
       # source = link[0]
       # dest = link[1]

        #dest_node = None
        #source_neighbors = graph[source]
        #for neighbor in source_neighbors:
        #  if neighbor[0] == dest:
            # found neighbor
         #   dest_node = neighbor

        #if dest_node is None:
         # print("Couldn't find node?")
          #exit(1)

        #new_dict = {
        #  'link': link,
        #  'weight': dest_node[1],
        #  'latency': dest_node[2][0],
        #  'bw': dest_node[2][1],
        #  'load': dest_node[2][2]
        #}

        #pathList.append(new_dict)
        
      #prometheusQueryURL = 'http://dynamo-broker1.exogeni.net:9090/api/v1/query?query='
      #workerinfo = getWorkerInfo()
      #for workerinf in workerinfo:
        #print("worker: " + workerinf['name'] + " address: " + workerinf['ipaddress'])
        #worker_rtt = getWorkerRTT_socket(workerinf['ipaddress'], 9100, 4)
        #print("worker: " + workerinf['name'] + " address: " + workerinf['ipaddress']
        #statusQueryURL = prometheusQueryURL + 'up{instance="' + workerinf['ipaddress'] + ':9100"}'
        #response = requests.get(statusQueryURL)
        #if response.status_code == 200:
          #connected = 1
          #nodestatusJSON = json.loads(response.content)
        #else:
          #connected = 0
          #print("response status: " + str(response.status_code))
          
        #online = 0
        #if connected and nodestatusJSON["status"] is not None and nodestatusJSON["data"] is not None:
          #if (nodestatusJSON["status"] == "success"):
            #querydata = nodestatusJSON["data"]
            #if querydata["result"] is not None:
              #result = querydata["result"]
              #if (result):
                #value = result[0]["value"]
                #status = value[1]
                #if (status == "1"):
                  #print ("node is online")
                  #online = 1
                #else:
                  #print ("node is offline")
              #else:
                #print ("no results present")
            #else:
              #print ("no results present")
          #else:
            #print ("query failed")
        #else:
          #print ("not connected or null result to query")

        #if connected and online:
          #loadQueryURL = prometheusQueryURL + 'node_load1{instance="' + workerinf['ipaddress'] + ':9100"}'
          #response = requests.get(loadQueryURL)
          #if response.status_code == 200:
            #nodeloadJSON = json.loads(response.content)
            #if (nodeloadJSON["status"] == "success") :
              #querydata = nodeloadJSON["data"]
              #result = querydata["result"]
              #if (result):
                #value = result[0]["value"]
                #load = value[1]
                #print ("load: " + load)
              #else:
                #print ("no load results given")
            #else:
              #print ("load query failed")
          #else:
            #print("response status: " + str(response.status_code))
            
          #networkQueryURL = prometheusQueryURL + 'rate(node_network_receive_bytes_total{instance="' + workerinf['ipaddress'] + ':9100"}[30s])'
          #response = requests.get(networkQueryURL)
          #if response.status_code == 200:
            #nodenetworkJSON = json.loads(response.content)
            #if (nodenetworkJSON["status"] == "success") :
              #querydata = nodenetworkJSON["data"]
              #result = querydata["result"]
              #if (result):
                #unclear what device to monitor right now
                #value = result[2]["value"]
                #load = value[1]
                #print ("bytes received in the last 30s: " + load)
              #else:
                #print ("no network results given")
            #else:
              #print ("network query failed")
          #else:
            #print("response status: " + str(response.status_code))
        
    
    
      #basestationData = {}
      #basestationData['net_path'] = pathList
      #dronePathName = pathList[0]['link'][0] + "_" + pathList[0]['link'][1] + "_link"
      #groundstationPathName = pathList[1]['link'][0] + "_" + pathList[1]['link'][1] + "_link"
      dronePathName = "drone_" + besttower + "_link"
      groundstationPathName = besttower + "_" + beststation + "_link"
      #print("dronePathName: " + dronePathName)
      #print("groundstationPathName: " + groundstationPathName)
      #{"net_path": [{"link": ["drone", "ct_-95.99335448724159_32.034022276430406"], "weight": 1.1584434516601756, "latency": 6.922172583008778, "bw": 34, "load": 0}, {"link": ["ct_-95.99335448724159_32.034022276430406", "gs_-96.12810614594274_32.07824729154417"], "weight": 21.272701295720545, "latency": 13.637583800437639, "bw": 366.66498732372645, "load": 20}]
    
      #submitToDrone(args, dronechannel, basestationData)

      groundstationCollection = {}
      groundstationCollection['type'] = "FeatureCollection"
      groundstationCollection['features'] = []
      for station in ground_stations:
        groundstationCollection['features'].append(station)

      linkCollection = {}
      linkCollection['type'] = "FeatureCollection"
      linkCollection['features'] = []

      nearestTowerName = ""
      nearestGroundStationName = ""
      for tower in towers:
        if (tower['properties']['nearestCellTower'] == "true"):
          nearestTowerName = tower['properties']['name']
          nearestGroundStationName = tower['properties']['nearestGroundStation']
      nearestDronePathName = "drone_" + nearestTowerName + "_link"
      nearestGroundstationPathName = nearestTowerName + "_" + nearestGroundStationName + "_link"
      #print("nearestDronePathName: " + nearestDronePathName)
      #print("nearestGroundstationPathName: " + nearestGroundstationPathName)
      nonlocal rowwriter
      nearestDronePathWeight = 0
      nearestGroundstationPathWeight = 0
      for thisLink in graphGeoJSON:
        #print("thisLinkName: " + thisLink['properties']['name'])
        
        if thisLink['properties']['name'] == nearestDronePathName:
          nearestDronePathWeight = thisLink['properties']['weight']
        elif thisLink['properties']['name'] == nearestGroundstationPathName:
          nearestGroundstationPathWeight = thisLink['properties']['weight']
        
        if thisLink['properties']['name'] == dronePathName or thisLink['properties']['name'] == groundstationPathName:
          thisLink['properties']['preferredLink'] = "true"
        else:
          thisLink['properties']['preferredLink'] = "false"

        if thisLink['properties']['name'] == dronePathName:
          bestDronePathWeight = thisLink['properties']['weight']
        elif thisLink['properties']['name'] == groundstationPathName:
          bestGroundstationPathWeight = thisLink['properties']['weight']
        linkCollection['features'].append(thisLink)

      nearestTotal = nearestDronePathWeight + nearestGroundstationPathWeight
      bestTotal = bestDronePathWeight + bestGroundstationPathWeight
      
      rowwriter.writerow([bestDronePathWeight, bestGroundstationPathWeight, bestTotal, nearestDronePathWeight, nearestGroundstationPathWeight, nearestTotal])
      
      #jsonDict = {
      #  "drone": droneData,
      #  "stations": ground_stations,
      #  "weights": graph,
      #  "basestation": basestationData
      #}

      droneData['properties']['userProperties']['groundstations'] = groundstationCollection
      droneData['properties']['userProperties']['networklinks'] = linkCollection
      drone_flights.append(droneData)
      
    drones = {}
    drones['type'] = "FeatureCollection"
    drones['features'] = drone_flights
    
    if args.directory is not None:
        # save to state
        state_directory = args.directory
        state_file_name = "flynet_%02d.geojson" % (update_no,)
        state_file = state_directory + "/" + state_file_name
        #json_dump = json.dumps(jsonDict)
        json_dump = json.dumps(drones)
        with open(state_file, "w") as file: # Use file to refer to the file object
          data = file.write(json_dump)

  basechannel.basic_consume(queue=args.basestation_queue,
                        auto_ack=True,
                        on_message_callback=callback)

  print(' [*] Waiting for messages. ')
  try:
    basechannel.start_consuming()
  except KeyboardInterrupt:
    basechannel.stop_consuming()
    
def normalize(parameters):
  rtt = parameters[0]
  bw = parameters[1]
  load = parameters[2]

  rtt = (100-(10*rtt))  # RTT.... low numbers are better so invert...
  if rtt < 0:
    rtt = 0
  bw = bw * 10  # BW
  #load = load / 100  # Load
  load = (100-load) #Load... because high numbers are bad, we invert it in the normalization as a precursor to applying the weights... 
  return [rtt, bw, load]

def normalize_alt(parameters):  #alternate normalization, mainly for bandwidth differences between drone-> tower and tower-> ground_stations
  rtt = parameters[0]
  bw = parameters[1]
  load = parameters[2]
  bw = bw/6
  rtt = (100-(10*rtt))  # RTT.... low numbers are better so invert...
  load = (100-load) #Load... because high numbers are bad, we invert it in the normalization as a precursor to applying the weights...
  return [rtt, bw, load]
  
def calculateWeights(towers, stations):
  # weights for calculating overall weight
  towerweights = [0, 100, 0]
  stationweights = [0, 100, 0]  # weights (rtt, bw, load)

  graph = defaultdict(list)
  
  for tower in towers:
    for station in stations:
      rtt = station['properties']['towerRTT'][tower['properties']['name']]
      #bw = random.random() * 1000  # up to 1000mb link bandwidth
      client = iperf3.Client()
      client.server_hostname = station['properties']['ipaddress']
      client.port = 5201
      client.duration = 1
      client.json_output = True
      result = client.run()
      bw = result.sent_Mbps
      del client
      load = station['properties']['load'] 

      parameters = [rtt, bw, load]
      param_norm = normalize(parameters)
      weighted_params = [a * b for a, b in zip(stationweights, param_norm)]
      total_weight = sum(weighted_params)
      # END SIMULATION

      graph[tower['properties']['name']].append([station['properties']['name'], total_weight, parameters])  # add path to graph
      graph[station['properties']['name']].append([tower['properties']['name'], total_weight, parameters])
    
    parameters = [tower['properties']['rtt'], tower['properties']['bandwidth'], 0]
    param_norm = normalize(parameters)
    weighted_params = [a * b for a, b in zip(towerweights, param_norm)]
    total_weight = sum(weighted_params)

    graph['drone'].append([tower['properties']['name'], total_weight, parameters])  # add drone node
    graph[tower['properties']['name']].append(["drone", total_weight, parameters])  # add drone node

  return graph

def calculateWeightsGeoJSON(droneData, towers, stations):
  # weights for calculating overall weight                                                                                                                 
  towerweights = [0, 100, 0]
  stationweights = [0, 100, 0]  # weights (rtt, bw, load)                                                                                                        
  graphGeoJSON = []
  
  for tower in towers:
    this_tower_ll = tower['geometry']['coordinates']
    for station in stations:
      this_station_ll = station['geometry']['coordinates']
      this_link = {}
      this_link['type'] = "Feature"
      this_link['geometry'] = {}
      this_link['geometry']['type'] = "LineString"
      this_link['geometry']['coordinates'] = []
      this_link['geometry']['coordinates'].append(this_tower_ll)
      this_link['geometry']['coordinates'].append(this_station_ll)
      this_link['properties'] = {}
      this_link['properties']['classification'] = "networkPath"
      this_link['properties']['name'] = tower['properties']['name'] + "_" + station['properties']['name'] + "_link" 
      this_link['properties']['startpoint'] = tower['properties']['name']
      this_link['properties']['endpoint'] = station['properties']['name']
      rtt = station['properties']['towerRTT'][tower['properties']['name']]
      client = iperf3.Client()
      client.server_hostname = station['properties']['ipaddress'] #change me... ideally read out of worker public.json file
      client.port = 5201
      client.duration = 1
      client.json_output = True
      result = client.run()
      bw = result.sent_Mbps
      del client
      load = station['properties']['load']
      #bw = random.random() * 1000  # up to 1000mb link bandwidth 

      parameters = [rtt, bw, load]
      param_norm = normalize_alt(parameters)
      weighted_params = [a * b for a, b in zip(stationweights, param_norm)]
      total_weight = sum(weighted_params)
      # END SIMULATION                                                                                                                                     
      this_link['properties']['weight'] = total_weight
      this_link['properties']['rtt'] = rtt
      this_link['properties']['bandwidth'] = bw
      this_link['properties']['load'] = load
      graphGeoJSON.append(this_link)
      
    this_link = {}
    this_link['type'] = "Feature"
    this_link['geometry'] = {}
    this_link['geometry']['type'] = "LineString"
    this_link['geometry']['coordinates'] = []
    this_link['geometry']['coordinates'].append(droneData['properties']['dynamicProperties']['location']['coordinates'])
    this_link['geometry']['coordinates'].append(this_tower_ll)
    parameters = [tower['properties']['rtt'], tower['properties']['bandwidth'], 0]
    param_norm = normalize(parameters)
    weighted_params = [a * b for a, b in zip(towerweights, param_norm)]
    total_weight = sum(weighted_params)
    this_link['properties'] = {}
    this_link['properties']['classification'] = "networkPath"
    this_link['properties']['name'] = "drone_" + tower['properties']['name'] + "_link"
    this_link['properties']['startpoint'] = "drone"
    this_link['properties']['endpoint'] = tower['properties']['name']
    this_link['properties']['weight'] = total_weight
    this_link['properties']['rtt'] = parameters[0]
    this_link['properties']['bandwidth'] = parameters[1]
    this_link['properties']['load'] = parameters[2]
    graphGeoJSON.append(this_link)

  return graphGeoJSON
  
  
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

def generateGroundStations(location, existing = []):
  count = 2
  location_delta = 0.1
  loc_lat = location[1]
  loc_long = location[0]

  min_long = loc_long - location_delta
  max_long = loc_long + location_delta
  min_lat = loc_lat - location_delta
  max_lat = loc_lat + location_delta

  # remove out of range stations
  for station in existing:
    if station['geometry']['coordinates'][0] < min_long or station['geometry']['coordinates'][0] > max_long or station['geometry']['coordinates'][1] < min_lat or station['geometry']['coordinates'][1] > max_lat:
      existing.remove(station)

  # add stations if needed
  out = existing
  if len(existing) < count:
    for i in range(count - len(existing)):
      longitude = min_long + random.random() * 2 * location_delta
      latitude = min_lat + random.random() * 2 * location_delta
      this_tuple = [longitude, latitude, 0]
      key = "gs_" + str(longitude) + "_" + str(latitude)
      this_station = {}
      this_station['type'] = "Feature"
      this_station['properties'] = {}
      this_station['properties']['classification'] = "groundstation"
      this_station['properties']['name'] = key
      #generate some random load... 
      this_station['properties']['load'] = random.randint(0, 100);
      this_station['geometry'] = {}
      this_station['geometry']['type'] = "Point"
      this_station['geometry']['coordinates'] = this_tuple 
      out.append(this_station)
  return out

def modulateGroundStationsLoad(maxChange, existing = []):
  for station in existing:
    station['properties']['load'] = station['properties']['load'] + random.randint(maxChange*-1, maxChange)
    if station['properties']['load'] > 100:
      station['properties']['load'] = 100
    elif station['properties']['load'] < 0:
      station['properties']['load'] = 0
  return existing

def getWorkerInfo():
  #available worker IP addresses can be found in /etc/hosts, and possibly modulated by an independent process
  current_hosts = Hosts()
  #print("hosts path: " + current_hosts.hosts_path)
  workers = []
  for entry in current_hosts.entries:
    if entry.names is not None:
      for name in entry.names:
        #print ("name: " + name)
        if 'externalWorker' in name:
          workerdict = { "name": name, "ipaddress": entry.address }
          workers.append(workerdict)
  return workers

def getWorkerRTT_tcpSocket(host, portno, timeout):
  sock_params = (host, portno)
  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.settimeout(timeout)
    sock.connect(sock_params)
    inittime = time.time()
    sock.sendall(b'1')
    respdata = sock.recv(1)
    recvtime = time.time()
    return recvtime - inittime

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
  parser.add_argument("-z", "--directory", dest="directory", default=properties['directory'], help="directory path for json output")
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
