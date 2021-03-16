# flynetDemo
repository for the FlyNet Demo

At present (3/16/21)

Intended for a 5 node system (drone + basestation + core + 2x workers)
Post boot scripts for each 

Basestation sets up containerized rabbitMQ for message passing to drone

Drone runs two programs (sendToBasestation & listenToBasestation)
Basestation runs one program to talk to drone (listenToDrone)

In addition to WAN, set up a 10Mbps LAN between basestation and drone
config.ini files expect basestation will be 192.168.120.1

sendToBasestation on drone periodically sends location information and network name to basestation
listenToDrone on basestation receives this, runs a simple utility function, and sends back network parameters
listenToBasestation on drone receives network parameters and emulates via the netem (tc) program
No data is sent... yet

In addition to WAN, set	up a 1Gbps LAN between core and	workers	on 192.168.125.0/24 network... .10, .11, .12 respectively for now

Core sets up containerized rabbitMQ for message	passing	to basestation

Core installs Kubernetes

To Do List:
--Port web app flight and network simulation functionality to sendToBasestation, or use to replace sendToBasestation for this demo
--Integrate utility function with Mobius on core where load info about processing hubs may be available
--Upload video data batches that drone can actually send out to Chameleon processing hubs
--Instantiate this whole setup with Mobius?  Currently using Flukes... portal would also work
--Update data structures in communications with necessary information
--etc. etc. etc


