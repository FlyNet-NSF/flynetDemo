
# flynetDemo
repository for the FlyNet Demo

At present (3/9/21)

Intended for a 2 node system (drone + basestation)
Post boot scripts for each (drone.sh and basestation.sh)

Basestation sets up containerized rabbitMQ for message passing

Drone runs two programs (sendToBasestation & listenToBasestation)
Basestation runs one program (listenToDrone)

In addition to WAN, set up a 10Mbps LAN between basestation and drone
config.ini files expect basestation will be 192.168.120.1

sendToBasestation on drone periodically sends location information and network name to basestation
listenToDrone on basestation receives this, runs a simple utility function, and sends back network parameters
listenToBasestation on drone receives network parameters and emulates via the netem (tc) program
No data is sent... yet

To Do List:
--Port web app flight and network simulation functionality to sendToBasestation, or use to replace sendToBasestation for this demo
--Bring more nodes into this simulating the core and the processing hubs... Mobius on Core, Processing hubs on Chameleon
--Integrate utility function with Mobius on core where load info about processing hubs may be available
--Upload video data batches that drone can actually send out to Chameleon processing hubs
--Instantiate this whole setup with Mobius?  Currently using Flukes... portal would also work
--Update data structures in communications with necessary information
--etc. etc. etc

Current architecture plan:
![FlyNet_sim_architecture(5)](https://user-images.githubusercontent.com/30157582/110546554-a13a5280-80fc-11eb-8abc-c3d5748df624.png)
