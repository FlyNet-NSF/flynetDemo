# Tracking/Fusion Module
This module takes a frame and the output of the motion and detection modules and a class of objects that you are interested in and draws a bounding box around objects of that class that are in motion. ![Final Output](../docs/images/finaloutput.jpg)

## Setup
### For the application
1. Make sure you're in the detectionmodule directory `cd trackingmodule`.
2. Build the container for this module `docker build -t trackingmodule .`. This takes a while. 
3. Make a coconet directory on your host wherever you would like it your files to be `mkdir /coconet`. Set the location as appropriate.
3. Create the docker volume for the result of the detection module `docker volume create -d local-persist -o mountpoint=/coconet/inputdetection --name=detectioninput`. Change mountpoint as appropriate.
4. Create the docker volume for the result of the motion module `docker volume create -d local-persist -o mountpoint=/coconet/inputmotion --name=motioninput`. Change mountpoint as appropriate.
3. Create the docker volume for its input `docker volume create -d local-persist -o mountpoint=/coconet/dataset --name=coconetinput`. Change mountpoint as appropriate.
4. Create the docker volume for its output `docker volume create -d local-persist -o mountpoint=/coconet/output --name=coconetoutput`. Change mountpoint as appropriate.
4. Create the docker volume for the tracking coordinates `docker volume create -d local-persist -o mountpoint=/coconet/trackingCoordinates --name=trackingcoordinatesinput`. Change mountpoint as appropriate.

### For Tornado
#### Deploy tornado server on the host

#### Deploy tornado client on the host
1. Make sure tornado server is deployed on the host that contains the tracking/fusion module.

#### Confirm tornado client on the image source is sending images to this tornado server
1. 

## Deployment
1. Run the container `docker run --gpus all --rm -i -v coconetinput:/coconet/dataset -v coconetoutput:/coconet/output -v detectioninput:/coconet/inputdetection -v motioninput:/coconet/inputmotion -v trackingcoordinatesinput:/coconet/trackingCoordinates -t trackingmodule bash`.
2. Run the application in the container `./TrackingFusion ./dataset/ ./inputmotion/ ./inputdetection/ car ./output`