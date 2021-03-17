# Detection Module
This module takes a frame and outputs the coordinates of a bounding box(x, y, width, height) and the class of the objects. Sample output is below
```
363.319641, 168.942062, 55.834351, 35.885056, car 
100.088768, 114.228661, 44.680878, 26.405672, person 
```
This implementation uses PJREDDIE's YOLOV3[(See original README)](darknetREADME.md) with weights that were pretrained on Imagenet

## Setup
### For the application
1. Make sure you're in the detectionmodule directory `cd detectionmodule`.
2. Build the container for this module `docker build -t detectionmodule .`. This takes a while. 
3. Make a coconet directory on your host wherever you would like it your files to be `mkdir /coconet`. Set the location as appropriate.
3. Create the docker volume for its input `docker volume create -d local-persist -o mountpoint=/coconet/dataset --name=coconetinput`. Change mountpoint as appropriate.
4. Create the docker volume for its output `docker volume create -d local-persist -o mountpoint=/coconet/output --name=coconetoutput`. Change mountpoint as appropriate.

### For Tornado
#### Deploy tornado server on the host

#### Deploy tornado client on the host
1. Make sure tornado server is deployed on the host that contains the tracking/fusion module.

#### Confirm tornado client on the image source is sending images to this tornado server
1. 

## Deployment
1. Run the container `docker run --gpus all --rm -i -v coconetinput:/coconet/dataset -v coconetoutput:/coconet/output -t detectionmodule bash`.
2. Run the application in the container `./darknet detect cfg/yolov3.cfg yolov3.weights ./dataset/`