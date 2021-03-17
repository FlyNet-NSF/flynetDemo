# Motion Module
This application takes a sequence of frames and for each frame, outputs another where objects in motion are colored blue and stationary objects are colored black. ![Motion Module Output](../docs/images/MotionModule.jpg). 

Processing a frame destroys it.

## Setup
### For the application
1. Make sure you're in this directory `cd motionmodule`.
2. Build the container for this module `docker build -t motionmodule .`. This takes a while. 
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
1. Run the container `docker run --gpus all --rm -i -v coconetinput:/coconet/dataset -v coconetoutput:/coconet/output -t motionmodule bash`.
2. Run the application in the container `./build/Motion_Module ./dataset/ ./output/`