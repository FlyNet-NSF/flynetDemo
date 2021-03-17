# Overview 

This folder consists of drone video analytics **_dataset_** and **_algorithm_**. 

## Dataset

This folder consists of drone settings for collecting video dataset, drone trace data and video data. Also including selected data for the video analytics algorithm.

## Algorithm

This folder consists of drone video analytics algorithm and basic service requests example. Priliminary results and light weight settings are given as well. 


# Video Analytics Application Process

This document provide detailed video streaming and analytics process. Include useful code.


## Video Capturing
Video is captured by multi-drones in real world or drone simulators. The video dataset is given inside data/video_data folder.

## Video Streaming

We use [FFmpeg](https://www.ffmpeg.org/) for video streaming. FFmpeg needs to be installed on each nodes. [FFserver](https://trac.ffmpeg.org/wiki/ffserver) is optional for RTSP streaming. UDP is used in default.

Captured drone video data is stored inside drone nodes (could be in real drones compute resources, e.g., RPi, Jetson Nano; or emulated nodes), with default Mpeg-4 format. We use ffmpeg to stream such existing video file using UDP via assigned ports (*23000 in default):
```console
foo@bar:~$ ffmpeg -i sample1.mp4 -v 0 -vcodec mpeg4 -f mpegts udp://127.0.0.1:23000
```
Edge server is used to monitor the stream and store the files into chuck of video files which used to be ready for the video analytics (deafult 10 seconds):
```console
foo@bar:~$ ffmpeg -i udp://127.0.0.1:23000 -c copy -flags +global_header -f segment -segment_time 10 -segment_format_options movflags=+faststart -reset_timestamps 1 test%d.mp4
```


## Video Analytics

Deatiled steps on video analytics is given in folder experiments/

