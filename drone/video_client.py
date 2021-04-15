import requests
import math
import cv2
import numpy as np
import time
import os
from argparse import ArgumentParser

class video_client(object):
        def __init__(self, url, outputfile):
                self.url = url
                self.outputfile = outputfile

if __name__=="__main__":
        parser = ArgumentParser(description="video_client")

        parser.add_argument("-u", "--url", metavar="URL", type=str, help="URL to send video data for processing", required=True)
        parser.add_argument("-o", "--outputfile", metavar="OUTPUT_FILE", type=str, help="Path to output video file", required=True)

        args = parser.parse_args()
        url = args.url
        outputfile = args.outputfile
        
        frameCount = 1
        peakFPS = 0
        minFPS = 10000
        averageFPS =0
        FPS = []
        startTime = time.time()
        video = cv2.VideoCapture(outputfile)
        
        uri_init = "http://" + url + "/init"
        uri = "http://" + url + "/frameProcessing"
        #force 640x480 webcam resolution
        video.set(3,640)
        video.set(4,480)
        response = requests.post(uri_init, json = {})
        
        for i in range(0,20):
                (grabbed, frame) = video.read()
            
        while True:
                frameStartTime = time.time()
                frameCount+=1
                (grabbed, frame) = video.read()
                height = np.size(frame,0)
                width = np.size(frame,1)

                #if cannot grab a frame, this program ends here.
                if not grabbed:
                        break
                #cv2.imwrite("Frame.jpg", frame)
                #print(frame.shape)
                data = {"Frame":frame.tolist()}
                r = requests.post(uri, json = data)
                currentFPS = 1.0/(time.time() - frameStartTime)
                FPS.append(currentFPS)
                print("response = {}, frame = {}, fps = {} ".format(r, frameCount, round(currentFPS, 3)))
                file2 = open("/home/drone/result.txt", "a")
                file2.write("response = {}, frame = {}, fps = {} ".format(r, frameCount, round(currentFPS, 3)))
                file2.close
                if r == "<Response [500]>":
                        break
                print("Average FPS = {}".format(round(np.mean(FPS), 3)))
                print("RunTimeInSeconds = {}".format(round(frameStartTime - startTime, 3)))

if __name__=="__main__":
        parser = ArgumentParser(description="feature_ensemble")
        
        parser.add_argument("-u", "--url", metavar="URL", type=str, help="URL to query for feature collection", required=True)
        parser.add_argument("-r", "--runs", metavar="RUNS", type=int, help="Number of runs", required=True)
        parser.add_argument("-o", "--outputfile", metavar="OUTPUT_FILE", type=str, help="Path to output geojson file", required=True)
        
        args = parser.parse_args()
        url = args.url
        runs = args.runs
        outputfile = args.outputfile
