#!/usr/bin/env python3
import requests
import cv2
import numpy as np
import time
import os
from argparse import ArgumentParser


if __name__ == "__main__":
    parser = ArgumentParser(description="video_client")

    parser.add_argument("-u", "--url", metavar="URL", type=str, help="URL to send video data for processing",
                        required=True)
    parser.add_argument("-i", "--input_dir", metavar="INPUT_DIR", type=str, help="Path to input video directory",
                        required=True)
    parser.add_argument("-o", "--output_dir", metavar="OUTPUT_DIR", type=str, help="Path to output directory",
                        required=True)

    args = parser.parse_args()
    url = args.url
    input_dir = args.input_dir
    output_dir = args.output_dir

    frameCount = 1
    peakFPS = 0
    minFPS = 10000
    averageFPS = 0
    FPS = []
    startTime = time.time()
    uri_init = f"http://{url}/init"
    uri = f"http://{url}/frameProcessing"

    print(f"uri_init={uri_init}")
    print(f"uri={uri}")
    processed_files = []

    while True:
        file_list = os.listdir(input_dir)
        # Retry to check if there are any new files to be processed every 60 secs.
        if file_list is None or len(file_list) < 1:
            time.sleep(60)
        
        if file_list == processed_files:
            time.sleep(60)

        for video_file in os.listdir(input_dir):
            if video_file in processed_files:
                continue
            video_file_path = f"{input_dir}/{video_file}"
            print(f"Processing file: {video_file_path}")
            file2 = open(f"{output_dir}/result.txt", "a")
            file2.write(f"Processing file: {video_file_path}\n")
            file2.close()

            video = cv2.VideoCapture(video_file_path)

            # force 640x480 webcam resolution
            video.set(3, 640)
            video.set(4, 480)
            response = requests.post(uri_init, json={})

            for i in range(0, 20):
                (grabbed, frame) = video.read()

            while True:
                frameStartTime = time.time()
                frameCount += 1
                (grabbed, frame) = video.read()

                # if cannot grab a frame, this program ends here.
                if not grabbed:
                    break

                height = np.size(frame, 0)
                width = np.size(frame, 1)

                # cv2.imwrite("Frame.jpg", frame)
                # print(frame.shape)
                data = {"Frame": frame.tolist()}
                r = requests.post(uri, json=data)
                currentFPS = 1.0 / (time.time() - frameStartTime)
                FPS.append(currentFPS)
                print("response = {}, frame = {}, fps = {} ".format(r, frameCount, round(currentFPS, 3)))
                file2 = open(f"{output_dir}/result.txt", "a")
                file2.write("response = {}, frame = {}, fps = {}\n".format(r, frameCount, round(currentFPS, 3)))
                file2.close()
                if r == "<Response [500]>":
                    break
                print("Average FPS = {}".format(round(np.mean(FPS), 3)))
                print("RunTimeInSeconds = {}".format(round(frameStartTime - startTime, 3)))

            processed_files.append(video_file)
