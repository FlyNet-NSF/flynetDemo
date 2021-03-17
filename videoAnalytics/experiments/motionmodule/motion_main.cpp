/****************************************************************************************
* Author: Ke Gao & Hadi Ali Akbarpour & Kannappan Palaniappan
*
* Copyright (C) 2019. K. Gao, H. Ali Akbarpour, K. Palaniappan
*                     and Curators of the University of Missouri, a public corporation.
*                     All Rights Reserved.
*
* Created by
* Ke Gao & Hadi Ali Akbarpour & Kannappan Palaniappan
* Department of Electrical Engineering and Computer Science (EECS)
* University of Missouri-Columbia
*
*
* For more information, contact:
*     Ke Gao
*     226 Naka Hall (EBW)
*     University of Missouri-Columbia
*     Columbia, MO 65211
*     kegao@mail.missouri.edu
*
* or
*     Dr. H. Ali Akbarpour
*     222A Naka Hall (EBW)
*     University of Missouri-Columbia
*     Columbia, MO 65211
*     aliakbarpourh@missouri.edu
*
* or
*     Dr. K. Palaniappan
*     205 Naka Hall (EBW)
*     University of Missouri-Columbia
*     Columbia, MO 65211
*     palaniappank@missouri.edu
*
****************************************************************************************/



#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <time.h>
#include <string>
#include <algorithm>
#include <unistd.h>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include "flux.h"


//-- set to show PFlux detection
#define FLAG_VIS    0
//-- set to save PFlux detections (images)
#define FLAG_SAVE_PFLUX    1
//-- size of the temporal buffer in flux tensor computation
#define BUFFER_SIZE     5


using namespace std;


int main(int argc, char **argv) {

    ///-- Load input image sequence
    ///
    std::string img_path, pflux_outpath;
    if (argc < 3) {
        std::cout << "USAGE: ./MotionModule <image_sequence_path> <out_path>" << std::endl;
        return EXIT_FAILURE;
    }
    else {
        img_path = argv[1];
        pflux_outpath = argv[2];
    }

    //-- instantiate an object of "FluxTensor" class
    FluxTensor fluxTensor;
    std::string debug_outpath = img_path + std::string("/debug/");
    std::string flux_outpath = img_path + std::string("/Flux/");
    if (FLAG_SAVE_PFLUX) {
        createFolderOrDie(pflux_outpath);
        // createFolderOrDie(flux_outpath);
    }
    // uint elapsed_t = 0;
    // myClock clock;
    cv::Mat img;
    cv::Mat img_gray;
    NVTX_PUSH("main", 0);
    std::cout << "------------------------------------------" << std::endl;
    std::cout << "-------------- Motion Module -------------" << std::endl;
    std::cout << "------------------------------------------" << std::endl << std::endl;

    try {
        cudaDeviceInit(argc, (const char **)argv);
        if (printfNPPinfo(argc, argv) == false)
        {
            cudaDeviceReset();
            exit(EXIT_SUCCESS);
        }


        int num_processed_frames = 0;
        while (true) {
            //-- list all the files in the directory
            DIR *dir_img;
            struct dirent *ent_dir_img;
            std::vector<std::string> img_list;
            if ((dir_img = opendir(img_path.c_str())) == NULL) {
                usleep(5000);
            }
            else {
                while ((ent_dir_img = readdir(dir_img)) != NULL) {
                    // printf ("%s\n", ent->d_name);
                    std::string img_name_temp = ent_dir_img->d_name;
                    std::string first_char = img_name_temp.substr(0,1);
                    if (first_char != ".") { // remove hidden files
                        img_list.push_back(img_name_temp);
                    }
                }
                closedir(dir_img);
                if (img_list.size() > 0) { // if there is at least one image in the directory
                    std::sort(img_list.begin(), img_list.end()); // sort the image file list
                    //-- define an initial bounding box
                    std::string img_path_name = img_path + "/" + img_list[0];
                    std::cout << img_path_name << std::endl;
                    img = cv::imread(img_path_name, CV_LOAD_IMAGE_COLOR);
                    if (!img.data) { // check if the 1st frame can be loaded
                        std::cout << "ERROR: image cannot be loaded!!!" << std::endl;
                        continue;
                    }
                    num_processed_frames++;
                    // std::cout << "Frame: " << num_processed_frames << std::endl;
                    cv::cvtColor(img, img_gray, CV_BGR2GRAY);

                    ///-- Initialize FluxTensor and tracker in the first frame
                    ///
                    static bool first = true;
                    if (first) {
                        first = false;
                        //-- initialize the 'FluxTensor' for the first time
                        BG_model_pars bg_model_pars;
                        fluxTensor.Init(img.cols, img.rows, BUFFER_SIZE, &fluxFilter, bg_model_pars, 1, 0, 0, debug_outpath);
                    }
                    // std::cout << "Initialization finished." << std::endl;

                    ///-- Compute PFlux result
                    ///
                    cv::Mat flux_raw = cv::Mat::zeros(img.size(), CV_8UC1);
                    cv::Mat flux = cv::Mat::zeros(img.size(), CV_8UC1);
                    cv::Mat pers_raw = cv::Mat::zeros(img.size(), CV_8UC1);
                    fluxTensor.addFrame_Mat(img_gray); // pass a frame to FluxTensor to process
                    flux_raw = fluxTensor.getFluxTraceNorm_Mat(); // get normalized trace of FluxTensor result
                    pers_raw = fluxTensor.getPersistent_Mat(); // get detected persistence change
                    cv::dilate(flux_raw, flux_raw, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
                    cv::dilate(pers_raw, pers_raw, cv::Mat(), cv::Point(-1, -1), 5, 1, 1);
                    flux_raw = flux_raw * 6;
                    pers_raw = pers_raw * 6;
                    cv::Mat pflux_3chs = cv::Mat::zeros(img.size(), CV_8UC3);
                    std::vector<cv::Mat> rgb(3);
                    cv::split(pflux_3chs, rgb);
                    rgb[0] = flux_raw.clone();
                    rgb[2] = pers_raw.clone();
                    cv::merge(rgb, pflux_3chs);
                    if (FLAG_VIS) {
                        cv::imshow("PFlux", pflux_3chs);
                    }
                    if (FLAG_SAVE_PFLUX) {
                        cv::imwrite(pflux_outpath+"/"+img_list[0]+".jpg", pflux_3chs);
                        // cv::imwrite(flux_outpath+img_idx+"_flux.png", flux_raw);
                    }
                    //-- delete the image in the directory
                    remove((img_path+img_list[0]).c_str());
                }
                //-- if no valid image files
            }
        }
        cudaDeviceReset();
    }


    ///-- Error checking
    ///
    catch (npp::Exception &rException)
    {
        std::cerr << "Program error! The following exception occurred: \n";
        std::cerr << rException.toString() << std::endl;
        std::cerr << "Aborting." << std::endl;
        cudaDeviceReset();
        exit(EXIT_FAILURE);
    }
    catch (...)
    {
        std::cerr << "Program error! An unknown type of exception occurred. \n";
        std::cerr << "Aborting." << std::endl;
        cudaDeviceReset();
        exit(EXIT_FAILURE);
        return -1;
    }
    NVTX_POP;
    return 0;
}
