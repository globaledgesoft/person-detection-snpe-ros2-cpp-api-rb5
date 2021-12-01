#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <map>
#include <unistd.h>
#include <algorithm>
#include <string>
#include <numeric>
#include <fstream>
#include <sys/types.h>
#include <sys/wait.h>

#include <opencv2/opencv.hpp>

//Macros for main.cpp
#define IOU_THRESHOLD 0.01
#define MOBILENET_IMG_HEIGHT 300
#define MOBILENET_IMG_WIDTH 300
#define LOOP_COUNT 150
#define OUT_FRAME_PATH "output.jpg"
#define OUTPUT_LAYER_1 "Postprocessor/BatchMultiClassNonMaxSuppression"
#define OUTPUT_LAYER_2 "add_6"
#define ROI_CONFIG_PATH "./config.json"
#define GST_CAM_PIPELINE "qtiqmmfsrc ldc=TRUE !video/x-raw, format=NV12, width=1280, height=720, framerate=30/1 ! videoconvert ! appsink"
#define GST_VIDEO_STORE_PIPELINE "appsrc ! videoconvert ! omxh264enc ! h264parse ! mp4mux ! filesink location=video.mp4"

//Macros for utility.cpp
#define BOXES_TENSOR "Postprocessor/BatchMultiClassNonMaxSuppression_boxes"
#define SCORES_TENSOR "Postprocessor/BatchMultiClassNonMaxSuppression_scores"
#define CLASSES_TENSOR "detection_classes:0"
#define PERSON_CLASS_INDEX 1
#define PROBABILITY_THRESHOLD 0.7
#define AWS_SCRIPT_PATH "aws_send.py"

cv::Rect parse_json(std::string filepath);
float calculate_iou(cv::Rect rec1, cv::Rect rec2);
float find_average(std::vector<float> &vec);
std::vector<cv::Rect> postprocess(std::map<std::string, std::vector<float>> out, float video_height, float video_width);
bool aws_iot_send(char *time_str, unsigned int total_people_count, unsigned int inframe_count);

#endif