#ifndef HEADER_FILE_H
#define HEADER_FILE_H

#include <iostream>
#include <vector>
#include <getopt.h>
#include <fstream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;
using namespace cv;

// Set desired position values 
extern vector<float> desired;

// Set PID values
extern vector<float> pid_x;
extern vector<float> pid_y;
extern vector<float> pid_w;

// Set maximum velocity values
extern float maxlinVel;
extern float maxangVel;
extern float DT;

// Function declarations
float pidController(float error, vector<float> pid, float DT, float maxVel);
vector<float> ibvs(float x, float y, float w, cv::Size sz);

#endif
