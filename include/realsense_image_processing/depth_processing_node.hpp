//
// Created by marc on 15.03.18.
//

#ifndef PROJECT_DEPTH_PROCESSING_NODE_HPP
#define PROJECT_DEPTH_PROCESSING_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/fill_image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

cv_bridge::CvImageConstPtr cv_ptr;
static const std::string OPENCV_WINDOW = "Image window";

#endif //PROJECT_DEPTH_PROCESSING_NODE_HPP
