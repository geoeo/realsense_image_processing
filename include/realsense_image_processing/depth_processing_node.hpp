//
// Created by marc on 15.03.18.
//

#ifndef PROJECT_DEPTH_PROCESSING_NODE_HPP
#define PROJECT_DEPTH_PROCESSING_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread.hpp>

static const std::string OPENCV_WINDOW_COLOR = "Color Image window";
static const std::string OPENCV_WINDOW_DEPTH = "Depth Image window";

#endif //PROJECT_DEPTH_PROCESSING_NODE_HPP
