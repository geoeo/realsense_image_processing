//
// Created by marc on 15.03.18.
//
#include "realsense_image_processing/depth_processing_node.hpp"

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
public:
ImageConverter()
        : it_(nh_)
{

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/realsense/camera/depth/image_raw", 1, &ImageConverter::depthCallback,this);


    cv::namedWindow(OPENCV_WINDOW);
}

~ImageConverter()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{

    //ROS_INFO("cv_bridge exception: %s", msg.encoding.c_str());
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::MONO16);
        //cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat view = cv_ptr->image;
    cv::imshow(OPENCV_WINDOW, view);

}
};

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{


    //ROS_INFO("cv_bridge exception: %s", msg.encoding.c_str());
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg,msg->encoding);
        //cv_ptr = cv_bridge::toCvCopy(msg,msg.encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat view = cv_ptr->image;
    cv::imshow(OPENCV_WINDOW, view);

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_processing");
    //ImageConverter ic;
    cv::namedWindow(OPENCV_WINDOW);
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/realsense/camera/depth/image_raw", 2, depthCallback);
                                                   //ros::VoidPtr(),image_transport::TransportHints("raw"));

    ros::spin();
    //ros::spinOnce();

    return 0;
}

