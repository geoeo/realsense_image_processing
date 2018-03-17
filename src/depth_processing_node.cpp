//
// Created by marc on 15.03.18.
//
#include "realsense_image_processing/depth_processing_node.hpp"


// https://github.com/ros-perception/image_pipeline/blob/indigo/image_view/src/nodes/image_view.cpp
// http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber depth_sub_;
    ros::Subscriber color_sub_;
public:
    ros::Publisher pub_twist_cmd_;
ImageConverter()
        : it_(nh_)
{

    // Subscrive to input video feed and publish output video feed
    color_sub_ = nh_.subscribe("/realsense/camera/color/image_raw", 1, &ImageConverter::imageCallback,this);
    depth_sub_ = it_.subscribe("/realsense/camera/depth/image_raw", 1, &ImageConverter::depthCallback,this);
    pub_twist_cmd_ = nh_.advertise<geometry_msgs::Twist> ("ackermann/cmd_vel", 1 );
    //ROS_INFO("topic: %s",image_sub_.getTopic().c_str());
    //ROS_INFO("transport: %s",image_sub_.getTransport().c_str());


    cv::namedWindow(OPENCV_WINDOW_COLOR);
    cv::namedWindow(OPENCV_WINDOW_DEPTH);
}

~ImageConverter()
{
    cv::destroyWindow(OPENCV_WINDOW_COLOR);
    cv::destroyWindow(OPENCV_WINDOW_DEPTH);
}

void imageCallback(const sensor_msgs::ImageConstPtr &image) {

    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(image,sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat view = cv_ptr->image;
    if(!view.empty()){
        const cv::Mat &image_ref = view;
        cv::imshow(OPENCV_WINDOW_COLOR, image_ref);
        cv::waitKey(1); // super important
    }
}

void depthCallback(const sensor_msgs::ImageConstPtr& image)
{

    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(image,sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat view = cv_ptr->image;
    if(!view.empty()){
        const cv::Mat &image_ref = view;
        cv::imshow(OPENCV_WINDOW_DEPTH, image_ref);
        cv::waitKey(1); // super important
    }

}
};

void publishCmd (ImageConverter& ic) {
    //boost::interprocess::scoped_lock<boost::mutex> scoped_lock ( mutex_ );
    geometry_msgs::Twist twist;
    twist.linear.x = 1.0;
    twist.angular.z = 0.2;
    ic.pub_twist_cmd_.publish ( twist );
}

// TODO: Look into locks when publishing motion commands which depend images
int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_processing");
    ImageConverter ic;

    //ros::spin();
    while ( ros::ok()) {
        ros::spinOnce();
        publishCmd(ic);
        ros::Duration(0.01).sleep();
    }

    return 0;
}

