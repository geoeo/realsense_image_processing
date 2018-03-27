//
// Created by marc on 15.03.18.
//
#include "realsense_image_processing/depth_processing_node.hpp"


boost::mutex depth_image_mutex_;           /// mutex
cv_bridge::CvImageConstPtr cv_depth_ptr;

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
    color_sub_ = nh_.subscribe("color_channel", 1, &ImageConverter::imageCallback,this);
    depth_sub_ = it_.subscribe("depth_channel", 1, &ImageConverter::depthCallback,this);
    pub_twist_cmd_ = nh_.advertise<geometry_msgs::Twist> ("motion_cmd", 1 );


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
        cv::waitKey(1); // super important otherwise image wont be displayed
    }
}

void depthCallback(const sensor_msgs::ImageConstPtr& image)
{

    try
    {
        boost::mutex::scoped_lock scoped_lock ( depth_image_mutex_, boost::try_to_lock );
        if(!scoped_lock)
            return;
        cv_depth_ptr = cv_bridge::toCvShare(image,sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat view = cv_depth_ptr->image;
    if(!view.empty()){
        const cv::Mat &image_ref = view;
        cv::imshow(OPENCV_WINDOW_DEPTH, image_ref);
        cv::waitKey(1); // super important otherwise image wont be displayed
    }

}
};

void publishCmd (ImageConverter& ic) {
    boost::mutex::scoped_lock scoped_lock ( depth_image_mutex_, boost::try_to_lock );
    if(!scoped_lock)
        return;
    ushort minImageValue = UINT16_MAX;
    ushort minLeftValue = UINT16_MAX;
    ushort minRightValue = UINT16_MAX;
    ushort minStraightValue = UINT16_MAX;
    cv::Mat img = cv_depth_ptr->image;
    for(int i = 0; i < img.rows/2; i++){
        for(int j = 0; j < img.cols; j++){
            ushort value = img.at<ushort>(i,j);

            if(value < minImageValue)
                minImageValue = value;

            // LEFT
            if(j < cvFloor(img.cols / 3) && value < minLeftValue){
                minLeftValue = value;
            }

            //RIGHT
            if(j > cvFloor(2*img.cols / 3) && value < minRightValue){
                minRightValue = value;
            }

            //STRAIGHT
            if(j >= cvFloor(img.cols / 3) && j <= cvFloor(2*img.cols / 3) && value < minStraightValue){
                minStraightValue = value;
            }
        }
    }
    cv::imshow(OPENCV_WINDOW_PROCESSING, img);
    cv::waitKey(1); // super important otherwise image wont be displayed
    geometry_msgs::Twist twist;
    twist.linear.x = 0.5;
    twist.angular.z = 0.0;
    if(minRightValue < minLeftValue)
        twist.angular.z = -0.2;
    if(minLeftValue < minRightValue)
        twist.angular.z = 0.2;
    ic.pub_twist_cmd_.publish ( twist );
}

// TODO: Look into locks when publishing motion commands which depend images
int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_processing");
    ImageConverter ic;
    cv::namedWindow(OPENCV_WINDOW_PROCESSING);
    int key = 0;

    // checking for q or Q
    while ( ros::ok() && key != 81 && key != 113) {
        ros::spinOnce();
        publishCmd(ic);
        ros::Duration(0.01).sleep();
        key = cv::waitKey(1);
    }

    cv::destroyWindow(OPENCV_WINDOW_PROCESSING);

    return 0;
}

