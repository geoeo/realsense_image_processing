//
// Created by marc on 15.03.18.
//
#include "realsense_image_processing/image_processing_node.hpp"


// https://github.com/ros-perception/image_pipeline/blob/indigo/image_view/src/nodes/image_view.cpp
// http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
class ImageConverter
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber depth_sub_;
    image_transport::Subscriber color_sub_;


public:
    ros::Publisher pub_twist_cmd_;
    boost::mutex depth_image_mutex_;           /// mutex
    boost::mutex color_image_mutex_;           /// mutex
    cv_bridge::CvImageConstPtr cv_depth_ptr;
    cv_bridge::CvImageConstPtr cv_color_ptr;

ImageConverter()
        : it_(nh_)
{

    // Subscrive to input video feed and publish output video feed
    color_sub_ = it_.subscribe("color_channel", 1, &ImageConverter::imageCallback,this);
    depth_sub_ = it_.subscribe("depth_channel", 1, &ImageConverter::depthCallback,this);

    cv::namedWindow(OPENCV_WINDOW_COLOR);
    cv::namedWindow(OPENCV_WINDOW_DEPTH);
}

~ImageConverter()
{
    cv::destroyWindow(OPENCV_WINDOW_COLOR);
    cv::destroyWindow(OPENCV_WINDOW_DEPTH);
}

void imageCallback(const sensor_msgs::ImageConstPtr &image) {

    //cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        boost::mutex::scoped_lock scoped_lock ( color_image_mutex_, boost::try_to_lock );
        if(!scoped_lock)
            return;
        cv_color_ptr = cv_bridge::toCvShare(image,sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat view = cv_color_ptr->image;
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

//http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html
//https://docs.opencv.org/3.2.0/d4/d86/group__imgproc__filter.html#gad78703e4c8fe703d479c1860d76429e6
void calcGradient(ImageConverter &ic) {
    boost::mutex::scoped_lock scoped_lock ( ic.color_image_mutex_, boost::try_to_lock );
    if(!scoped_lock)
        return;

    cv::Mat img = ic.cv_color_ptr->image;
    cv::Mat gradient;
    cv::Mat gradient_blur;
    cv::Mat gradient_x;
    cv::Mat gradient_y;
    cv::Laplacian(img,gradient,-1,5);

    //cv::GaussianBlur(gradient,gradient_blur,cv::Size(5,5),0);
    //cv::blur(gradient,gradient_blur,cv::Size(5,5));
    cv::medianBlur(gradient,gradient_blur,5);
    //cv::bilateralFilter(gradient,gradient_blur,9,75,75);
    cv::Sobel(img,gradient_x,-1,1,0,1);
    cv::Sobel(img,gradient_y,-1,0,1,1);

    cv::imshow(OPENCV_WINDOW_PROCESSING+" Sobel X", gradient_x);
    cv::waitKey(1); // super important otherwise image wont be displayed
    cv::imshow(OPENCV_WINDOW_PROCESSING+" Sobel Y", gradient_y);
    cv::waitKey(1); // super important otherwise image wont be displayed
    cv::imshow(OPENCV_WINDOW_PROCESSING+"Laplace", gradient);
    cv::waitKey(1); // super important otherwise image wont be displayed
    cv::imshow(OPENCV_WINDOW_PROCESSING+"Laplace Blur", gradient_blur);
    cv::waitKey(1); // super important otherwise image wont be displayed
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_processing");
    ImageConverter ic;
    //cv::namedWindow(OPENCV_WINDOW_PROCESSING);
    int key = 0;

    // checking for q or Q
    while ( ros::ok() && key != 81 && key != 113) {
        ros::spinOnce();
        calcGradient(ic);
        ros::Duration(0.01).sleep();
        key = cv::waitKey(1);
    }

    cv::destroyWindow(OPENCV_WINDOW_PROCESSING);

    return 0;
}

