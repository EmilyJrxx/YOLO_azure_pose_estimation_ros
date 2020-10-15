# include <ros/ros.h> // ros basic

# include <message_filters/macros.h>
# include <message_filters/subscriber.h>
# include <message_filters/time_synchronizer.h>
# include <message_filters/sync_policies/approximate_time.h>
# include <sensor_msgs/image_encodings.h>
# include <sensor_msgs/Image.h>
# include <sensor_msgs/CameraInfo.h>
# include <sensor_msgs/PointCloud2.h>
# include <std_msgs/Header.h>

# include <opencv4/opencv2/core/core.hpp>
# include <opencv4/opencv2/highgui/highgui.hpp>
# include <cv_bridge/cv_bridge.h>

# include <iostream>

# include "../include/YOLO_Detection_class.h"

using namespace std;
using namespace cv;
using namespace message_filters;
using namespace sensor_msgs;
using namespace yolo;

// Topic Specification
const string rgb_topic = "/rgb/image_raw";
const string depth_topic = "/depth_to_rgb/image_raw";
const string cloud_topic = "/points2";
// YOLO Specification
const int inpHeight = 416;
const int inpWidth = 416;
const double confThreshold = 0.5;
const double nmsThreshold = 0.4;
const string modelConfiguration = "/home/xxwang/Packages/darknet/cfg/yolov3.cfg";
const string modelWeights = "/home/xxwang/Packages/darknet/yolov3.weights";
const string classesFile = "/home/xxwang/Packages/darknet/data/coco.names";
yolo::YOLODetector detector(modelConfiguration, modelWeights);
// cv bridge Specification
// Main Function
void Callback(const sensor_msgs::Image::ConstPtr & rgb, 
    const sensor_msgs::Image::ConstPtr & depth)
 // const sensor_msgs::PointCloud2::ConstPtr & cloud
{
    // Image Receiving
    // cout << "Callback: " << endl; // debug
    cv_bridge::CvImagePtr rgb_ptr;
    cv_bridge::CvImagePtr depth_ptr;
    try{
        rgb_ptr = cv_bridge::toCvCopy(rgb, image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }
    try{
        depth_ptr = cv_bridge::toCvCopy(depth, image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }
    Mat rgb_frame = rgb_ptr->image;
    Mat depth_frame = depth_ptr->image;
    // cout << "Images received" << endl; // debug
    detector.reloadImages(rgb_frame, depth_frame);
    // Detection
    vector<Mat> outs;
    // Detect
    detector.detect(outs, inpWidth, inpHeight); 
    // Postprocess
    detector.postprocess(outs, confThreshold, nmsThreshold);
    // Display
    detector.display();

    // Cloud Cropping
}
int main(int argc, char** argv)
{
    detector.LoadClassNames(classesFile);

    ros::init(argc, argv, "Azure_YOLO_detector_cropper");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgb_topic, 3);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 3);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, cloud_topic, 5);
    
    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, depth_sub);
    cout << "Subscriber Configured" << endl;

    sync.registerCallback(boost::bind(&Callback, _1, _2));

    ros::spin();

    return 0;
}