# include <ros/ros.h>

# include <message_filters/macros.h>
# include <message_filters/subscriber.h>
# include <message_filters/time_synchronizer.h>
# include <message_filters/sync_policies/approximate_time.h>
# include <sensor_msgs/image_encodings.h>
# include <sensor_msgs/Image.h>
# include <sensor_msgs/CameraInfo.h>
# include <sensor_msgs/PointCloud2.h>
# include <std_msgs/Header.h>

# include <Eigen/Core>
# include <opencv4/opencv2/core/eigen.hpp>
# include <opencv4/opencv2/core.hpp>
# include <opencv4/opencv2/highgui/highgui.hpp>
# include <cv_bridge/cv_bridge.h>

# include <pcl/point_types.h>
# include <pcl/io/ply_io.h>
# include <pcl_conversions/pcl_conversions.h>
# include <pcl/filters/filter.h>
# include <pcl/filters/passthrough.h>
# include <pcl/sample_consensus/method_types.h>
# include <pcl/sample_consensus/model_types.h>
# include <pcl/segmentation/sac_segmentation.h>
# include <pcl/visualization/pcl_visualizer.h>

# include <iostream>
# include <fstream>

# include "../include/YOLO_Detection_class.h"
# include "../include/CloudProcessing.h"

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
using namespace pcl;

// ROS configuration
const string rgb_topic = "/rgb/image_raw";
const string depth_topic = "/depth_to_rgb/image_raw";
const string cloud_topic = "/points2";
const string info_topic = "/rgb/camera_info";
const string output_dir = "/home/xxwang/Desktop/Azure_ROS_Recorded/";
// YOLO parameters
const int inpHeight = 416;
const int inpWidth = 416;
const double confThreshold = 0.5;
const double nmsThreshold = 0.4;
const string modelConfiguration = "/home/xxwang/Packages/darknet/cfg/yolov3.cfg";
const string modelWeights = "/home/xxwang/Packages/darknet/yolov3.weights";
const string classesFile = "/home/xxwang/Packages/darknet/data/coco.names";
// const string modelFile = "/home/xxwang/ppf_matching/src/Camera/clouds/";
// const string PPFDetectorFile = "/home/xxwang/ppf_matching/src/Camera/clouds/";
// PPF parameters
double relativeSamplingStep = 0.025;
double relativeDistanceStep = 0.05;
// Camera Parameters
// Detectors Announcement
yolo::YOLODetector yolo_detector (modelConfiguration, modelWeights);
ppf::CloudProcessor ppf_processor (relativeSamplingStep, relativeDistanceStep);

string demand_object_name;

void Callback(const sensor_msgs::Image::ConstPtr & rgb, 
              const sensor_msgs::Image::ConstPtr & depth,
             const sensor_msgs::PointCloud2::ConstPtr & cloud,
              const sensor_msgs::CameraInfo::ConstPtr & info
              )
{
    int tick1 = getTickCount();
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
    int tick2 = getTickCount();

    tick1 = getTickCount();
    PointCloud<PointXYZRGB>::Ptr pcl_cloud (new PointCloud<PointXYZRGB>);
    PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud); 
    PointCloud<PointXYZRGB>::Ptr cloud_filtered (new PointCloud<PointXYZRGB>);
    vector<int> indices;
    pcl::removeNaNFromPointCloud(*pcl_cloud, *cloud_filtered, indices);
    Mat CameraInfo(3, 3, CV_32F, (void*)info->K.data());
    tick2 = getTickCount();
    cout << "Message Receiving: " 
         << (tick2 - tick1)/getTickFrequency() << "sec.\n";

    // YOLO Detection
    tick1 = getTickCount();
    yolo_detector.reloadImages(rgb_frame, depth_frame);
    vector<Mat> outs;
    yolo_detector.detect(outs, inpWidth, inpHeight);
    yolo_detector.postprocess(outs, confThreshold, nmsThreshold);
    tick2 = getTickCount();
    cout << "YOLO Detection: " 
         << (tick2 - tick1)/getTickFrequency() << "sec.\n";  

    // SAC Segmentation
    tick1 = getTickCount();
    vector<Rect> bboxes_2d_ppf; vector<int> classIds_ppf; 
    vector<int> indices_ppf; vector<float> conf_out;
    vector<string> names_out;
    yolo_detector.TransferResults(bboxes_2d_ppf, classIds_ppf, 
                                  indices_ppf, conf_out, names_out);
    // get the highest votes result
    // 1. first filter with demand names
    // 2. secondly select the highest-vote object
    vector<int> demanded_objects;
    for(int i = 0; i < names_out.size(); i++)
    {
        if (names_out[i] == demand_object_name)
        {
            demanded_objects.push_back(i);
        }
    }
    float max_conf = 0; int max_ind = 0;
    for(int i = 0; i < demanded_objects.size(); i++)
    {
        if (conf_out[demanded_objects[i]] > max_conf)
        {
            max_conf = conf_out[demanded_objects[i]];
            max_ind  = demanded_objects[i];
        }
    }
    vector<Rect>bboxes_2d_maxconf; bboxes_2d_maxconf.push_back(bboxes_2d_ppf[max_ind]);
    vector<int>classIds_maxconf; classIds_maxconf.push_back(classIds_ppf[max_ind]);
    vector<int>indices_maxconf; indices_maxconf.push_back(indices_ppf[max_ind]);

    PointCloud<PointXYZ>::Ptr scene (new PointCloud<PointXYZ>);
    copyPointCloud(*cloud_filtered, *scene);
    ppf_processor.ReloadScenes(scene, depth_frame, 
                            bboxes_2d_maxconf, classIds_maxconf, indices_maxconf);
    ppf_processor.SceneCropping(CameraInfo);
    ppf_processor.Subsampling(0.005f);
    ppf_processor.OutlierProcessing(50, 1.2);
    vector<PointCloud<PointNormal>> object_with_normals;
    object_with_normals = ppf_processor.NormalEstimation(50);

    //

}