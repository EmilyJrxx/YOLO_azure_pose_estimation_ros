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

# include <opencv4/opencv2/core/eigen.hpp>
# include <opencv4/opencv2/core.hpp>
# include <opencv4/opencv2/highgui/highgui.hpp>
# include <cv_bridge/cv_bridge.h>

# include <pcl/point_types.h>
# include <pcl/io/ply_io.h>
# include <pcl_conversions/pcl_conversions.h>
# include <pcl/filters/filter.h>

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
const string output_dir = "/home/emilyjr/Desktop/Azure_ROS_Recorded/";
// YOLO parameters
const int inpHeight = 416;
const int inpWidth = 416;
const double confThreshold = 0.5;
const double nmsThreshold = 0.4;
const string modelConfiguration = "/home/emilyjr/Packages/darknet/cfg/yolov3.cfg";
const string modelWeights = "/home/emilyjr/Packages/darknet/yolov3.weights";
const string classesFile = "/home/emilyjr/Packages/darknet/data/coco.names";
const string modelFile = "/home/emilyjr/ppf_matching/src/Camera/clouds/";
const string PPFDetectorFile = "/home/emilyjr/ppf_matching/src/Camera/clouds/";
// PPF parameters
double relativeSamplingStep = 0.025;
double relativeDistanceStep = 0.05;
// Camera Parameters
// Detectors Announcement
yolo::YOLODetector yolo_detector (modelConfiguration, modelWeights);
ppf::CloudProcessor ppf_processor (relativeSamplingStep, relativeDistanceStep);

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
    vector<Mat> outs[1];
    yolo_detector.detect(outs, inpWidth, inpHeight);
    yolo_detector.postprocess(outs, confThreshold, nmsThreshold);
    tick2 = getTickCount();
    cout << "YOLO Detection: " 
         << (tick2 - tick1)/getTickFrequency() << "sec.\n";

    // PPF Recognition
    tick1 = getTickCount();
    vector<Rect> bboxes_2d_ppf; vector<int> classIds_ppf; vector<int> indices_ppf;
    yolo_detector.TransferResults(bboxes_2d_ppf, classIds_ppf, indices_ppf);
    PointCloud<PointXYZ>::Ptr scene (new PointCloud<PointXYZ>);
    copyPointCloud(*cloud_filtered, *scene);
    ppf_processor.ReloadScenes(scene, depth_frame, bboxes_2d_ppf, classIds_ppf, indices_ppf);
    ppf_processor.SceneCropping(CameraInfo);
    ppf_processor.Subsampling(0.005f);
    ppf_processor.OutlierProcessing(50, 1.2);
    vector<PointCloud<PointNormal>> objects_with_normals;
    objects_with_normals = ppf_processor.NormalEstimation(30);
    tick2 = getTickCount();
    cout << "Cloud Preprocessing " 
         << (tick2 - tick1)/getTickFrequency() << "sec.\n";

    Mat object_wn_mat;
    ppf_processor.PointCloudXYZNormalToMat(object_wn_mat, objects_with_normals[0].makeShared());
    Pose3D result_pose = ppf_processor.Matching("bottle", object_wn_mat);
    cout << "Result_pose (model -> scene): " << endl;
    result_pose.printPose();

}

int main(int argc, char** argv)
{
    // YOLO global configuration
    yolo_detector.LoadClassNames(classesFile);
    // PPF global configuration
    Mat bottle = ppf_match_3d::loadPLYSimple(modelFile.c_str(), 1);
    ppf_processor.LoadSingleModel(bottle, "bottle");
    ppf_processor.LoadTrainedDetector("bottle", PPFDetectorFile);
    // ROS init and configuartion
    ros::init(argc, argv, "Azure_YOLO_PPF");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgb_topic, 3);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 3);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, cloud_topic, 5);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub (nh, info_topic, 3);
    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, depth_sub, cloud_sub, info_sub);
    cout << "Subscriber Configured" << endl;

    sync.registerCallback(boost::bind(&Callback, _1, _2, _3, _4));

    ros::spin();

    return 0;
}
