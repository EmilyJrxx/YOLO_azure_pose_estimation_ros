# include <ros/ros.h>
# include <math.h>

# include <message_filters/macros.h>
# include <message_filters/subscriber.h>
# include <message_filters/time_synchronizer.h>
# include <message_filters/sync_policies/approximate_time.h>
# include <sensor_msgs/image_encodings.h>
# include <sensor_msgs/Image.h>
# include <sensor_msgs/CameraInfo.h>
# include <sensor_msgs/PointCloud2.h>
# include <geometry_msgs/Pose.h>
# include <std_msgs/Header.h>

# include <Eigen/Core>
# include <opencv4/opencv2/core/eigen.hpp>
# include <opencv4/opencv2/core.hpp>
# include <opencv4/opencv2/highgui/highgui.hpp>
# include <opencv4/opencv2/surface_matching/t_hash_int.hpp>
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
const string pose_topic = "/ppf_object_pose";
// const string output_dir = "/home/emilyjr/Desktop/Azure_ROS_Recorded/";
// YOLO parameters
const int inpHeight = 416;
const int inpWidth = 416;
const double confThreshold = 0.5;
const double nmsThreshold = 0.4;
const string modelConfiguration = "/home/xxwang/Packages/darknet/cfg/yolov3.cfg";
const string modelWeights = "/home/xxwang/Packages/darknet/yolov3.weights";
const string classesFile = "/home/xxwang/Packages/darknet/data/coco.names";
const string modelFile = "/home/xxwang/Workspaces/YOLO_PPF_Pose_Estimation/data/bluemoon_1/or1.2_mls15_remesh_3.0_meter.ply";
const string PPFDetectorFile = "/home/xxwang/Workspaces/YOLO_PPF_Pose_Estimation/src/detector_bluemoon_bottle.xml";
const string TrainedDetectorFile = "/home/xxwang/Workspaces/YOLO_PPF_Pose_Estimation/src/detector_bluemoon_bottle_remesh.xml";
// PPF parameters
double relativeSamplingStep = 0.025;
double relativeDistanceStep = 0.05;
// Camera Parameters
// Detectors Announcement
yolo::YOLODetector yolo_detector (modelConfiguration, modelWeights);
ppf::CloudProcessor ppf_processor (relativeSamplingStep, relativeDistanceStep);

string demand_object_name = "bottle";
Mat CameraInfo_mat;

void posePublish (Pose3D pose, ros::Publisher publisher)
{
    // publish object pose informs of geometry_msgs pose
    //     position: x, y, z
    //     orientation: x, y, z, w
    // quaternion & angle-axis
    Matx44d pose_mat = pose.pose;
    Eigen::Matrix4d pose_eigen;
    cv2eigen(pose_mat, pose_eigen);

    Eigen::Matrix3d mat_rotation = pose_eigen.block(0, 0, 3, 3);
    Eigen::Quaterniond q_rotation(mat_rotation);
    
    geometry_msgs::Pose obj_pose;
    obj_pose.position.x = pose_eigen(0, 3);
    obj_pose.position.y = pose_eigen(1, 3);
    obj_pose.position.z = pose_eigen(2, 3);
    obj_pose.orientation.x = q_rotation.x();
    obj_pose.orientation.y = q_rotation.y();
    obj_pose.orientation.z = q_rotation.z();
    obj_pose.orientation.w = q_rotation.w();

    cout << "Publishing! " << endl;
    publisher.publish(obj_pose);
}
void Callback(const sensor_msgs::Image::ConstPtr & rgb, 
              const sensor_msgs::Image::ConstPtr & depth,
             const sensor_msgs::PointCloud2::ConstPtr & cloud,
              ros::Publisher pose_pub
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
    io::savePLYFileASCII("scene_cloud.ply", *cloud_filtered); // debug
    // Mat CameraInfo(3, 3, CV_32F, (void*)info->K.data());
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

    // get the highest votes result
    // 1. first filter with demand names
    // 2. secondly select the highest-vote object
    vector<Rect> bboxes_2d_ppf; vector<int> classIds_ppf; 
    vector<int> indices_ppf; vector<float> conf_out;
    vector<string> names_out;
    yolo_detector.TransferResults(bboxes_2d_ppf, classIds_ppf, 
                                  indices_ppf, conf_out, names_out);
                                  
    vector<int> demanded_objects;
    for(int i = 0; i < names_out.size(); i++)
    {
        cout << "name: " << names_out[i] << endl; // debug
        if (names_out[i] == demand_object_name)
        {
            demanded_objects.push_back(i);
        }
    }
    float max_conf = 0; int max_ind = 0;
    for(int i = 0; i < demanded_objects.size(); i++)
    {
        cout << "confidence: " << conf_out[i] << endl; // debug
        if (conf_out[demanded_objects[i]] > max_conf)
        {
            max_conf = conf_out[demanded_objects[i]];
            max_ind  = demanded_objects[i];
        }
    }
    vector<Rect>bboxes_2d_maxconf; bboxes_2d_maxconf.push_back(bboxes_2d_ppf[max_ind]);
    vector<int>classIds_maxconf; classIds_maxconf.push_back(classIds_ppf[max_ind]);
    vector<int>indices_maxconf; indices_maxconf.push_back(indices_ppf[max_ind]);
    // single object selected

    // PPF Recognition
    tick1 = getTickCount();
    // vector<Rect> bboxes_2d_ppf; vector<int> classIds_ppf; vector<int> indices_ppf;
    // yolo_detector.TransferResults(bboxes_2d_ppf, classIds_ppf, indices_ppf);
    PointCloud<PointXYZ>::Ptr scene (new PointCloud<PointXYZ>);
    copyPointCloud(*cloud_filtered, *scene);
    ppf_processor.ReloadScenes(scene, depth_frame, bboxes_2d_maxconf, classIds_maxconf, indices_maxconf);
    ppf_processor.SceneCropping(CameraInfo_mat);
    ppf_processor.Subsampling(0.002f);
    ppf_processor.OutlierProcessing(50, 1.2);
    vector<PointCloud<PointNormal>> objects_with_normals;
    objects_with_normals = ppf_processor.NormalEstimation(50);
    vector<PointCloud<PointNormal>> edges_with_normals;
    edges_with_normals = ppf_processor.EdgeExtraction(0.03);

    tick2 = getTickCount();
    cout << "Cloud Preprocessing " 
         << (tick2 - tick1)/getTickFrequency() << "sec.\n";

    Mat object_wn_mat, edges_wn_mat;
    ppf_processor.PointCloudXYZNormalToMat(object_wn_mat, objects_with_normals[0].makeShared());
    ppf_processor.PointCloudXYZNormalToMat(edges_wn_mat, edges_with_normals[0].makeShared());
    Pose3D result_pose = ppf_processor.Matching_S2B("bottle", object_wn_mat, edges_wn_mat);
    // Pose3D result_pose = ppf_processor.Matching("bottle", object_wn_mat);
    Mat object_wn_trans = transformPCPose(object_wn_mat, result_pose.pose.inv());
    writePLY(object_wn_trans, "trans_object.ply");
    cout << "Result_pose (model -> scene): " << endl;
    result_pose.printPose();
    
    posePublish(result_pose, pose_pub);

    
}
int main(int argc, char** argv)
{
    // Camera Parameters
    Eigen::MatrixXd CameraIntr_tmp(3, 3);
    CameraIntr_tmp << 983.0159, 0.0, 1021.2937,
                  0.0, 982.9841, 774.7237,
                  0.0, 0.0, 1.0;
    // Mat CameraIntr;
    eigen2cv(CameraIntr_tmp, CameraInfo_mat);
    
    // YOLO global configuration
    yolo_detector.LoadClassNames(classesFile);
    // PPF global configuration
    Mat bottle = ppf_match_3d::loadPLYSimple(modelFile.c_str(), 1);
    ppf_processor.LoadSingleModel(bottle, "bottle");
    // ppf_processor.TrainDetector(TrainedDetectorFile, false, 0.025, 0.05);
    ppf_processor.TrainSingleDetector("bottle", 0.025, 0.05);
    // ppf_processor.LoadTrainedDetector("bottle", PPFDetectorFile);
    // ROS init and configuartion
    ros::init(argc, argv, "Azure_YOLO_PPF");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgb_topic, 3);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 3);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, cloud_topic, 3);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub (nh, info_topic, 3);
    typedef sync_policies::ApproximateTime<Image, Image, PointCloud2, CameraInfo> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, depth_sub, cloud_sub, info_sub);
    cout << "Subscriber Configured" << endl;

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>(pose_topic, 10);

    sync.registerCallback(boost::bind(&Callback, _1, _2, _3, pose_pub));

    ros::spin();
    return 0;
}