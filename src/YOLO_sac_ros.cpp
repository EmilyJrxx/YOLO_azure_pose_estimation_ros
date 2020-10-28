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
# include <cv_bridge/cv_bridge.h>

# include <pcl/point_types.h>
# include <pcl/io/ply_io.h>
# include <pcl_conversions/pcl_conversions.h>
# include <pcl/filters/filter.h>
# include <pcl/filters/passthrough.h>
# include <pcl/filters/extract_indices.h>
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
const string pose_topic = "/object_pose";
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

string demand_object_name = "bottle";

typedef PointXYZ PointT;

Eigen::Vector3f initial_direc (0.0, 0.0, 1.0);
void posePublish (ModelCoefficients cylinder_coeff, ros::Publisher publisher)
{
    // publish object pose informs of geometry_msgs pose
    //     position: x, y, z
    //     orientation: x, y, z, w
    // quaternion & angle-axis
    geometry_msgs::Pose obj_pose;
    obj_pose.position.x = cylinder_coeff.values[0];
    obj_pose.position.y = cylinder_coeff.values[1];
    obj_pose.position.z = cylinder_coeff.values[2];
    
    Eigen::Vector3f cylinder_direc (cylinder_coeff.values[3],
                                    cylinder_coeff.values[4],
                                    cylinder_coeff.values[5]);
    Eigen::Vector3f rot_axis = initial_direc.cross(cylinder_direc);
    float rot_angle = acos(initial_direc.dot(cylinder_direc) / 
                           (initial_direc.norm() * cylinder_direc.norm()));
    obj_pose.orientation.x = sin(rot_angle / 2) * rot_axis[0];
    obj_pose.orientation.y = sin(rot_angle / 2) * rot_axis[1];
    obj_pose.orientation.z = sin(rot_angle / 2) * rot_axis[2];
    obj_pose.orientation.w = cos(rot_angle / 2);

    cout << "Publishing! " << endl;
    publisher.publish(obj_pose);
}
PointXYZ cloud_centroid (pcl::PointCloud<PointXYZ>)
{
    
}

Mat CameraInfo_mat;

void Callback(const sensor_msgs::Image::ConstPtr & rgb, 
              const sensor_msgs::Image::ConstPtr & depth,
             const sensor_msgs::PointCloud2::ConstPtr & cloud,
              const sensor_msgs::CameraInfo::ConstPtr & info,
              ros::Publisher publisher)
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

    // imwrite("rgb.jpg", rgb_frame); // debug
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
    // cout << "Camera info: " << CameraInfo << endl;
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

    PointCloud<PointXYZ>::Ptr scene (new PointCloud<PointXYZ>);
    copyPointCloud(*cloud_filtered, *scene);
    ppf_processor.ReloadScenes(scene, depth_frame, 
                            bboxes_2d_maxconf, classIds_maxconf, indices_maxconf);
    ppf_processor.SceneCropping(CameraInfo_mat);
    ppf_processor.Subsampling(0.002f);
    ppf_processor.OutlierProcessing(50, 1.2);
    vector<PointCloud<PointNormal>> objects_with_normals;
    objects_with_normals = ppf_processor.NormalEstimation(50);

    // default: only one object after cropping
    PointCloud<PointXYZ>::Ptr object (new PointCloud<PointXYZ>);
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>); 
    copyPointCloud(objects_with_normals[0], *object);
    copyPointCloud(objects_with_normals[0], *normals);

    // SAC segmentation with:
    // 1. surface normals
    // 2. filtered point clouds
    SACSegmentationFromNormals<PointT, Normal> seg;
    ExtractIndices<PointT> indi_extract;
    ExtractIndices<Normal> indi_extract_normal;

    // First remove planar parts
    seg.setOptimizeCoefficients (true);
    seg.setModelType (SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType (SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(object);
    seg.setInputNormals(normals);
    PointIndices::Ptr inliers_plane (new PointIndices);    
    ModelCoefficients::Ptr coefficients_plane (new ModelCoefficients);
    seg.segment(*inliers_plane, *coefficients_plane);

    indi_extract.setInputCloud (object);
    indi_extract.setIndices (inliers_plane);
    indi_extract.setNegative (true);
    indi_extract_normal.setInputCloud (normals);
    indi_extract_normal.setIndices (inliers_plane);
    indi_extract_normal.setNegative (true);
    PointCloud<PointXYZ>::Ptr object_plane_removed (new PointCloud<PointXYZ>);
    PointCloud<Normal>::Ptr normals_plane_removed (new PointCloud<Normal>);
    indi_extract.filter (*object_plane_removed);
    indi_extract_normal.filter (*normals_plane_removed);

    // Match with Cylinder parts
    seg.setOptimizeCoefficients (true);
    seg.setModelType (SACMODEL_CYLINDER);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType (SAC_RANSAC);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0, 0.1);
    seg.setInputCloud (object_plane_removed);
    seg.setInputNormals (normals_plane_removed);
    PointIndices::Ptr inliers_cylinder (new PointIndices);    
    ModelCoefficients::Ptr coefficients_cylinder (new ModelCoefficients);   
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    cerr << "Cylinder coefficients: " << *coefficients_cylinder << endl;

    indi_extract.setInputCloud (object_plane_removed);
    indi_extract.setIndices (inliers_cylinder);
    indi_extract.setNegative (false);
    indi_extract_normal.setInputCloud (normals_plane_removed);
    indi_extract_normal.setIndices (inliers_cylinder);
    indi_extract_normal.setNegative (false);
    PointCloud<PointXYZ>::Ptr object_cylinder (new PointCloud<PointXYZ>);
    PointCloud<Normal>::Ptr normals_cylinder (new PointCloud<Normal>);
    indi_extract.filter (*object_cylinder);

    float x = coefficients_cylinder->values[0];
    float y = coefficients_cylinder->values[1];
    float z = coefficients_cylinder->values[2];
    float ax = coefficients_cylinder->values[3];
    float ay = coefficients_cylinder->values[4];
    float az = coefficients_cylinder->values[5];

    if (object_cylinder->points.empty())
        cerr << "Can't find the cylindrical component. " << endl;
    else
    {
        // std::cerr << "PointCloud representing the cylindrical component: " 
        //           << object_cylinder->size () << " data points." << endl;
        // cerr << "Cylinder coefficients: " << coefficients_cylinder->header << endl;
        posePublish (*coefficients_cylinder, publisher);
    }
    
    // posePublish (*coefficients_cylinder);
}
int main (int argc, char** argv)
{
    // Camera Parameters
    Eigen::MatrixXd CameraIntr_tmp(3, 3);
    CameraIntr_tmp << 983.0159, 0.0, 1021.2937,
                  0.0, 982.9841, 774.7237,
                  0.0, 0.0, 1.0;
    // Mat CameraIntr;
    eigen2cv(CameraIntr_tmp, CameraInfo_mat);
    // YOLO global configuration
    yolo_detector.LoadClassNames (classesFile);
    // PPF global configuration

    // ROS init and configuration
    ros::init(argc, argv, "Azure_YOLO_SAC");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgb_topic, 3);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 3);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, cloud_topic, 5);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub (nh, info_topic, 3);
    typedef sync_policies::ApproximateTime<Image, Image, PointCloud2, CameraInfo> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, depth_sub, cloud_sub, info_sub);
    cout << "Subscriber Configured" << endl;
    
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>(pose_topic, 10);
    
    sync.registerCallback(boost::bind(&Callback, _1, _2, _3, _4, pose_pub));

    ros::spin();
    return 0;
}
