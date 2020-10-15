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

# include <opencv2/core.hpp>
# include <opencv2/highgui/highgui.hpp>
# include <cv_bridge/cv_bridge.h>

# include <pcl/point_types.h>
# include <pcl/io/ply_io.h>
# include <pcl_conversions/pcl_conversions.h>
# include <pcl/filters/filter.h>

# include <iostream>
# include <fstream>

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
using namespace pcl;

const string rgb_topic = "/rgb/image_raw";
const string depth_topic = "/depth_to_rgb/image_raw";
const string cloud_topic = "/points2";
const string output_dir = "/home/emilyjr/Desktop/Azure_ROS_Recorded/";

int saved_count = 0;

void Callback(const sensor_msgs::Image::ConstPtr & rgb, 
              const sensor_msgs::Image::ConstPtr & depth,
              const sensor_msgs::PointCloud2::ConstPtr & cloud)
{
    cv_bridge::CvImagePtr rgb_ptr;
    cv_bridge::CvImagePtr depth_ptr;
    try{
        rgb_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }
    try{
        depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }

    // Wait for key to save images and clouds
    cout << "\rPress 's' to save a frame\n";
    
    imshow("rgb catch", rgb_ptr->image);
    if (waitKey(5) == 's'){
        Mat rgb_frame = rgb_ptr->image;
        Mat depth_frame = depth_ptr->image;
        PointCloud<PointXYZRGB>::Ptr pcl_cloud (new PointCloud<PointXYZRGB>);
        PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);
        
        PointCloud<PointXYZRGB>::Ptr cloud_filtered (new PointCloud<PointXYZRGB>);
        vector<int> indices;
        pcl::removeNaNFromPointCloud(*pcl_cloud, *cloud_filtered, indices);

        string rgb_name = output_dir + to_string(++saved_count) + "_rgb.jpg";
        string depth_name = output_dir + to_string(saved_count) + "_depth.exr"; // 32FC format depth image saved in OpenEXR format
        string cloud_name = output_dir + to_string(saved_count) + "_cloud.ply";
        string depth_txt_name = output_dir + to_string(saved_count) + "_depth.txt";
        imwrite(rgb_name, rgb_frame);
        imwrite(depth_name, depth_frame);
        ofstream fout (depth_txt_name);
        fout << depth_frame;
        fout.close();

        io::savePLYFileASCII(cloud_name, *cloud_filtered);

        cout << "Scene " << saved_count << " saved. " << output_dir << endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Azure_Kinect_ROS_Recorder");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgb_topic, 3);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 3);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, cloud_topic, 5);

    typedef sync_policies::ApproximateTime<Image, Image, PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync (MySyncPolicy(10), rgb_sub, depth_sub, cloud_sub);
    cout << "Subscriber Configured" << endl;

    sync.registerCallback(boost::bind(&Callback, _1, _2, _3));

    ros::spin();

    return 0;
}
