// Input: RGB, Depth, CameraInfo, PointCloud2
// Callback Procedure: 
//      2. YOLO detection & Bounding box drawing - YOLODetection.cpp
//      3. Bounding box deprojection             - Deprojection.cpp
//      4. Cloud Cropping                        - CloudPreprocessing.cpp
//      5. Preprocessing & PPF Matching          - PPFMatching.cpp

# include <ros/ros.h> // ros basic

# include <message_filters/macros.h>
# include <message_filters/subscriber.h>
# include <message_filters/time_synchronizer.h>
# include <sensor_msgs/image_encodings.h>
# include <sensor_msgs/Image.h>
# include <sensor_msgs/CameraInfo.h>
# include <std_msgs/Header.h>

# include <opencv4/opencv2/core/core.hpp>
# include <opencv4/opencv2/highgui/highgui.hpp>
# include <cv_bridge/cv_bridge.h>

# include <iostream>


using namespace std;
using namespace cv;

const string rgb_topic = "/rgb/image_raw";

void ImageCallback(const sensor_msgs::Image::ConstPtr & image){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }

    cv::imshow("image from k4a", cv_ptr->image);
    waitKey(10);
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "Azure_image_receiver");

    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, rgb_topic, 1);
    image_sub.registerCallback(ImageCallback);

    ros::spin();
    
    return 0;
}

