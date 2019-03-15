#include "../include/cyclone_vision/vision.h"
#include "../include/cyclone_vision/DetectMarker.h"
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_broadcaster.h>

ros::Publisher debugImage;

int main(int argc, char **argv){
    ros::init(argc, argv, "vision");

    ros::NodeHandle n;

    ros::Subscriber imageSub = n.subscribe("/usb_cam/image_raw", 1000, imageCallback);
    debugImage = n.advertise<sensor_msgs::Image>("debug_raw", 1000);

    vector<float> camera_matrix, dist_matrix;
    if(n.getParam("camera_matrix", camera_matrix)){
        //ROS_INFO("Got config: %s", camera_matrix.c_str());
    }else{
        ROS_ERROR("Failed to get param 'camera_matrix'");
    }
    if(!n.getParam("dist_matrix", dist_matrix)){
        ROS_ERROR("Failed to get param 'dist_matrix'");
    }

    vision::setupVariables(camera_matrix.data(), dist_matrix.data());

    ros::spin();
    return 0;
}

void imageCallback(const sensor_msgs::Image::ConstPtr &image) {
    //ROS_INFO("Received image.");
    static tf::TransformBroadcaster br;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    tf::Pose pose;
    if(vision::detectHoop(cv_ptr, pose)){
        br.sendTransform(tf::StampedTransform(pose, ros::Time::now(), "camera", "hoop"));
    }



    cv::imshow("test", cv_ptr->image);
    cv::waitKey(3);

    debugImage.publish(cv_ptr->toImageMsg());
}
