#include "../include/cyclone_vision/vision.h"
#include "../include/cyclone_vision/DetectMarker.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

image_transport::Publisher debugImage;
ros::Publisher debugImageCompressed;
ros::Publisher hoopPose;

int main(int argc, char **argv){
    ros::init(argc, argv, "vision");

    ros::NodeHandle n;

    image_transport::ImageTransport it(n);

    image_transport::Subscriber imageSub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
    debugImage = it.advertise("debug_image", 1);
    hoopPose = n.advertise<geometry_msgs::PoseStamped>("hoop_pos", 1000);

    vector<float> camera_matrix, dist_matrix; //Camera matrix parameters.
    if(n.getParam("camera_matrix", camera_matrix)){
        //ROS_INFO("Got config: %s", camera_matrix.c_str());
    }else{
        ROS_ERROR("Failed to get param 'camera_matrix'");
    }
    if(!n.getParam("dist_matrix", dist_matrix)){
        ROS_ERROR("Failed to get param 'dist_matrix'");
    }

    map<string, float> camera_offset, camera_rotation;

    n.getParam("camera_transform/offset", camera_offset);
    n.getParam("camera_transform/rotation", camera_rotation);

    tf::TransformBroadcaster br;
    tf::Transform cameraTransform;
    cameraTransform.setOrigin(tf::Vector3(camera_offset["x"], camera_offset["y"], camera_offset["z"]));
    tf::Quaternion rotation;
    rotation.setRPY(camera_rotation["roll"], camera_rotation["pitch"], camera_rotation["yaw"]);
    cameraTransform.setRotation(rotation);

    vision::setupVariables(camera_matrix.data(), dist_matrix.data());

    ros::Rate updateRate(30);
    while(ros::ok()){
        br.sendTransform(tf::StampedTransform(cameraTransform, ros::Time::now(), "base_link", "camera"));

        ros::spinOnce();
        updateRate.sleep();
    }


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
        ros::Time now = ros::Time::now();
        tf::StampedTransform transform(pose, now, "camera", "hoop");
        tf::Stamped<tf::Pose> stampedPose;
        stampedPose.setData(pose);
        stampedPose.frame_id_ = "camera";
        stampedPose.stamp_ = now;

        br.sendTransform(transform);

        geometry_msgs::PoseStamped poseMsg;
        tf::poseStampedTFToMsg(stampedPose, poseMsg);
        hoopPose.publish(poseMsg);
    }

    //cv::imshow("test", cv_ptr->image);
    //cv::waitKey(3);

    debugImage.publish(cv_ptr->toImageMsg());
    //Compress image for publication over network.
}
