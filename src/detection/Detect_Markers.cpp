#include "../../include/cyclone_vision/CircleBoard.h"
#include "../../include/cyclone_vision/DetectMarker.h"
#include <ctime>
#include <ros/ros.h>
#include <tgmath.h>
#include <math.h>

Mat calCam;
Mat calDist;
Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);

namespace {
    //used for detectboard
    const vector<Point2f> polarCoordinates{
            Point2f((1.057-0.141)/2,M_PI_2),
            Point2f((1.036-0.141)/2,0),
            Point2f((1.057-0.141)/2,-M_PI_2),
            Point2f((1.036-0.141)/2,M_PI)
    };
    const vector<int> boardIds{26,27,25,24};
    const float markerLength = 0.141; // Marker side length in meters
    const Ptr<Board> board = CircleBoard::create(polarCoordinates, markerLength, dictionary, boardIds);
}

bool vision::readCameraParameters(float* camera_matrix, float* distortion_coefficients, OutputArray calCam, OutputArray calDist){ // not OutputArray

    try {
        cv::Mat localCam(Size(3,3), CV_32FC1, camera_matrix);
        cv::Mat localDist(Size(5,1),CV_32FC1, distortion_coefficients);
        localCam.copyTo(calCam);
        localDist.copyTo(calDist);
        return true;
    } catch (Exception e){
        ROS_ERROR("Could not load camera configuration parameters");
        return false;
    }

}

void vision::setupVariables(float* camera_matrix, float* distortion_coefficients){
    vision::readCameraParameters(camera_matrix, distortion_coefficients, calCam, calDist);
}

bool vision::detectHoop(cv_bridge::CvImagePtr cv_ptr, tf::Pose &result) {
    vector<int> ids;
    vector<vector<Point2f> > corners;
    aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);

    if (!ids.empty()){
        aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        //vector<Vec3d> rvec, tvec;
        Vec3d rvec, tvec;

        int valid = aruco::estimatePoseBoard(corners, ids, board, calCam, calDist, rvec, tvec);

        if(valid > 0){
            ROS_INFO("Found board!");
            aruco::drawAxis(cv_ptr->image, calCam, calDist, rvec, tvec, 0.1);

            result.setOrigin(tf::Vector3(tvec[2], -tvec[0], -tvec[1]));
            float theta = (float)(sqrt(rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]));
            tf::Quaternion q, flip;
            tf::Vector3 axis(rvec[2], -rvec[0], -rvec[1]);
            q.setRotation(axis, theta);
            flip.setRPY(0, M_PI, 0);
            result.setRotation(q * flip);
            return true;
        }
        /*aruco::estimatePoseSingleMarkers(corners, 0.141, calCam, calDist, rvec, tvec);

        if(!tvec.empty() ){
            for(int i = 0; i < ids.size(); i++){
                if(ids.at(i) == 26){
                    ROS_INFO("Found board");
                    ROS_INFO_STREAM("rvec: " << rvec.at(i) << " tvec " << tvec.at(i));
                    //ROS_INFO_STREAM("camMat: " << calCam << " distMat " << calDist);
                    aruco::drawAxis(cv_ptr->image, calCam, calDist, rvec.at(i), tvec.at(i), 0.10);
                    result.setOrigin(tf::Vector3(tvec.at(i)[2], -tvec.at(i)[0], -tvec.at(i)[1]));
                    Mat rotMat;
                    //Rodrigues(rvec.at(i), rotMat);
                    float theta = (float)(sqrt(rvec.at(i)[0]*rvec.at(i)[0] + rvec.at(i)[1]*rvec.at(i)[1] + rvec.at(i)[2]*rvec.at(i)[2]));
                    tf::Quaternion q, flip;
                    tf::Vector3 axis(rvec.at(i)[2], -rvec.at(i)[0], -rvec.at(i)[1]);
                    q.setRotation(axis, theta); //Set the rotation to theta radians around the specified axis
                    flip.setRPY(0, M_PI, 0);
                    result.setRotation(q * flip);
                    return true;
                }

            }

        }*/
    }
    return false;

}

