#ifndef MARKER_VISION_DETECTMARKER_H
#define MARKER_VISION_DETECTMARKER_H

#include <iostream>
#include <opencv/cv.hpp>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/aruco.hpp>        //
#include <opencv2/highgui.hpp>      //
#include "CircleBoard.h"
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
//#include "../V_PP.h"


#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;

using namespace cv;
using namespace std;
using namespace Eigen;



class vision {
public:
    static bool detectHoop(cv_bridge::CvImagePtr cv_ptr, tf::Pose &result);
    static bool run(VectorXd& currentState, Vector3d& hoopTransVec, Matrix3d& hoopRotMat);
    static void setupVariables(float* camera_matrix, float* distortion_coefficients);
    static void cleanup();
    static void projectPointsOntoCam(vector<Point3d> cvPoints, VectorXd& currentState, vector<Point2d>& imagePoints);
    static void writeVideo(Mat frame);
    static Mat debugFrame;  // for visualization.
private:
    static bool readCameraParameters(float* camera_matrix, float* distortion_coefficients, OutputArray calCam, OutputArray calDist);
    static Matrix3d anglesToRotMatXYZ(double roll, double pitch, double yaw);
    static Matrix3d anglesToRotMatZYX(double roll, double pitch, double yaw);
};



#endif //MARKER_VISION_DETECTMARKER_H