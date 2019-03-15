
#ifndef CIRCLE_BOARD_INCL
#define CIRCLE_BOARD_INCL 1
#include <iostream>
#include <opencv/cv.hpp>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include <opencv/cv.hpp>

using namespace std;
using namespace cv;
using namespace aruco;

class CircleBoard : public Board {
public:
    static Ptr<Board> create(vector<Point2f> polarCoordinates, float markerLength, const Ptr<Dictionary> &dictionary, InputArray ids);
};

#endif