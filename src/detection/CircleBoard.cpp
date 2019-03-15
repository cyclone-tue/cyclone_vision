#include "../../include/cyclone_vision/CircleBoard.h"

using namespace std;
using namespace cv;
using namespace aruco;


Ptr<Board> CircleBoard::create(vector<Point2f> polarCoordinates, float markerLength, const Ptr<Dictionary> &dictionary, InputArray ids) {
    //convert to cartesian and add corners
    vector<vector<Point3f>> objPoints;
    for (int i = 0; i < polarCoordinates.size(); i++) {
        float r = polarCoordinates[i].x;
        float theta = polarCoordinates[i].y;

        vector<Point3f> corners;

        //clockwise order and starting with the top left corner
        Point3f corner1(r*cos(theta)-markerLength/2,r*sin(theta)+markerLength/2,0);
        Point3f corner2(r*cos(theta)+markerLength/2,r*sin(theta)+markerLength/2,0);
        Point3f corner3(r*cos(theta)+markerLength/2,r*sin(theta)-markerLength/2,0);
        Point3f corner4(r*cos(theta)-markerLength/2,r*sin(theta)-markerLength/2,0);

        corners.push_back(corner1);
        corners.push_back(corner2);
        corners.push_back(corner3);
        corners.push_back(corner4);

        objPoints.push_back(corners);
    };


    return Board::create(objPoints, dictionary, ids);
};
