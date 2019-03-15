#include "../../include/cyclone_vision/CircleBoard.h"
#include "../../include/cyclone_vision/DetectMarker.h"
#include <ctime>
#include <ros/ros.h>
#include <tgmath.h>
#include <math.h>
//#include "../logging.h"
//#include "spdlog/fmt/ostr.h"


VideoCapture cap;
//Videowriter debugStream;
Mat calCam;
Mat calDist;
Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);
VideoWriter debugWriter;

Mat vision::debugFrame;// = Mat::zeros(cv::Size(1,49), CV_64FC1);

//VectorXd coef_vec2(6);




namespace {
    const char* keys =
            "{cal |    | File to load calibration data from}"
            "{cam | 0  | camera input to use}";


    const Vec3d hoop_offset = Vec3d(0,0.37,0);//Offset variable in world space from center of marker to center of hoop.

    //used for detectboard
    const vector<Point2f> polarCoordinates{
            Point2f((1.057-0.141)/2,M_PI_2),
            Point2f((1.036-0.141)/2,0),
            Point2f((1.057-0.141)/2,-M_PI_2),
            Point2f((1.036-0.141)/2,M_PI)
    };
    const vector<int> boardIds{26,27,25,24};
    const float markerLength = 0.141; // Marker side length in meters
    //Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);
    const Ptr<Board> board = CircleBoard::create(polarCoordinates, markerLength, dictionary, boardIds);

}






bool vision::readCameraParameters(float* camera_matrix, float* distortion_coefficients, OutputArray calCam, OutputArray calDist){ // not OutputArray

    /*FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened()){
        fs.release();
        ROS_ERROR("Could not open configuration file: %s", filename.c_str());
        //v_logger->error("Could not open file: {}", filename);
        return false;(float* camer, OutputArray cameraMat, OutputArray distCoefficients)
    }

    Mat localCamera, localDist;
    fs["camera_matrix"] >> localCamera;
    fs["distortion_coefficients"] >> localDist;

    localCamera.copyTo(cameraMat);
    localDist.copyTo(distCoefficients);

    fs.release();
    return true;*/
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


bool vision::run(VectorXd& currentState, Vector3d& hoopTransVec, Matrix3d& hoopRotMat) {        // visualize should be removed in future version.

    bool foundMarker = false;
    Mat image, imageCopy;
    cap >> image;
    //cap.read(image);
    image.copyTo(imageCopy);
    Vector3d translation;
    Matrix3d rotation;

    //resize(imageCopy, imageCopy, Size(64, 36), 0, 0, INTER_CUBIC);

    vector<int> ids;
    vector<vector<Point2f> > corners;
    aruco::detectMarkers(image, dictionary, corners, ids);

    //At least one marker detected
    if (!ids.empty()) {

        aruco::drawDetectedMarkers(imageCopy, corners, ids);
        Vec3d rvec, tvec;

        // tvec is given in the following frame: (I call this frame the oddCam frame, because it is strange)
        // positive z is in the direction of sight.
        // the origin of the frame is the camera.
        // tvec points to the center of the hoop.
        // positive x direction is to the right(relative to sight).
        // positive y direction is down(relative to sight).
        // this frame is thus right handed.

        int valid = aruco::estimatePoseBoard(corners, ids, board, calCam, calDist, rvec, tvec);  // get rvec and tvec in camera frame.

        //The returned transformation is the one that transforms points from the board coordinate system to the camera coordinate system.


        // if at least one board marker detected
        if(valid > 0) {
            ROS_DEBUG("Board found!");
            //v_logger->debug("board found");

            foundMarker = true;

            aruco::drawAxis(imageCopy, calCam, calDist, rvec, tvec, 0.1);


            double temp_val = rvec(2);
            rvec(2) = rvec(1);
            rvec(1) = rvec(0);
            rvec(0) = temp_val;


            Mat rotMat;
            Rodrigues(rvec, rotMat);
            Mat invertBoard = (Mat_<double>(3,3) << -1,0,0, 0,1,0, 0,0,-1);
            rotMat = rotMat*invertBoard;
            //rotMat = rotMat.t();



            Vector3d hoopPos_oddCam, hoopPos_camera;        // calculate the hoop position in camera frame
            cv2eigen(tvec, hoopPos_oddCam);
            hoopPos_camera(0) = hoopPos_oddCam(2);
            hoopPos_camera(1) = hoopPos_oddCam(0);
            hoopPos_camera(2) = hoopPos_oddCam(1);

            Matrix3d rot_hoopFrameToCamFrame;
            cv2eigen(rotMat, rot_hoopFrameToCamFrame);

            Vector3d camPos_world = currentState.block<3,1>(0,0);
            Matrix3d rot_camFrameToWorldFrame = anglesToRotMatXYZ(currentState(6), currentState(7), currentState(8));


            Vector3d hoopPos_world = rot_camFrameToWorldFrame*hoopPos_camera + camPos_world;            // calculate the board position in world frame.
            Matrix3d rot_hoopFrameToWorldFrame = rot_camFrameToWorldFrame*rot_hoopFrameToCamFrame;    // calculate the rotation matrix from hoop frame to world frame.

            //Mat pos = rotMat.t() * Mat(tvec);   //Calculate board position in world space

            /* seems somewhat complex, so did not remove it, but could not find a use for it....
            Mat pixelsTranslated = rotMat * pos;
            Vec3d pixels;
            pixelsTranslated.copyTo(pixels);
            tvec = pixels;
            double sy = sqrt(pow(rotMat.at<double>(0, 0), 2) + pow(rotMat.at<double>(1, 0), 2));
            bool singular = sy < 1e-6;
            double rollTemp, pitchTemp, yawTemp;
            if (!singular) {
                rollTemp = atan2(rotMat.at<double>(2, 1), rotMat.at<double>(2, 2));
                pitchTemp = atan2(-rotMat.at<double>(2, 0), sy);
                yawTemp = atan2(rotMat.at<double>(1, 0), rotMat.at<double>(0, 0));
            } else {
                rollTemp = atan2(rotMat.at<double>(2, 1), rotMat.at<double>(2, 2));
                pitchTemp = 0;
                yawTemp = atan2(rotMat.at<double>(1, 0), rotMat.at<double>(0, 0));
            }

            double yaw = -pitchTemp;
            double roll = -yawTemp;
            double pitch = M_PI - rollTemp;
            if (pitch > M_PI) {
                pitch -= 2 * M_PI;
            }

            double x = pos.at<double>(0, 0);
            double y = pos.at<double>(1, 0);
            double z = pos.at<double>(2, 0);
            */


            // here a transformation to world frame should take place. (from body frame, which is called world frame above).

            //Vector3d bodyTransVec = currentState.block<3,1>(0,0);
            //pp_logger->debug("bodyTransVec in world frame is {}", bodyTransVec);
            //pp_logger->debug("hoop position in body frame is {}", hoopPos_world);
            //hoopPos_world += bodyTransVec;
            //pp_logger->debug("final hoop position is {}", hoopPos_world);

            Matrix3d bodyFrameToWorldFrame;
            bodyFrameToWorldFrame = anglesToRotMatXYZ(currentState(6), currentState(7), currentState(8));

            hoopTransVec = hoopPos_world;
            //hoopRotMat = bodyFrameToWorldFrame*rot_hoopFrameToWorldFrame;
            hoopRotMat = rot_hoopFrameToWorldFrame;

        }
    }

    imageCopy.copyTo(vision::debugFrame);       // for visualization

    return foundMarker;
}

Matrix3d vision::anglesToRotMatXYZ(double roll, double pitch, double yaw){
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = yawAngle*pitchAngle*rollAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();
    return rotationMatrix;
}

Matrix3d vision::anglesToRotMatZYX(double roll, double pitch, double yaw){
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = rollAngle*pitchAngle*yawAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();
    return rotationMatrix;
}



/*
int main(int argc, char* argv[]){
    CommandLineParser parser = CommandLineParser(argc, argv, keys);
    parser.about("Run marker detection code.");

    if(argc < 2){
        parser.printMessage();
        return 0;
    }

    int cam = parser.get<int>("cam");
    String filename = parser.get<String>("cal");


    setupVariables(cam, filename.c_str());

    while(true){
        Mat path;
        if(runFrame(true, path)){
            for(int j = 0; j < 100; j+=10){
                cout << "Printing points from " << j << " to " << (j + 9) << endl;
                for(int i = j; i < j + 10; i++){
                    cout << "x: " << path.at<double>(0,i) << ", y: " << path.at<double>(4,i) << ", z: " << path.at<double>(8,i) << endl;
                }
            }
        }
        char key = (char) waitKey(1);
        if (key == 27) break;
    }

    cleanup();

}*/



/*
double* vision::MatrixToArray(MatrixXd m) {
    static double db_array[100][12];
//    cout << "Path is: " << endl;
//    cout <<  m  << endl;
    Map<MatrixXd>(&db_array[0][0], m.rows(), m.cols()) = m;
    return &db_array[0][0];
}
*/

void vision::setupVariables(float* camera_matrix, float* distortion_coefficients){
    //String filename = String(calibrationFile);
    //ROS_INFO("Opening camera %d", camera);
    //v_logger->info("Opening camera {}", camera);

    //cap = VideoCapture();
    //cap.open(camera);

    //if (!cap.isOpened())  // check if succeeded to connect to the camera
    //    CV_Assert("Cam open failed");

    //cap.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);        // I(Arnoud) changed here from CV_CAP_PROP_FRAME_WIDTH
    //cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
    //cap.set(CV_CAP_PROP_FPS, 15);
    //cout << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
    //cout << cap.get(cv::CAP_PROP_FRAME_WIDTH) << endl;
    //cout << cap.get(cv::CAP_PROP_FPS) << endl;

    //debugStream.open("appsrc use-damage=false ! videoconvert ! videoscale ! vp8enc ! rtpvp8pay ! udpsink host=localhost port=9999", 0, (double)30, Size(640, 480), true);

    //ROS_INFO("Reading camera parameters from config file %s", filename.c_str());
    //v_logger->info("Reading camera parameters");
    //v_logger->info("Calibration file is {}", filename);

    if(!vision::readCameraParameters(camera_matrix, distortion_coefficients, calCam, calDist)){
        //ROS_ERROR("Could not load camera calibration file: %s", filename.c_str());
        //v_logger->error("Could not load camera calibration file: {}", calibrationFile);
    }


    //namedWindow("out", WINDOW_KEEPRATIO);
    //resizeWindow("out", 300,300);

    /*Mat testFrame;
    cap >> testFrame;

    time_t now = time(0);
    tm* localTime = localtime(&now);
    int codec = VideoWriter::fourcc('H','2','6','4');
    ostringstream converter;
    converter << "Field_test-" << localTime->tm_mday << "-" << localTime->tm_mon << "-" << localTime->tm_year << "_" << localTime->tm_hour << ":" << localTime->tm_min << ":" << localTime->tm_sec << ".mp4";
    String outFilename;
    outFilename = converter.str();
    //String outFilename = "Field_test-"  + to_string(localTime->tm_mday) + "-" + to_string(localTime->tm_mon) + "-" + to_string(localTime->tm_year) + "_" + to_string(localTime->tm_hour) + ":" + to_string(localTime->tm_min) + ":" + to_string(localTime->tm_sec) + ".mp4";

    //v_logger->info("Writing to video file: {}", outFilename.c_str());
    debugWriter = VideoWriter(outFilename.c_str(), codec, 25.0, Size(testFrame.cols, testFrame.rows), true);
    if(!debugWriter.isOpened()){
        //v_logger->error("Could not open video writer!");
    }*/
}

void vision::projectPointsOntoCam(vector<Point3d> cvPoints, VectorXd& currentState, vector<Point2d>& imagePoints){

    VectorXd temp(12);
    temp << 0,0,0, 0,0,0, 0,0,0, 0,0,0;

    Matrix3d rot_worldFrameToCamFrame = anglesToRotMatZYX(-temp(6), -temp(7), -temp(8));


    Matrix3d rot_camFrameToOddcamFrame;
    rot_camFrameToOddcamFrame << 0,1,0, 0,0,1, 1,0,0;

    Matrix3d rot_worldFrameToOddcamFrame = rot_camFrameToOddcamFrame*rot_worldFrameToCamFrame;

    Mat cvrot_worldFrameToOddcamFrame;
    eigen2cv(rot_worldFrameToOddcamFrame, cvrot_worldFrameToOddcamFrame);

    vector<Point3d> points(cvPoints.size());
    for(int i = 0; i < cvPoints.size(); i++){
        Mat point = Mat(cvPoints[i], false);
        Mat newPoint = cvrot_worldFrameToOddcamFrame*point;
        newPoint.copyTo( cv::Mat(points[i], false) );
    }

    projectPoints(points, Vec3d(0, 0, 0), Vec3d(0, 0, 0), calCam, calDist, imagePoints);
    return;
}


void vision::cleanup(){
    cap.release();

    debugWriter.release();
}

void vision::writeVideo(Mat frame){
    if(debugWriter.isOpened()){
        debugWriter.write(frame);
    }
}

bool vision::detectHoop(cv_bridge::CvImagePtr cv_ptr, tf::Pose &result) {
    vector<int> ids;
    vector<vector<Point2f> > corners;
    //Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);
    //cv::Mat frame;
    //MarkerDetector mDetector;
    //image->image.copyTo(frame);
    //cv_bridge::CvImagePtr cv_ptr;
    //cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
    //ROS_INFO_STREAM(cv_ptr->image);
    //ROS_INFO_STREAM(cv_ptr->image.cols << " " << cv_ptr->image.rows << " " << cv_ptr->image.type());
    aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);

    if (!ids.empty()){
        aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        vector<Vec3d> rvec, tvec;

        //int valid = aruco::estimatePoseBoard(corners, ids, board, calCam, calDist, rvec, tvec);
        aruco::estimatePoseSingleMarkers(corners, 0.141, calCam, calDist, rvec, tvec);

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

        }
    }
    return false;

}

