#include <cmath>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv4/opencv2/opencv.hpp>

#include "preprocessing.h"
#include "blob_detection.h"

#define KEY_ESC 27
#define WIDTH 640
#define HEIGHT 400
#define PI 3.1415926
#define EULER 2.71828

#define SHOW_IMG

#define BLUE true
#define YELLOW false
#define TEAM BLUE

bool ballInterceptWithGoal(BlobDetection::Blob ball, BlobDetection::Blob goal) {
    bool leftIntercept = ball.x < goal.x;
    bool rightIntercept = (ball.x + ball.w) > (goal.x + goal.w);
    return leftIntercept && rightIntercept;
}

int main (int argc, char **argv) {

    ros::init(argc, argv, "frontcamera");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("frontcamera_topic", 10);

    ROS_INFO("Using OPENCV version %s", CV_VERSION);

    cv::VideoCapture cap("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)1280, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)1280, height=(int)1280, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink");

    if (!cap.isOpened()) {
        ROS_FATAL("Could not open camera");
        ros::shutdown();
        return 1;
    }

    BlobDetection blueDetection;
    blueDetection.set_color_range(cv::Scalar(95, 67, 101), cv::Scalar(111, 89, 114));
    blueDetection.set_area(750, 100000);

    BlobDetection ballDetection;
    ballDetection.set_color_range(cv::Scalar(0, 36, 124), cv::Scalar(12, 88, 255));
    ballDetection.set_area(15, 100000);

    BlobDetection yellowDetection;
    yellowDetection.set_color_range(cv::Scalar(0, 133, 141), cv::Scalar(24, 224, 224));
    yellowDetection.set_area(500, 100000);

    for (unsigned long frame_id = 0;;frame_id++) {
        cv::Mat frame;
        cap >> frame;

        preprocessing::resize(frame, WIDTH, HEIGHT);
        preprocessing::saturation(frame, 2.7);
        cv::rectangle(frame, cv::Point(0, 0), cv::Point(WIDTH - 1, 30), cv::Scalar(255, 255, 255), cv::FILLED);

        std::vector<BlobDetection::Blob> blueBlobs = blueDetection.detect(frame);
        std::vector<BlobDetection::Blob> ballBlobs = ballDetection.detect(frame);
        std::vector<BlobDetection::Blob> yellowBlobs = yellowDetection.detect(frame);


        BlobDetection::plot_blobs(frame, ballBlobs);
        BlobDetection::plot_blobs(frame, blueBlobs);
        BlobDetection::plot_blobs(frame, yellowBlobs);

        float ballCenterX = 9999;
        float ballCenterY = 9999;
        float goalCenterX = 9999;
        float goalCenterY = 9999;
        float goalCorner1X = 9999;
        float goalCorner1Y = 9999;
        float goalCorner2X = 9999;
        float goalCorner2Y = 9999;
        float ballAngle = 9999;
        float ballDistance = 9999;
        float goalCorner1Angle = 9999;
        float goalCorner2Angle = 9999;
        float goalAngle = 9999;
        float goalCenterDistance = 9999;

        if (!ballBlobs.empty()) {
            BlobDetection::Blob biggestBall = ballBlobs.at(0);
            ballCenterX = biggestBall.x + biggestBall.w / 2;
            ballCenterY = biggestBall.y + biggestBall.h / 2;
            float ballFromViewX = ballCenterX - WIDTH / 2;
            float ballFromViewY = HEIGHT - ballCenterY;
            float angle = atan(ballFromViewY / ballFromViewX) * 180 / PI;
            float ballDistancePixels = std::sqrt(std::pow(ballFromViewX, 2) + std::pow(ballFromViewY, 2));

            if (angle < 0) angle = (90 + angle) * -1;
            else angle = 90 - angle;

            double distance = 9 * std::pow(10, -11) * std::pow(ballDistancePixels, 4.7);

            ballDistance = distance;
            ballAngle = angle;

            ROS_INFO("angle: %f, distance: %f", angle, distance);
            cv::line(frame, cv::Point(WIDTH / 2, HEIGHT - 1), cv::Point(ballCenterX, HEIGHT - 1), cv::Scalar(0, 255, 255), 2);
            cv::line(frame, cv::Point(ballCenterX, HEIGHT - 1), cv::Point(ballCenterX, ballCenterY), cv::Scalar(0, 255, 255), 2);
            cv::line(frame, cv::Point(ballCenterX, ballCenterY), cv::Point(WIDTH / 2, HEIGHT - 1), cv::Scalar(0, 255, 255), 2);
        }

        std::vector<BlobDetection::Blob> enemyTeamGoalBlobs;
        if (TEAM == BLUE) enemyTeamGoalBlobs = yellowBlobs;
        if (TEAM == YELLOW) enemyTeamGoalBlobs = blueBlobs;

        if (!enemyTeamGoalBlobs.empty()) {
            BlobDetection::Blob biggestBlob = enemyTeamGoalBlobs.at(0);
            goalCenterX = biggestBlob.x + biggestBlob.w / 2;
            goalCenterY = biggestBlob.y + biggestBlob.h / 2;

            goalCorner1X = biggestBlob.x;
            goalCorner1Y = biggestBlob.y;
            goalCorner2X = biggestBlob.x + biggestBlob.w;
            goalCorner2Y = biggestBlob.y + biggestBlob.h;

            goalCorner1Angle = atan((HEIGHT - goalCorner1Y) / (WIDTH / 2 - goalCorner1X)) * 180 / PI;
            goalCorner2Angle = atan((HEIGHT - goalCorner2Y) / (WIDTH / 2 - goalCorner2X)) * 180 / PI;
            goalAngle = atan((HEIGHT - goalCenterY) / (WIDTH / 2 - goalCenterX)) * 180 / PI;

            goalCenterDistance = std::sqrt(std::pow(WIDTH / 2 - goalCenterX, 2) + std::pow(HEIGHT - goalCenterY, 2));
            goalCenterDistance = (-19.1033) / (1 - 29.841 * std::pow(EULER, -0.0092 * goalCenterDistance));

            if (goalAngle > 0) goalAngle -= 90;
            else goalAngle += 90;

            if (goalCorner1Angle > 0) goalCorner1Angle -= 90;
            else goalCorner1Angle += 90;

            if (goalCorner2Angle > 0) goalCorner2Angle -= 90;
            else goalCorner2Angle += 90;
        }

        bool intercepts = false;

        if (goalCorner1X && goalCorner2X && ballCenterX) {
            intercepts = (ballCenterX > goalCorner1X) && (ballCenterX < goalCorner2X);
        }

        if (intercepts) {
            ROS_INFO("Ball is infront of the goal");
        }

        std_msgs::Float32MultiArray msg;
        msg.data.resize(14);
        msg.data[0] = ballCenterX;
        msg.data[1] = ballCenterY;
        msg.data[2] = goalCenterX;
        msg.data[3] = goalCenterY;
        msg.data[4] = goalCorner1X;
        msg.data[5] = goalCorner1Y;
        msg.data[6] = goalCorner2X;
        msg.data[7] = goalCorner1Y;
        msg.data[8] = goalCorner1Angle;
        msg.data[9] = goalCorner2Angle;
        msg.data[10] = goalCenterDistance;
        msg.data[11] = goalAngle;
        msg.data[12] = ballAngle;
        msg.data[13] = ballDistance;

        pub.publish(msg);
        cv::imwrite("/home/aizer/frontcamera/frame_" + std::to_string(frame_id) + ".jpg", frame);
/*
        #ifdef SHOW_IMG
        cv::imshow("Front-Camera", frame);

        if (cv::waitKey(10) == KEY_ESC) {
            break;
        }
        #endif
*/
    }

    cap.release();

//    #ifdef SHOW_IMG
//    cv::destroyAllWindows();
//    #endif

    return 0;
}

