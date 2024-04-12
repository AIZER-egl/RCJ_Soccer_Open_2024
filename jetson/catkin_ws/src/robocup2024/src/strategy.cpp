#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt64.h>
#include "rpi-bitmask.h"
#include "chassis.h"
#include "angle.h"
#include "pid.h"

float omni_ballDistance;
float omni_ballAngle;

float ballCenterX;
float ballCenterY;
float goalCenterX;
float goalCenterY;
float goalCorner1X;
float goalCorner1Y;
float goalCorner2X;
float goalCorner2Y;
float goalAngle1;
float goalAngle2;
float goalAngle;
float ballAngle;
float ballDistance;
float goalCenterDistance;

PID::PidParameters chassisLinearPID;
PID::PidParameters anglePID;

void omni_message(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    omni_ballDistance = msg -> data[0];
    omni_ballAngle = msg -> data[1];
}

void front_message(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    ballCenterX = msg -> data[0];
    ballCenterY = msg -> data[1];
    goalCenterX = msg -> data[2];
    goalCenterY = msg -> data[3];
    goalCorner1X = msg -> data[4];
    goalCorner1Y = msg -> data[5];
    goalCorner2X = msg -> data[6];
    goalCorner2Y = msg -> data[7];
    goalAngle1 = msg -> data[8];
    goalAngle2 = msg -> data[9];
    goalCenterDistance = msg -> data[10];
    goalAngle = msg -> data[11];
    ballAngle = msg -> data[12];
    ballDistance = msg -> data[13];
}

void rpi_message(const std_msgs::UInt64::ConstPtr& msg) {
    RPI_Bitmask::setBitmask(msg -> data);

    ROS_INFO("Data received, S: %d, NE: %d, NW: %d, yaw: %d",
             RPI_Bitmask::bitmaskToValue(static_cast<int>(RPI_Bitmask::MOTOR_ADDRESS::S_SPEED)),
             RPI_Bitmask::bitmaskToValue(static_cast<int>(RPI_Bitmask::MOTOR_ADDRESS::NE_SPEED)),
             RPI_Bitmask::bitmaskToValue(static_cast<int>(RPI_Bitmask::MOTOR_ADDRESS::NW_SPEED)),
             RPI_Bitmask::bitmaskToValue(static_cast<int>(RPI_Bitmask::COMPASS_ADDRESS::YAW))
             );
}

bool intercepts () {
    if (goalCorner1X == 9999) return false;
    if (goalCorner2X == 9999) return false;
    if (ballCenterX == 9999) return false;

    return (goalCorner1X < ballCenterX) && (ballCenterX < goalCorner2X);
}

uint64_t time123 = 0;
bool firstRun = true;
bool closeToGoal = false;
uint64_t lastTime = 0;
void attack() {
    int robot_yaw = RPI_Bitmask::bitmaskToValue(static_cast<int>(RPI_Bitmask::COMPASS_ADDRESS::YAW));
    Chassis::rotateToAngle(0, robot_yaw, anglePID);

    //    chassisLinearPID.target = 0;
//    chassisLinearPID.maxOutput = 100 * 0.5;
//
//    Chassis::move(100, 90, robot_yaw, chassisLinearPID);
//    int ballQuadrant = Angle::quadrant(omni_ballAngle);
//    int bias = 0;
//    ROS_INFO("Quadrant: %d, omnicamera %f", ballQuadrant, omni_ballAngle);
//    if (ballQuadrant == 1 || ballQuadrant == 2) bias = 50;
//    if (ballQuadrant == 3 || ballQuadrant == 4) bias = - 50;
//
//    int omni_ballDistance_copy = omni_ballDistance;
//    if (omni_ballDistance_copy > 100) omni_ballDistance_copy = 100;
//    int speed = Chassis::map(omni_ballDistance_copy, 0, 100, 60, 150);
//
//    if (!intercepts()) {
//        ROS_INFO("Ball is not infront of the goal, %f", omni_ballAngle + bias);
//        chassisLinearPID.maxOutput = speed * 0.3;
//        Chassis::move(speed, omni_ballAngle + bias, robot_yaw, chassisLinearPID);
//        return;
//    }
//
//    chassisLinearPID.maxOutput = 125 * 0.3;
//    ROS_INFO("Ball is infront of the goal");
//    Chassis::move(125, ballAngle, robot_yaw, chassisLinearPID);
}

void publish_task(ros::Publisher& pub) {
    lastTime = PID::millis();
    while (ros::ok()) {
//        if (omni_ballAngle > 0) {
            attack();
//        } else {
//            ROS_INFO("Unable to locate ball");
//            Chassis::move(0, 0);
//        }

        std_msgs::UInt64 msg;
        msg.data = RPI_Bitmask::generateBitmask();
        pub.publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 60));
    }
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "strategy");
    ros::NodeHandle nh;

    ROS_INFO("Listening for messages");

    chassisLinearPID.kp = 1.3;
    chassisLinearPID.ki = 0;
    chassisLinearPID.kd = 0;
    chassisLinearPID.maxOutput = 30;
    chassisLinearPID.minOutput = 0;
    chassisLinearPID.maxError = 750;
    chassisLinearPID.errorThresholdPermission = 2;
    chassisLinearPID.target = 0;
    chassisLinearPID.delayMiliseconds = 20;

    anglePID.kp = 0.3;
    anglePID.ki = 0;
    anglePID.kd = 0;
    anglePID.maxOutput = 200;
    anglePID.minOutput = 30;
    anglePID.maxError = 750;
    anglePID.errorThresholdPermission = 10;
    anglePID.target = 0;
    anglePID.delayMiliseconds = 20;

    ros::Subscriber omni_sub = nh.subscribe("omnicamera_topic", 10, omni_message);
    ros::Subscriber front_sub = nh.subscribe("frontcamera_topic", 10, front_message);
    ros::Subscriber rpi_sub = nh.subscribe("rpi2jetson", 10, rpi_message);
    ros::Publisher pub = nh.advertise<std_msgs::UInt64>("jetson2rpi", 10);

    std::thread publish_thread(publish_task, std::ref(pub));

    ros::spin();

    publish_thread.join();

    return 0;
}
