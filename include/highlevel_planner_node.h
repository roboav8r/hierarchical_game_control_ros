#ifndef HIGHLEVEL_PLANNER_NODE_H_
#define HIGHLEVEL_PLANNER_NODE_H_

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "tf/transform_datatypes.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "nav_msgs/Odometry.h"

#include "graph_datatypes.h"

using Eigen::Vector2d;

/*
ROS messages and variables
*/
nav_msgs::Odometry robotOdom;
tf::Quaternion robotOrientation;

// State variables
double robotX{0.0};
double robotY{10.0};
double robotV;
double robotYawRate;
double robotHeading,robotRoll, robotPitch;

float dt = 3.0;

// Mapping/visualization
cv::Mat mapImg;
cv::Mat plannerImg, scaledImg;
std::string mapPath;
double mapResolution{.05};
double mapOriginX{-99.7};
double mapOriginY{-90.0};
float freeThresh;
uint robotXpixel;
uint robotYpixel;

// MCTS/game variables
std::shared_ptr<GameState> currentState;
//Tree gameTree;
bool robotHasCargo{false};
bool cargoAtPickup{true};
bool cargoAtDest{false};
float cargoX{5.0};
float cargoY{12.0};
uint pickupXpixel;
uint pickupYpixel;
float destX{-5.0};
float destY{12.0};
uint dropoffXpixel;
uint dropoffYpixel;
float cargoDist{1.0};
float movementStepSize{1.0};
int pixStepsize;
bool mapNegate{false};

/*
Functions
*/
// bool inCargoDist(float rob_x, float rob_y, float pos_x, float pos_y){
//     return sqrt(pow(rob_x - pos_x,2) + pow(rob_y - pos_y,2)) <= cargoDist;
// }

// Convert between ROS map coordinates and actual map pixels for planning
void updateRobotPixels()
{ 
    robotXpixel = (robotX - mapOriginX)/mapResolution;
    robotYpixel = (robotY - mapOriginY)/mapResolution;
}

void updateCargoPixels()
{
    pickupXpixel = (cargoX - mapOriginX)/mapResolution;
    pickupYpixel = (cargoY - mapOriginY)/mapResolution;
    dropoffXpixel = (destX - mapOriginX)/mapResolution;
    dropoffYpixel = (destY - mapOriginY)/mapResolution;
}

void sendGoalMessage(ros::Publisher& goal_pub, float deltaX, float deltaY)
{
    move_base_msgs::MoveBaseGoal goalMsg;
    goalMsg.target_pose.header.frame_id = "map";
    goalMsg.target_pose.pose.position.x = robotX + deltaX;
    goalMsg.target_pose.pose.position.y = robotY + deltaY;
    goal_pub.publish(goalMsg);
}

/*
ROS Callbacks
*/
void robotOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    robotOdom = (*msg);
    robotX = robotOdom.pose.pose.position.x;
    robotY = robotOdom.pose.pose.position.y;
    robotV = robotOdom.twist.twist.linear.x;
    robotYawRate = robotOdom.twist.twist.angular.z;

    tf::quaternionMsgToTF(msg->pose.pose.orientation, robotOrientation);
    tf::Matrix3x3(robotOrientation).getRPY(robotRoll, robotPitch, robotHeading);
}


#endif //HIGHLEVEL_PLANNER_NODE_H_