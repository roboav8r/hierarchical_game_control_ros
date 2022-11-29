#ifndef LOWLEVEL_CONTROLLER_NODE_H_
#define LOWLEVEL_CONTROLLER_NODE_H_

#include <ros/ros.h>


#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include "tf/transform_datatypes.h"

#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

// ROS messages
nav_msgs::Odometry robotOdom;
tf::Quaternion robotOrientation;
geometry_msgs::Twist controlMsg;

// State variables
double robotX;
double robotY;
double robotV;
double robotYawRate;
double robotHeading,robotRoll, robotPitch;
int numScans{11};
int numMeasScans{640};
float minScanAngle{-2.268899917602539};
float maxScanAngle{2.268899917602539};
float rangeMax{10.0};
float scanInc{0.007101408671587706};
std::vector<float> scanAngles(numScans);
std::vector<float> scanRanges(numScans);
std::vector<int> scanIndices(numScans);
std::vector<float> scanXRobot(numScans);
std::vector<float> scanYRobot(numScans);
std::vector<float> scanXMap(numScans);
std::vector<float> scanYMap(numScans);

// Cost weights
float yawRateCost{0.5};
float accCost{0.1};
float goalCost{5.0};
float latObstCost{0.1};
float axObstCost{0.5};

// System limits
float minvel = -0.1;
float maxvel = 5.0;

// ROS Subscriber callback
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

void scanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    // Convert from raw scan data into limited scan dataset for processing
    std::cout << "Laser scan has n_points = " << msg->ranges.size() << std::endl;
    for (int ii = 0; ii < numScans; ii++) 
    {
        msg->ranges[scanIndices[ii]] > rangeMax ? scanRanges[ii] = rangeMax : scanRanges[ii] = msg->ranges[scanIndices[ii]];
        scanXRobot[ii] = scanRanges[ii]*cos(scanAngles[ii]);
        scanYRobot[ii] = scanRanges[ii]*sin(scanAngles[ii]);
        scanXMap[ii] = robotX + cos(robotHeading)*scanXRobot[ii] - sin(robotHeading)*scanYRobot[ii];
        scanYMap[ii] = robotY + sin(robotHeading)*scanXRobot[ii] + cos(robotHeading)*scanYRobot[ii];
        std::cout << scanAngles[ii] << "," << scanRanges[ii] <<": " << scanXRobot[ii] << "," << scanYRobot[ii] <<std::endl;
    };
    // robotOdom = (*msg);
}

// Solver variables
Eigen::VectorXd uOptimal; // optimal control input

#endif //LOWLEVEL_CONTROLLER_NODE_H_