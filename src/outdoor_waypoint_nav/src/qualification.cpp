#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <bits/stdc++.h>
#include "sensor_msgs/NavSatFix.h"
#include <bits/stdc++.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "robot_localization/navsat_conversions.h"
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <stdio.h>

using namespace std;
using namespace ros;

sensor_msgs::NavSatFix coordinates;

double current_lat, current_long;
int seq;
bool gps_data = 0;

typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient; //create a type definition for a client called MoveBaseClient

int main(int argc, char** argv)
{
    init (argc, argv, "switchgps");
    NodeHandle n;

    //read waypoints
    FILE* odom_pts;
    double x_goal, y_goal;
    try {
        odom_pts=fopen("odom_points.txt","r");
        if (odom_pts == NULL) {
            throw -1;
        }
        fscanf(odom_pts,"%lf %lf",&x_goal, &y_goal);
    }
    catch (int e) {
        ROS_INFO("Try running the gps switching code from the root of the IGVC workspace\n");
        return 0;
    }

    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();
    cout << "time: " << goal.target_pose.header.stamp << endl;

    cout << "x_goal: " << x_goal << " y_goal: " << y_goal << endl;
    goal.target_pose.pose.position.x = x_goal;
    goal.target_pose.pose.position.y = y_goal;
    goal.target_pose.pose.orientation.w = 1.0;

    //Build goal to send to move_base
    ROS_INFO("Sending goal");
    ac.sendGoal(goal); //push goal to move_base node

    //Wait for result
    ac.waitForResult(); //waiting to see if move_base was able to reach goal

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Eklavya has reached its goal!");
        return 1;

    }
    else
    {
        ROS_ERROR("Eklavya was unable to reach its goal. GPS Waypoint unreachable.");
        return 0;
    }


    return 1;
}
