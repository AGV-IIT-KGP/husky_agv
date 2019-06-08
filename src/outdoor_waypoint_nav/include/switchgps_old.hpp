#ifndef GPS_WAYPOINT
#define GPS_WAYPOINT

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "robot_localization/navsat_conversions.h"
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>
using namespace std;


// initialize variables

typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction>
MoveBaseClient; //create a type definition for a client called MoveBaseClient

std::vector <std::pair<double, double>> waypointVect;
std::vector<std::pair < double, double> > ::iterator iter; //init. iterator
geometry_msgs::PointStamped UTM_point, UTM_point_current, map_point, map_point_current, UTM_next, map_next;
int count = 0, waypointCount = 0, wait_count = 0;
double numWaypoints = 0;
double latiGoal, longiGoal, latiNext, longiNext;
std::string utm_zone;
std::string path_local, path_abs;


geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input)
{
    double utm_x = 0, utm_y = 0;
    geometry_msgs::PointStamped UTM_point_output;

    //convert lat/long to utm
    RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

    //Construct UTM_point and map_point geometry messages
    UTM_point_output.header.frame_id = "utm";
    UTM_point_output.header.stamp = ros::Time(0);
    UTM_point_output.point.x = utm_x;
    UTM_point_output.point.y = utm_y;
    UTM_point_output.point.z = 0;

    return UTM_point_output;
}

geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
{
    geometry_msgs::PointStamped map_point_output;
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
        try
        {
            UTM_point.header.stamp = ros::Time::now();
            listener.waitForTransform("odom", "utm", time_now, ros::Duration(0.5));
            listener.transformPoint("odom", UTM_input, map_point_output);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
            //return;
        }
    }
    return map_point_output;
}

move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped map_point, geometry_msgs::PointStamped map_point_current, geometry_msgs::PointStamped map_next, bool last_point)
{
    move_base_msgs::MoveBaseGoal goal;

    //Specify what frame we want the goal to be published in
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();

    // Specify x and y goal
    goal.target_pose.pose.position.x = map_point.point.x; //specify x goal
    goal.target_pose.pose.position.y = map_point.point.y; //specify y goal
    // Specify heading goal using current goal and next goal (point robot towards its next goal once it has achieved its current goal)
    if(last_point == false)
    {
        tf::Matrix3x3 rot_euler;
        tf::Quaternion rot_quat;

        // Calculate quaternion
        float x_curr = map_point.point.x, y_curr = map_point.point.y; // set current coords.
        float x_next = map_next.point.x, y_next = map_next.point.y; // set coords. of next waypoint
        float delta_x = x_next - x_curr, delta_y = y_next - y_curr;   // change in coords.
        float yaw_curr = 0, pitch_curr = 0, roll_curr = 0;
        yaw_curr = atan2(delta_y, delta_x);

        // Specify quaternions
        rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
        rot_euler.getRotation(rot_quat);

        goal.target_pose.pose.orientation.x = rot_quat.getX();
        goal.target_pose.pose.orientation.y = rot_quat.getY();
        goal.target_pose.pose.orientation.z = rot_quat.getZ();
        goal.target_pose.pose.orientation.w = rot_quat.getW();
    }
    else
    {
        double pitch=0, roll=0, yaw;
        yaw = atan2(map_point.point.y - map_point_current.point.y, map_point.point.x - map_point_current.point.x);
        ROS_INFO("Yaw:%.8f", yaw);
        tf::Quaternion q;
        q.setRPY(tfScalar(roll), tfScalar(pitch), tfScalar(yaw));
        goal.target_pose.pose.orientation.x = q.getX();
        goal.target_pose.pose.orientation.y = q.getY();
        goal.target_pose.pose.orientation.z = q.getZ();
        goal.target_pose.pose.orientation.w = q.getW();
    }
    return goal;
}


int gps_waypoint(double end_lat, double end_long, double current_lat, double current_long)
{

    MoveBaseClient ac("/move_base", true);
    
    
    //construct an action client that we use to communication with the action named move_base.
    //Setting true is telling the constructor to start ros::spin()

    //Get Longitude and Latitude goals from text file

    //Count number of waypoints
    

    // Iterate through vector of waypoints for setting goals

        //Setting goal:
        latiGoal = end_lat;
        longiGoal = end_long;

        ROS_INFO("Received Latitude goal:%.8f", latiGoal);
        ROS_INFO("Received longitude goal:%.8f", longiGoal);

        //Convert lat/long to utm:
        UTM_point = latLongtoUTM(latiGoal, longiGoal);
        UTM_point_current = latLongtoUTM(current_lat, current_long);

        //Transform UTM to map point in odom frame
        map_point = UTMtoMapPoint(UTM_point);
        map_point_current = UTMtoMapPoint(UTM_point_current);

        map_next = map_point;
        bool final_point = true;

        //Build goal to send to move_base
        move_base_msgs::MoveBaseGoal goal = buildGoal(map_point, map_point_current, map_next, final_point); //initiate a move_base_msg called goal
        // Send Goal
        ROS_INFO("Sending goal");
        ac.sendGoal(goal); //push goal to move_base node

        //Wait for result
        ac.waitForResult(); //waiting to see if move_base was able to reach goal

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Eklavya has reached its goal!");
            return 0; //if safely exiting

        }
        else
        {
            ROS_ERROR("Eklavya was unable to reach its goal. GPS Waypoint unreachable.");
            return 1; //exiting with error
        }
}

#endif