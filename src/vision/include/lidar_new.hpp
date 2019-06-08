#ifndef LIDAR
#define LIDAR

#include <bits/stdc++.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"

/*#define obstacleWidth 30*/
using namespace ros;
using namespace std;
using namespace cv;

bool is_laserscan_retrieved = false;

sensor_msgs::LaserScan lidar_scan;
void laserscan(sensor_msgs::LaserScan msg)   /// CALLBACK FOR LIDAR DATA
{
    lidar_scan = msg;
    is_laserscan_retrieved = true;
}
#endif
