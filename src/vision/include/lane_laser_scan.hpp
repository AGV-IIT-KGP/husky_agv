#ifndef LANE_LASER_SCAN
#define LANE_LASER_SCAN

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <cmath>
#include <limits>
#include <iostream>

using namespace ros;
using namespace std;
using namespace cv;


/*
Converts any binary image's white points(Obstacles/Lanes) 
    into a LaserScan(LIDAR format) Message.
*/

//Takes a b/w image as input
sensor_msgs::LaserScan laneLaser(Mat img)
{
    sensor_msgs::LaserScan scan;
    scan.angle_min = -CV_PI/2;
    scan.angle_max = CV_PI/2;
    scan.angle_increment = CV_PI/bins;
    
    //NOTE the declaration of infinity
    double inf = std::numeric_limits<double>::infinity();
    scan.range_max = inf; 
    //range_max= maximum r value after which obstacles are to be seen 

    //NOTE frame_id
    scan.header.frame_id = "laser";
    scan.header.stamp = ros::Time::now();
    scan.scan_time = 0.025;     //Total scan duration
    scan.time_increment = (float)(scan.scan_time)/bins; //Time interval between scanning 2 angles
    
    //Initialising each LIDAR scan distance to infinity
    //  This allows for rest of the points to not be given in the LaserScan
    for (int i=0;i<bins;i++)
    {
        scan.ranges.push_back(scan.range_max);
    }

    scan.range_max = 80; //Set according to desire.

    //Putting in 
    for(int i = 0; i < img.rows; ++i)
    {
        for(int j = 0; j < img.cols; ++j)
        {
            if(img.at<uchar>(i,j)>0)
            {
                /*
                Converting image co-ordinates to real ones,
                    i.e shifting origin from top-left(image frame)
                    to bottom-centre(LIDAR frame).
                */ 
                float a = (j - img.cols/2)/pixelsPerMetre;
                float b = (img.rows - i)/pixelsPerMetre + yshift;

                double angle = atan(a/b);

                double r = sqrt(a*a  + b*b);

                int k = (angle - scan.angle_min)/(scan.angle_increment);    
                //For finding bin no. 
                
                //Assigning r if present
                if (r < scan.ranges[bins-k-1])  //NOTE (bins-k-1) 
                    scan.ranges[bins-k-1] = r ;
            }
        }
    }

    return scan;    /// returns LaserScan data
}

#endif
