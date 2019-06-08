#ifndef HOUGH
#define HOUGH

#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/core.hpp"
#include<iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace cv;

/*

This is used when the vision module encouters a horizontal
  lane(since due to fixed x-axis, RANSAC can't fit them).
  
This module:
  * check_whether_hough: first draws the horizontal lane.
  * waypoint_for_hough: Generates a waypoint for the lane.

*/

double theta;

/*
The slope is -ve of what it is by convention.

The -ve sign is introduced to change the origin from 
  the image frame(top-left) to the co-ordinate 
  frame(bottom-left).
*/
double Slope(int x0, int y0, int x1, int y1)
{
  //NOTE the angle.
  return (double)(y0-y1)/(x1-x0);

}

double fullLine(Mat *img, Point a, Point b, Scalar color)
{
    double slope = Slope(a.x, a.y, b.x, b.y);

    Point p(0,0), q(img->cols, img->rows); // '->' used since pointer to image is used

    //(y1 - y0)= m(x1 - x0)
    p.y = (p.x - a.x) * slope + a.y;
    q.y = (q.x - b.x) * slope + b.y;
    //line(*img,p,q,color,3,CV_AA);

    //Setting slope range for which hough code will initiate
    if((atan(slope)*180/CV_PI) <= 30.00 && (atan(slope)*180/CV_PI) >= -30.00)
      line(*img,p,q,color,3,CV_AA);
    theta = atan(slope)*180/CV_PI;
    return slope;
}

/*
Checks whether hough line is to be applied or not.
Takes for input:
    * Final output image of pre-processing (img)
    * Blank image for plotting the hough line (hough_img)
*/
bool check_whether_hough(Mat hough_img,Mat img)
{
  vector<Vec4i> lines;  //NOTE the vector type
  
  //Hough Lines Probabilistic
  HoughLinesP(img, lines, 1, CV_PI/180, hough_min_points, hough_min_line_length, hough_max_line_gap); 
  /*
  Params: dest_img, vector to store pts, resolution of r,
         resolution of theta, minimum no. points, 
         min. line length, maxlineGap 
  */ 

  Point p1(0,0), p2(0,0);
  double slope_avg=0;

  //Finding maximum line length & its index in lines[]
  //  We assume that the longest line will be the lane
  float max_dist = 0;
  int max_index = 0;
  for(size_t i = 0; i < lines.size(); i++)
  {
    Vec4i l = lines[i];
    if(sqrt(pow(l[0]-l[2],2) + pow(l[1]-l[3],2)) > max_dist)
    {
      max_dist = sqrt( pow(l[0]-l[2],2) + pow(l[1]-l[3],2));
      max_index = i;
    }
  }

  if(lines.size() == 0)
  {
    if(false) {cout << "----------------------\nNo hough line could be fit\n----------------------" << endl;}
    return false;
  }

  //Largest line= lane
  Vec4i l = lines[max_index];
  p1.x = l[0];
  p1.y = l[1];
  p2.x = l[2];
  p2.y = l[3];

  
  double slope = fullLine(&hough_img, p1, p2, Scalar(255));
  
  //Checking that the line slope is within a certain threshold angle from the x-axis
  if((atan(slope)*180/CV_PI) <= 20.00 && (atan(slope)*180/CV_PI) >= -20.00)
    return true;

  return false;
      
}

//Giving waypoint if hough initiated
NavPoint waypoint_for_hough(Mat img, char c, float theta)
{
    int ver_i,ver_j;
    
    //Finding lane centre in the image
    for(int i=img.rows -1;i>=0;i--)
    {
        if(img.at<uchar>(i,img.cols/2) == 255)
        {
            ver_i = i;
            ver_j = img.cols/2;
            break;
        }
    }

    int waypoint_i,waypoint_j;
    //Giving point to shift 2/3rd right & down
    if(c == 'l')
    {
        waypoint_i = (ver_i + 2*img.rows -1)/3;
        waypoint_j = (ver_j + 2*img.cols -1)/3;  
        
        /*changing co-ordinate system
            Image frame -> co-ordinate frame*/
        theta = theta*CV_PI/180 - CV_PI/2;
    }

    //Giving point to shift 2/3rd right & up
    else if(c == 'r')
    {
        waypoint_i = (ver_i + 2*img.rows -2)/3;
        waypoint_j = (ver_j)/3;
        /*changing co-ordinate system + reverse 
            i.e. co-ordinate angle + CV_PI*/
        theta = theta*CV_PI/180 + CV_PI/2;
    }
    
    //Setting waypoint position + orientation
    NavPoint way_point;
    way_point.x = waypoint_j;
    way_point.y = waypoint_i;
    way_point.angle = theta;
    return way_point;
}

#endif