#ifndef RANSAC
#define RANSAC


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <bits/stdc++.h>

using namespace std;
using namespace cv;

//define no of iteration_2, max dist squre of pt from our estimated model
#define iteration_2 30
#define maxDist_2 5
//define threshold distance to remove white pixel near lane1
#define removeDist 20
//define minimum number of points to be lie on a lane
#define minlaneInlier_2 30

#define minPointsForRANSAC_2 40

Parabola2 swap_2(Parabola2 param) {

    float temp1, temp2,temp3;
    temp1=param.a1;
    temp2=param.b1;
    temp3=param.c1;

    param.a1=param.a2;
    param.b1=param.b2;
    param.c1=param.c2;

    param.a2=temp1;
    param.b2=temp2;
    param.c2=temp3;

    return param;
}

// Parabola2 classify_lanes(Mat img,Parabola2 present,Parabola2 previous)
// {
//     float a1=present.a1;
//     float a2=present.a2;
//     float b1=present.b1;
//     float b2=present.b2;
//     float c1=present.c1;
//     float c2=present.c2;
//     int number_of_lanes=present.numModel;

//     if(number_of_lanes==2)
//     {
//         if((a2*img.rows*img.rows+b2*img.rows+c2)<(a1*img.rows*img.rows+b1*img.rows+c1))
//         {
//             present=swap_2(present);
//             return present;
//         }
//         else 
//             return present;
//     }

//     else if(number_of_lanes==1)
//     {
//         if((a1*img.rows*img.rows + b1*img.rows + c1)>(img.cols/2))
//         {
//             present=swap_2(present);
//             return present;
//         }

//         if((a2*img.rows*img.rows + b2*img.rows + c2)<(img.cols/2))
//         {
//             present=swap_2(present);
//             return present;
//         }

//     }

//     return present;

// }

//calculation of model parameters based on 3 randonmly selected points
float get_a_2(Point p1, Point p2, Point p3)
{
  int x1 = p1.x;
  int x2 = p2.x;
  int x3 = p3.x;
  int y1 = p1.y;
  int y2 = p2.y;
  int y3 = p3.y;
  float del = (y2 - y1)*(y3 - y2)*(y1 - y3);
  float del_a = (x2 - x1)*(y3 - y2) - (x3 - x2)*(y2 - y1);
  return(del_a / del);
}
float get_b_2(Point p1, Point p2, Point p3)
{
  int x1 = p1.x;
  int x2 = p2.x;
  int x3 = p3.x;
  int y1 = p1.y;
  int y2 = p2.y;
  int y3 = p3.y;
  float del = (y2 - y1)*(y3 - y2)*(y1 - y3);
  float del_b = (x3 - x2)*((y2*y2) - (y1*y1)) - (x2 - x1)*((y3*y3) - (y2*y2));
  return (del_b / del);
}
float get_c_2(Point p, float a, float b)
{
  int x = p.x;
  int y = p.y;
  return(x - (a*y*y) - (b*y));
}

//calculation of error b/w actual and estimated y
float get_delX(Point p, float a, float b, float c)
{
    float predictedX = (a*(p.y*p.y) + b*p.y + c);
    float errorx=fabs(p.x - predictedX);
    float y1,y2,errory;

    y1 = (-b - sqrt((b*b)-(4*a*(c-p.x))))/(2*a);
    y2 = (-b + sqrt((b*b)-(4*a*(c-p.x))))/(2*a);
    
    if(y1 < 0)
      y1=10000000;
    if(y2 < 0)
      y2=10000000;
    errory = min(fabs(y1 - p.y),fabs(y2 - p.y));
    // cout<<"errorx  : "<<errorx<<" errory : "<<errory<<endl;
    return(min(errorx,errory));
}

//choose model parameters of best fit curve basis on randomly selected 3 points
Parabola2 ransac_2_left(vector<Point> ptArray, Parabola2 param)
{
  int dataPt = ptArray.size();
  int maxInlier = 0;
  Parabola2 tempParam;
  for(int i = 0; i < iteration_2; ++i)
  {
    int d1 = rand() % dataPt;
    int d2 = rand() % dataPt;
    int d3 = rand() % dataPt;
    Point p1 = ptArray[d1];
    Point p2 = ptArray[d2];
    Point p3 = ptArray[d3];

    if((p1.x == p2.x) || (p2.x == p3.x) || (p3.x == p1.x)||(p1.y == p2.y) || (p2.y == p3.y) || (p3.y == p1.y))
    {
      continue;
    }

    //get the model parameters
    float temp_a = get_a_2(p1, p2, p3);
    float temp_b = get_b_2(p1, p2, p3);
    float temp_c = get_c_2(p1, temp_a, temp_b);
    //count inliers on predicted model
    int tempInlier = 0;
    for(int j = 0; j < dataPt; ++j)
    {
      Point z = ptArray[j];
      float error = get_delX(z, temp_a, temp_b, temp_c);
      if(error < maxDist_2)
      {
        ++tempInlier;
      }
    }
    if((tempInlier > maxInlier) || (i == 0))
    {
      maxInlier = tempInlier;
      tempParam.a1 = temp_a;
      tempParam.b1 = temp_b;
      tempParam.c1 = temp_c;
    }
  }
  if(maxInlier > minlaneInlier_2)
  {
    
      param.a1 = tempParam.a1;
      param.b1 = tempParam.b1;
      param.c1 = tempParam.c1;
      param.numModel += 1;
  }
  return param;
}

Parabola2 ransac_2_right(vector<Point> ptArray, Parabola2 param)
{
  int dataPt = ptArray.size();
  int maxInlier = 0;
  Parabola2 tempParam;
  for(int i = 0; i < iteration_2; ++i)
  {
    int d1 = rand() % dataPt;
    int d2 = rand() % dataPt;
    int d3 = rand() % dataPt;
    Point p1 = ptArray[d1];
    Point p2 = ptArray[d2];
    Point p3 = ptArray[d3];

    if((p1.x == p2.x) || (p2.x == p3.x) || (p3.x == p1.x)||(p1.y == p2.y) || (p2.y == p3.y) || (p3.y == p1.y))
    {
      continue;
    }

    //get the model parameters
    float temp_a = get_a_2(p1, p2, p3);
    float temp_b = get_b_2(p1, p2, p3);
    float temp_c = get_c_2(p1, temp_a, temp_b);
    //count inliers on predicted model
    int tempInlier = 0;
    for(int j = 0; j < dataPt; ++j)
    {
      Point z = ptArray[j];
      float error = get_delX(z, temp_a, temp_b, temp_c);
      if(error < maxDist_2)
      {
        ++tempInlier;
      }
    }
    if((tempInlier > maxInlier) || (i == 0))
    {
      maxInlier = tempInlier;
      tempParam.a1 = temp_a;
      tempParam.b1 = temp_b;
      tempParam.c1 = temp_c;
    }
  }
  if(maxInlier > minlaneInlier_2)
  {
      param.a2 = tempParam.a1;
      param.b2 = tempParam.b1;
      param.c2 = tempParam.c1;
      param.numModel += 1;
  }
  return param;
}

//Check wheather a point is near lane1
bool IsNearLane1(Parabola2 param1, Point p)
{
  float dist_ = get_delX(p, param1.a1, param1.b1, param1.c1);
  if(dist_ < removeDist)
  {
    return true;
  }
  else
  {
    //cout<<"dist_ : "<<dist_<<" removeDist : "<<removeDist<<endl;
    return false;
  }
}

float distance_nikaal(Point A,Point B)
{
  return (sqrt(pow(A.x-B.x,2)+pow(A.y-B.y,2)));
}

Point centroid(float a,float b,float c,Mat img)
{
  Point A;
  int i,j,x,y;
  int sum_x,sum_y,count=0;

  for(j=0;j<img.rows;j++)
  {
    x=a*j*j+b*j+c;

    if(x>=0 && x<img.cols)
    {
      sum_y+=j;
      sum_x+=x;
      count++;
    }
  }

  A.x=sum_x/count;
  A.y=sum_y/count;

  return A;

}

Parabola2 getRansacModel_2(Mat img,Parabola2 previous)
{
  //apply ransac for first time it will converge for one lane
  vector<Point> ptArray1,ptArray2;
  Point A,B,C;
  for(int i = 0; i < img.rows; ++i)
  {
    for(int j = 0; j < img.cols; ++j)
    { 
        Point pt;
        pt.x = j;
        pt.y = i;
        if(img.at<Vec3b>(i,j)[0]>50)
          ptArray1.push_back(pt);  
        else if(img.at<Vec3b>(i,j)[2]>50)
          ptArray2.push_back(pt);   
    }
  }
  //cout<<"ptArray1.size() : "<<ptArray1.size()<<" \nminPointsForRANSAC_2 : "<<minPointsForRANSAC_2<<endl;
  //declare a model vaiable to store the model
  Parabola2 param;

  //get parameters of first model form ransac function
  if(ptArray1.size() > minPointsForRANSAC_2)
  {
    //cout<<"Ransac Called\n";
    param = ransac_2_left(ptArray1, param);
  }

 

  float temp1,temp2,temp3;
  //get parameters of second model form ransac function
  if(ptArray2.size() > minPointsForRANSAC_2)
  {
    param = ransac_2_right(ptArray2, param);
  }

   if(param.numModel==0)
   {
     //cout<<"Returning previous"<<endl;
    //return previous;
   }

  return param;
}

Mat drawLanes_top(Mat output, Parabola2 lanes) 
{
    vector<Point2f> left_lane, right_lane;
    float a1 = lanes.a1, a2 = lanes.a2, b1 = lanes.b1 , b2 = lanes.b2 , c1 = lanes.c1, c2 = lanes.c2;

    for (int j = 0; j < output.rows; j++){

        float x, y;
        if (a1 != 0 && c1 != 0 && b1 != 0) {
            y = j;
            x = (y*y)*(a1) + b1*y + c1;
            left_lane.push_back(Point2f(x, j));
        }

        if (a2 != 0 && c2 != 0 && b2!=0) {
            y = j;
            x = (y*y)*(a2) + b2*y + c2;
            right_lane.push_back(Point2f(x, j));
        }

    }

    //Left lane in Blue
    Mat left_curve(left_lane, true);
    left_curve.convertTo(left_curve, CV_32S); //adapt type for polylines
    polylines(output, left_curve, false, Scalar(255, 0, 0), 3, CV_AA);

    //Right lane in Red
    Mat right_curve(right_lane, true);
    right_curve.convertTo(right_curve, CV_32S); //adapt type for polylines
    polylines(output, right_curve, false, Scalar(0, 0, 255), 3, CV_AA);

    return output;
}

#endif
