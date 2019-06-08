#ifndef REMOVE_OBSTACLES
#define REMOVE_OBSTACLES
#include <bits/stdc++.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

Mat orange_barrels(Mat img1) {

  int max = 50;

  for (int j = 0; j < img1.cols ;j++) {
    int flag = 0;
    int count = 0;
    int index = 0;
    for (int i = 0; i < img1.rows-1; i++) {
      if (flag == 1 && img1.at<uchar>(i,j) == 0) {
        count++;
      }
      if (flag == 1 && img1.at<uchar>(i,j) == 255) {
        if (count < max) {
          for (int i = index; i < index+count; i++) {
            img1.at<uchar>(i,j) = 255;
          }
        }
        flag = 0;
        count = 0;

      }
      //black
      if (img1.at<uchar>(i,j) == 255 && img1.at<uchar>(i+1, j) == 0) {
        index = i;
        flag = 1;
      }
    }
  }
  return img1;
}

Mat remove_white_obstacles(Mat output, Mat input) {

  for(int  i = 0; i < input.rows; i++)
  {
    for (int j = 0; j < input.cols; j++)
    {
      int count = 0, n, check = 0;
      if(i<input.rows/4)
        n = 10;
      else if(i>=input.rows/4 && i < input.rows*2/4)
        n = 25;
      else if(i>=input.rows*2/4 && i < input.rows*3/4)
        n = 40;
      else if(i>=input.rows*3/4)
        n = 55;
      // if(i<input.rows/4)
      //   n = 20;
      // else if(i>=input.rows/4 && i < input.rows*2/5)
      //   n = 35;
      // else if(i>=input.rows*2/5 && i < input.rows*3/5)
      //   n = 55;
      // else if(i>=input.rows*3/5)
      //   n = 70;
      //n = (10 + ((50*(i+1))/input.rows));
      if ((abs(input.at<Vec3b>(i, j)[0] - input.at<Vec3b>(i, j)[1]) < input.at<Vec3b>(i, j)[0]*0.2 && abs(input.at<Vec3b>(i, j)[1] - input.at<Vec3b>(i, j)[2]) < input.at<Vec3b>(i, j)[1]*0.2 && abs(input.at<Vec3b>(i, j)[2] - input.at<Vec3b>(i, j)[0]) < input.at<Vec3b>(i, j)[2]*0.2) ||
          input.at<Vec3b>(i, j)[0] > 150 && input.at<Vec3b>(i, j)[1] > 150 && input.at<Vec3b>(i, j)[2]>150)
      {
        check = 1;
        for(int k = (-1)*(n/2); k < (n/2)+1; k++)
        {
          for(int l = (-1)*(n/2); l < (n/2)+1; l++)
          {
            if(i+k >=0 && i+k < input.rows && j+l >=0 && j+l <input.cols)
            {
              if(input.at<Vec3b>(i+k, j+l)[0] < input.at<Vec3b>(i+k, j+l)[1]*2/3 && input.at<Vec3b>(i+k, j+l)[2] < 1.3*input.at<Vec3b>(i+k, j+l)[1] && input.at<Vec3b>(i+k, j+l)[2] > 0.7*input.at<Vec3b>(i+k, j+l)[1])
                count++;
                if(count > n/3)
                  l = n;
                  k = n;
            }
          }
        }
      }
      if (count < n/3 && check == 1)
      {
        output.at<uchar>(i, j) = 255;
      }
      else
        {
        output.at<uchar>(i, j) = 0;
      }
      count = 0;
    }
  }
  return output;
}

Mat remove_obstacles(Mat img1, Mat processed, vector<Point> obs_lidar, bool vision, bool lidar) {

  //obstacles.clear();
   Mat img = img1.clone();
  // Mat chan[3];
  // Mat Simg17(img1.rows, img1.cols, CV_8UC1, Scalar(255));
  // split(img1, chan);
  // chan[0] = chan[0] - 70*(Simg17-chan[0])/255;
  // chan[1] = chan[1] + 30*(chan[1])/255;
  // merge(chan, 3, img1);

  if(vision)
  {
    Mat channels[3];
    Mat channels_w[3];
    Mat blue_obs(img1.rows, img1.cols, CV_8UC1, Scalar(0));
    Mat white_obs;
    Mat orange_obs(img1.rows, img1.cols, CV_8UC1, Scalar(0));

    vector<vector<Point> > contours_orange;
    vector<Vec4i> hierarchy_orange;
    vector<vector<Point> > contours_blue;
    vector<Vec4i> hierarchy_blue;
    vector<vector<Point> > contours_w;
    vector<Vec4i> hierarchy_w;

    /*  for bright orange obstacles*/
    split(img1, channels);
    orange_obs = channels[2] - channels[1];

    medianBlur(orange_obs, orange_obs, 9);



    /*  thresholding    */
    threshold(orange_obs, orange_obs, 50, 255, THRESH_BINARY);

    /*  morphology operations   */
    dilate(orange_obs, orange_obs, Mat(), Point(-1,-1), 7);
    for(int i = 0; i < img1.rows; i++)
    {
      for(int j = 0; j < img1.cols; j++)
      {
        if(img1.at<Vec3b>(i, j)[0] < img1.at<Vec3b>(i, j)[2]/2 && img1.at<Vec3b>(i, j)[1] < img1.at<Vec3b>(i, j)[2]/1.5)
        {
          orange_obs.at<uchar>(i, j) = 255;
        }
      }
    }
    for(int i = 0; i < img1.rows; i++)
    {
      for(int j = 0; j < img1.cols; j++)
      {
        if(img1.at<Vec3b>(i, j)[0]/2 > img1.at<Vec3b>(i, j)[1] || img1.at<Vec3b>(i, j)[0]/2 > img1.at<Vec3b>(i, j)[2])
        {
          blue_obs.at<uchar>(i, j) = 255;
        }
      }
    }
    // namedWindow("orange_obs", 0);
    // imshow("orange_obs", orange_obs);
    dilate(blue_obs, blue_obs, Mat(), Point(-1,-1), 2);
    
    // function joins the white spaces between detected obstacles
    orange_obs = orange_barrels(orange_obs);

    // finding contours
    findContours(orange_obs, contours_orange, hierarchy_orange, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    findContours(blue_obs, contours_blue, hierarchy_blue, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
     // storing bounding boxes
    vector<Rect> box_orange(contours_orange.size());
    vector<Rect> box_blue(contours_blue.size());
    
    Mat img1_obs(img1.rows, img1.cols, CV_8UC3, Scalar(0, 0, 0));

      // orange red 
    for (int i = 0; i < contours_orange.size(); i++) {
      box_orange[i] =  boundingRect(contours_orange[i]);

      if (box_orange[i].area() > 200 && ( (box_orange[i].br().y -box_orange[i].tl().y) < 10*(box_orange[i].br().x -box_orange[i].tl().x)) ) {
        drawContours(img1, contours_orange, i, Scalar(0, 0, 0), -1, 8, hierarchy_orange);
        drawContours(img1, contours_orange, i, Scalar(0, 0, 0), -1, 8, hierarchy_orange);
        drawContours(img1_obs, contours_orange, i, Scalar(255, 255, 255), -1, 8, hierarchy_orange);
      }
  }

  for (int i = 0; i < contours_blue.size(); i++) {
      box_blue[i] =  boundingRect(contours_blue[i]);

      if (box_blue[i].area() > 200 && (box_blue[i].br().y -box_blue[i].tl().y) < 10* (box_blue[i].br().x -box_blue[i].tl().x)) {
        drawContours(img1, contours_blue, i, Scalar(0, 0, 0), -1, 8, hierarchy_blue);
        drawContours(img1_obs, contours_blue, i, Scalar(255, 255, 255), -1, 8, hierarchy_blue);
      }
  }

  for(int i = 0; i < img1.rows; i++)
  {
    for(int j = 0; j < img1.cols; j++)
    {
      if(img1_obs.at<Vec3b>(i, j)[0] > 200 && img1_obs.at<Vec3b>(i, j)[1] > 200 && img1_obs.at<Vec3b>(i, j)[0] > 200)
      {
        processed.at<uchar>(i, j) = 0;
        
      }
    }
  }

    /*  bright blue */
  //   for (int i = 0; i < contours_blue.size(); i++) {
      // box_blue[i] =  boundingRect(contours_blue[i]);

  //     if (box_blue[i].area() > 8000) {
  //      for(int  k = box_blue[i].tl().x; k < box_blue[i].br().x; k++)
  //      {
  //        for (int l = box_blue[i].tl().y; l < box_blue[i].br().y; l++)
  //        {
  //          img1.at<Vec3b>(l, k) = {0, 0,0};
  //        }
  //      }
  //  }
  // }

  Mat wobs(img1.rows, img1.cols, CV_8UC1, Scalar(0));
  wobs = remove_white_obstacles(wobs, img1);
  erode(wobs, wobs, Mat(), Point(-1, -1), 5);

  findContours(wobs, contours_w, hierarchy_w, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  vector<Rect> box_w(contours_w.size());

  //cout << contours_w.size() << endl;

  for (int i = 0; i < contours_w.size(); i++) {
      box_w[i] =  boundingRect(contours_w[i]);
      // cout << box_w[i].area() << endl;
      if (box_w[i].area() > 200) {
        for(int  k = box_w[i].tl().x - 15; k < box_w[i].br().x + 15; k++)
        {
          for (int l = box_w[i].tl().y -15; l < box_w[i].br().y + 15; l++)
          {
            if(k < img1.cols && k >= 0 && l < img1.rows && l >= 0)
              {
                processed.at<uchar>(l, k) = 0;
                img1.at<Vec3b>(l, k) = {0, 0, 0};
              }
          }
        }
      }
      // for (int i = 0; i < contours_w.size(); i++) {
      // box_w[i] =  boundingRect(contours_w[i]);
      // // cout << box_w[i].area() << endl;
      // if (box_w[i].area() > 200) {
      //   for(int  k = box_w[i].tl().x - 15; k < box_w[i].br().x + 15; k++)
      //   {
      //     for (int l = box_w[i].tl().y -15; l < box_w[i].br().y + 15; l++)
      //     {
      //       if(k < img1.cols && k > -1 && l < img1.rows && l > -1)
      //         processed.at<Vec3b>(l, k) = {0, 0, 0};
      //     }
      //   }
      // }
  }
}


if(lidar)
{
  vector<vector<Point> > contours_lidar;
  vector<Vec4i> hierarchy_lidar;

  Mat lidar_img1(img1.rows, img1.cols, CV_8UC1, Scalar(0));
  for (int i = 0; i < obs_lidar.size(); ++i)
  {
    lidar_img1.at<uchar>(obs_lidar[i].y, obs_lidar[i].x) = 255;
  }

  float ratio = (float)lidar_stretch_ratio/40; 

  findContours(lidar_img1, contours_lidar, hierarchy_lidar, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  vector<Rect> box_lidar(contours_lidar.size());
  for (int i = 0; i < contours_lidar.size(); i++)
      {
        box_lidar[i] =  boundingRect(contours_lidar[i]);
        int x_new_tl = box_lidar[i].tl().x;
        int y_new_tl = box_lidar[i].tl().y;
        int x_new_br = box_lidar[i].br().x;
        int y_new_br = box_lidar[i].br().y;

        int u = (float)(x_new_tl-img1.cols/2)*((float)y_new_tl/img1.rows)*xshift;
        // cout << "\nvalue of u -> " << u << endl << endl;
        x_new_tl = x_new_tl + u;
        x_new_br = x_new_br + u;
      
        for(int k = (box_lidar[i].tl().y+ box_lidar[i].br().y)*ratio; k < box_lidar[i].br().y; k++)
        {
          for(int l = x_new_tl - lidar_stretch; l < x_new_br + lidar_stretch; l++)
          {
            
            
            if(k > -1 && k < img1.rows && l > -1 && l < img1.cols)
              {
                processed.at<uchar>(k, l) = 0;
                img1.at<Vec3b>(k, l) = {0, 0, 0};
              }
          } 
        }
      }
}
// for (int i = 0; i < contours_orange.size(); i++) {
// cout << contours_orange[i];
// }
namedWindow("ob", 0);
imshow("ob", img1);
waitKey(10);
return processed;   
}



#endif
