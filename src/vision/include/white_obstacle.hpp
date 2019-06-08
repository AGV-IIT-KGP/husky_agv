#ifndef WHITE_OBSTACLES
#define WHITE_OBSTACLES

#include <bits/stdc++.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

bool surrounding_check(Mat img, int i, int j, int p, Mat hsvimg)
{
	for(int m = 0; m < p-1; m++)
	{
		for(int n = 0; n < p-1; n++)
		{
			if(i+m >= 0 && i+m < img.rows && j+n >= 0 && j+n < img.cols)
			{
				if(m != 0 || n != 0)
				{
					if( (img.at<Vec3b>(i+m, j+n)[0] < img.at<Vec3b>(i+m, j+n)[1] && img.at<Vec3b>(i+m, j+n)[2] < img.at<Vec3b>(i+m, j+n)[1] && img.at<Vec3b>(i+m, j+n)[2] - img.at<Vec3b>(i+m, j+n)[0] > 40)
						||(img.at<Vec3b>(i+m, j+n)[0] < img.at<Vec3b>(i+m, j+n)[1] && img.at<Vec3b>(i+m, j+n)[0] < img.at<Vec3b>(i+m, j+n)[2]
						&& ((img.at<Vec3b>(i+m, j+n)[0] + img.at<Vec3b>(i+m, j+n)[2] - img.at<Vec3b>(i+m, j+n)[1]) < 40 && (img.at<Vec3b>(i+m, j+n)[0] + img.at<Vec3b>(i+m, j+n)[2] - img.at<Vec3b>(i+m, j+n)[1]) > 10
						||(hsvimg.at<Vec3b>(i+m, j+n)[0] > 30 && hsvimg.at<Vec3b>(i+m, j+n)[0] < 50 && hsvimg.at<Vec3b>(i+m, j+n)[1] > 120 && hsvimg.at<Vec3b>(i+m, j+n)[1] < 170 && hsvimg.at<Vec3b>(i+m, j+n)[2] > 90 && hsvimg.at<Vec3b>(i+m, j+n)[2] < 120))))
					{
						return false;
					}
				}
			}
		}
	}
	return true;
}

Mat black(Mat img, int i, int j, int p)
{
	for(int m = 0; m < p-1; m++)
	{
		for(int n = 0; n < p-1; n++)
		{
			if(i+m >= 0 && i+m < img.rows && j+n >= 0 && j+n < img.cols)
			{
				img.at<Vec3b>(i+m, j+n) = {0, 0, 0};
			}
		}
	}
	return img;
}
Mat White_obstacles(Mat img)
{
	Mat output[2];
	output[0] = img.clone();
	output[1] = img.clone();
	Mat hsvimg;
	cvtColor(img, hsvimg, COLOR_BGR2HSV);
	for(int i = 0; i < 30; i=i+5)
	{
		for(int j = 0; j < img.cols; j=j+5)
		{
			if(surrounding_check(output[0], i, j, 5, hsvimg))
			{
				output[0] = black(output[0], i, j, 7);
			}
		}
	}
	for(int i = 30; i < img.rows/3; i=i+15)
	{
		for(int j = 0; j < img.cols; j=j+15)
		{
			if(surrounding_check(output[0], i, j, 15, hsvimg))
			{
				output[0] = black(output[0], i, j, 20);
			}
		}
	}
	for(int i = (img.rows/3); i < img.rows*2/3; i=i+30)
	{
		for(int j = 0; j < img.cols; j=j+30)
		{
			if(surrounding_check(output[0], i, j, 40, hsvimg))
			{
				output[0] = black(output[0], i, j, 40);
			}
		}
	}
	for(int i = (img.rows*2/3); i < img.rows; i=i+40)
	{
		for(int j = 0; j < img.cols; j=j+40)
		{
			if(surrounding_check(output[0], i, j, 50, hsvimg))
			{
				output[0] = black(output[0], i, j, 60);
			}
		}
	}
	return output[0];
}

#endif