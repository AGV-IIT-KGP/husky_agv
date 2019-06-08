#ifndef LIDAR_PLOT
#define LIDAR_PLOT

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace ros;
using namespace std;
using namespace cv;

Mat obstaclePlot;

/*
This header plots inflated LIDAR obstacles in front view
	& stores them in a vector of points
*/

vector<Point> lidar_plot(sensor_msgs::LaserScan scan, Mat h_, int rows, int cols)
{
	vector<Point> vect;		//For storing obstacl points
	Mat img(rows, cols, CV_8UC3, Scalar(0,0,0));
	Mat temp(rows,cols,CV_8UC1,Scalar(0));
	double minangle = scan.angle_min;
	
	double step = scan.angle_increment;
	int iters = (scan.angle_max - scan.angle_min) / step;	//iterations

	for (int i=0;i<=iters;i++)
	{				
//NOTE: All this plotting is done in top view

		double angle = minangle + i*step - angleshift;
		// angleshift: angle between camera and lidar axis in radians
		
		//Checking if the scan is within the angle & distance limits 
		if (scan.ranges[i] <= scan.range_max && angle > -CV_PI/2 && angle < CV_PI/2 && scan.ranges[i] >= scan.range_min)
		{
			float a = scan.ranges[i] * sin(angle); //x co-oridnate
			float b = scan.ranges[i] * cos(angle) - yshift ; //y co-oridnate
			
			//Converting to pixel co-ordinates
			a *= pixelsPerMetre;
			b *= pixelsPerMetre;

			//Shifting origin from bottom-centre(LIDAR frame) to top-Right
/*NOTE: top-right since the LIDAR x & y axes are rotated by 90 degrees(CCW).*/
			int x = cols/2 + a;
			int y = rows - b;
			
			//Finally shifting origin to top-left(Image frame)
			x = cols - x;

			//If the co-ordinates are inside the image
			if (x >=0 && x < img.cols && y>=0 && y<img.rows)
			{
				//Setting the image co-ordinates to green in original image
				img.at<Vec3b>(y,x)[0] = 0;
				img.at<Vec3b>(y,x)[1] = 255;
				img.at<Vec3b>(y,x)[2] = 0;

				//Plotting obstacles in a B/W image
				temp.at<uchar>(y,x)=255;
				//circle(img, Point(x,y), 4, Scalar(0,255,0)), 2;
			}
		}
	}
	obstaclePlot=temp.clone();
	
	// Inflation of obstacles is done by plotting circles around obstacle pts.
	for(int i=0;i<img.rows;i++)
		for(int j=0;j<img.cols;j++)
			if(temp.at<uchar>(i,j)==255)
			{
				Point2f center(j,i);
				circle(obstaclePlot, center, inflation_r_waypt, Scalar(255), -1);
			}
	
	//Changing to front view (NOTE: WARP_INVERSE_MAP)
	warpPerspective(img, img, h_, img.size(), WARP_INVERSE_MAP);
	
	//Pushing inflated obstacles in a vector
	for (int i=0;i<rows;i++)
	{
		for (int j=0;j<cols;j++)
		{
			if(img.at<Vec3b>(i,j)[1] == 255) 
			{
				vect.push_back(Point(j,i));
			}
		}
	}
	return vect;
}

#endif
