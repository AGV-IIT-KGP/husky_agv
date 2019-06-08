//This code returns a Mat a on which red color represents lanes whereas blue rectangle is bounding the ramp. 

#ifndef RAMPDETECTOR
#define RAMPDETECTOR


//NOTE threshold for CONTOUR AREA is 25000, for image size 640*480, it must be scaled according to size of image
#define contour_area_lim 20000
#define point_thresh 15
#define pitch_thresh 6.5

#include"opencv2/highgui/highgui.hpp"
#include"opencv2/core/core.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include<bits/stdc++.h>
#include<ros/ros.h>


using namespace cv;
using namespace std;
using namespace ros;

vector<vector<Point> > contours;

void ramp_chad_gaya(){

	if(fabs(pitch) > pitch_thresh){
		const_frames++;
	}
	else {
		const_frames = 0;
		ramp_detected = false;
	}
	
	if(const_frames>3){
		ramp_detected = true;
	}
	else{
		ramp_detected = false;
	}

}

float line_dist(float x1, float y1, float x2, float y2)
{
	return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}
    
int val(int x)
{
	if(x<0) return 0;
	if(x>255) return 255;
	else return x;
}
Mat rampdetector(Mat img1, Mat out, vector<Point> lidar_points)
{
	
	Mat in = img1.clone();
	//   Mat chan[3];
	//   Mat Simg17(in.rows, in.cols, CV_8UC1, Scalar(255));
	//   split(in, chan);
	//   chan[0] = chan[0] - 70*(Simg17-chan[0])/255;
	//   chan[1] = chan[1] + 30*(chan[1])/255;
	//   merge(chan, 3, in);
	Mat b2r(in.rows,in.cols,CV_8UC1,Scalar(0));
	Mat bnw(in.rows,in.cols,CV_8UC1,Scalar(0));
	Mat copy,c1,blak(in.rows,in.cols,CV_8UC1,Scalar(0));
	int i,j,k;

		
			// Mat out(a.rows,a.cols,CV_8UC3,Scalar(0, 0, 0));
			
			for(i=0;i<in.rows;i++)
			{
				for(j=0;j<in.cols;j++)
				{
					b2r.at<uchar>(i,j)=val(2*in.at<Vec3b>(i,j)[0]-in.at<Vec3b>(i,j)[1]);
				}

			}
			for(i=0;i<in.rows;i++)
			{
				for(j=0;j<in.cols;j++)
				{
					if(b2r.at<uchar>(i,j) > 60 && (in.at<Vec3b>(i,j)[0] + in.at<Vec3b>(i,j)[1] + in.at<Vec3b>(i,j)[2]) < 540)
						bnw.at<uchar>(i,j) = 255;
					else
						bnw.at<uchar>(i,j) = 0;
				}
			}
			imshow("bnw",bnw);
			erode(bnw, bnw, Mat(), Point(-1, -1), 2);
			imshow("b2r",b2r);
			imshow("bnw_erode",bnw);
			waitKey(20);
			copy=bnw.clone();
			c1=blak.clone();
			findContours(bnw,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
			bool ramp_flag = false;
			/*cout<<contours.size()<<endl;*/
			vector<Vec4i> hierarchy;
		    vector<Rect> boundRect( contours.size() );
		    vector<vector<Point>> contours_poly( contours.size() );
				for(i=0,j=-1;i<contours.size();i++)
				{
					/*cout<<"yes"<<endl;*/	
					approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
					/*cout<<"no"<<endl;*/
					boundRect[i] = boundingRect( Mat(contours_poly[i]) );
					if(contourArea(contours[i])>contour_area_lim)
					{
						if(j == -1)
						{	
							j = i;
							ramp_flag = true;
							cout<<"efevbkjbedkbce"<<endl;
						}
						else if(contourArea(contours[i]) > contourArea(contours[j]))
						{
							j = i;
						}
					}
				}
				if(!ramp_flag)
				{
					ramp_detected = false;
					return out;
				}

				int count_lidar = 20;
				int points_intersecting = 0;
				Mat lidar_pts_ramp(out.rows, out.cols, CV_8UC1, Scalar(0));
				Point tl, br;
				tl.x = boundRect[j].tl().x;
				tl.y = boundRect[j].tl().y;
				br.x = boundRect[j].br().x;
				br.y = boundRect[j].br().y;
				if(ramp_flag)
				{
					for(int i = 0; i < lidar_points.size(); i++)
					{
						if(lidar_points[i].x >= tl.x && lidar_points[i].x <= tl.x && lidar_points[i].y >= (br.y - 0.7*(br.y-tl.y)) && lidar_points[i].y <= br.y)
						{
							lidar_pts_ramp.at<uchar>(lidar_points[i].y, lidar_points[i].x) = 255;
							points_intersecting++;
						}
					}
				}

				// float distance = 0;
				// int max_l = -1;
				// vector<Vec4i> lines;
			 //    HoughLinesP(lidar_pts_ramp, lines, 1, CV_PI/180, 80, 30, 10 );
			 //    for( size_t i = 0; i < lines.size(); i++ )
			 //    {
			 //        if(line_dist(lines[i][0], lines[i][1], lines[i][2], lines[i][3]) > distance)
			 //        {
			 //        	max_l = i;
			 //        	distance = line_dist(lines[i][0], lines[i][1], lines[i][2], lines[i][3]);
			 //        }    
			 //    }
			    // if(distance >= (0.7*(br.x - tl.x)))
			    // {
			    // 	ramp_detected = false;
			    // 	return out;
			    // }
			    if(points_intersecting > point_thresh)
			    {
			    	ramp_detected = false;
			    	return out;
			    }
				

				if(j!=-1)
				{
					// drawContours(c1,contours,j,125,1,8);
					// rectangle( out, boundRect[j].tl(), boundRect[j].br(), Scalar(255,0,0), 2, 8, 0 );
					for(int k = boundRect[j].tl().y; k < boundRect[j].br().y; k++)
					{
						for(int l = boundRect[j].tl().x; l < boundRect[j].br().x; l++)
						{
							if(bnw.at<uchar>(i,k)==0&&(in.at<Vec3b>(i,k)[0]+in.at<Vec3b>(i,k)[0]+in.at<Vec3b>(i,k)[0])>600)
								out.at<uchar>(k, l) = 255;
							else
								out.at<uchar>(k, l) = 0;								
						}
					}
				}
				// for(k=boundRect[j].tl().y; <boundRect[j].br().y;i++)
				// {
				// 	for(k=boundRect[j].tl().x;k<boundRect[j].br().x;k++)
				// 	{
				// 		if(bnw.at<uchar>(i,k)==0&&(a.at<Vec3b>(i,k)[0]+a.at<Vec3b>(i,k)[0]+a.at<Vec3b>(i,k)[0])>600)
				// 			out.at<uchar>(i,k)=255;
				// 	}
				// }		
				

			
			//imshow("W",out);


			/*imshow("W1",b2r);
			imshow("W2",copy);*/
			// waitKey(20);
			ramp_detected = true;
			return out;             
}


#endif
