#ifndef WAY_POINT_GENERATION2
#define WAY_POINT_GENERATION2
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include "ransac_new_2.hpp"
#define dist_y2 4  //y for two lanes
#define dist_y_vert 3.5
#define dist_y_horiz 2.5
#define dist_wayp_lane 1



//check for y for 2 lqnes
using namespace std;
using namespace cv;
//parabola,img,x,y,radius

typedef struct Parabola2 {
    int numModel = 0;
    float a1 = 0.0;
    float c1 = 0.0;
    float a2 = 0.0;
    float b2 = 0.0;
    float b1 = 0.0;
    float c2 = 0.0;
} Parabola2;
int sgn(float x)
{
	if(x<0) return -1;
	else return 1;
}
float xc,yc;
bool is_vertical(float average_angle)
{
	if(average_angle>CV_PI*0.15||average_angle<CV_PI*0.85)
		return true;
	else 
		return false;

}

// Custom struct for storing x, y & yaw
struct NavPoint{
    int x;
    int y;
    float angle;
};

float avg_angle_rad(Parabola2 p,Mat img,bool is_left){
	
	if(!is_left){
		p.a1=p.a2;
		p.b1=p.b2;
		p.c1=p.c2;
        if(fabs(p.a1)<0.0001)
	    return CV_PI/2;
    }
	   if(fabs(p.a1)<0.000001)
		return -CV_PI/2;

	else
	{

		float slope_avg=0,slope;
		int k=0; 
        for(int i=0;i<img.rows;i++)
		{
			
			float x = p.a1*i*i + p.b1*i + p.c1;
			if(x>0&&x<img.cols)
			{
                slope=atan(1/(2*p.a1*i+p.b1));
                if(slope<0)
                    slope=slope+CV_PI;
				    slope_avg += slope;
                //cout << slope << " s"<< endl;
				k++;
			}
			
		
		}
        slope_avg/=k;
        if(slope_avg>CV_PI/2)
            slope_avg=slope_avg-CV_PI;
        return (slope_avg);
	}
}
NavPoint get_point(Mat img,Parabola2 para,float x,float y,float dist)
{	

	if(para.a1==0 && para.b1==0 && para.c1==0){
		float slope = 2*para.a2*y + para.b2;
		NavPoint p;
		
		p.x= (x-(1/(sqrt(1+slope*slope)))*dist);
		p.y=(y+(slope/(sqrt(1+slope*slope)))*dist);
		// cout<<"p.x "<<p.x<<"  p.y  "<<p.y<<"  x  "<<x<<"  y  "<<y<<endl;
        return p;

	}
	else
	{
		float slope = 2*para.a1*y + para.b1;
		NavPoint p;
		
		p.x= (x+(1/(sqrt(1+slope*slope)))*dist);
		p.y=(y-(slope/(sqrt(1+slope*slope)))*dist);
		return p;
	}
}


int checklane(int y,int x,Mat img,Parabola2 lanes)
{
    if(fabs(lanes.a1*y*y+lanes.b1*y+lanes.c1-x)< (30/4)) 
        return 1;

    if(fabs(lanes.a2*y*y+lanes.b2*y+lanes.c2-x)< (30/4))
        return 2;
    return 0;
}

bool isValid(Mat img, int i, int j) 
{
    if (i < 0 || i >= img.rows || j < 0 || j >= img.cols) {
        return false;
    }
    return true;
}

NavPoint getCoordinatesxy(Mat img,float average_angle_l,float average_angle_r,Parabola2 lanes)
{
    int i,j;
    float x_parabola,y_parabola;
    
    NavPoint pt;
   	//float angle_degree=average_angle*180/CV_PI;
    pt.x=img.cols/2;
    pt.y=img.rows-pixelsPerMetre*dist_y2;
    pt.angle=CV_PI/2;
	//BOTH LANES PRESENT
    if(lanes.numModel==2)   
    {
        // Intuition
        j=img.rows-pixelsPerMetre*dist_y2;

        /* Checking if given y co-ordinate's corresponding x co-ordinates 
         of the 2 curves are within image boudaries. */
        if((lanes.a1*j*j+lanes.b1*j+lanes.c1)>0 && (lanes.a2*j*j+lanes.b2*j+lanes.c2)<img.cols)
        {
            pt.y=j;
            //x= mean of x co-ordinates of 2 curves for the given y co-ordinate.
            pt.x= ((lanes.a1*j*j+lanes.b1*j+lanes.c1)+(lanes.a2*j*j+lanes.b2*j+lanes.c2))/2;
        }
        pt.angle=(average_angle_r+average_angle_l)/2;
        // cout<<"pt.x:"<<pt.x<<"  pt.y:"<<pt.y<<endl;
    }

    //SINGLE LANE PRESENT
    else if(lanes.numModel==1)
    {
    	if(lanes.a2==0)
    	{	
	    	if(is_vertical(average_angle_l))
	    	{
	    		y_parabola=img.rows-pixelsPerMetre*dist_y_vert;
	    		x_parabola=lanes.a1*y_parabola*y_parabola+lanes.b1*y_parabola+lanes.c1;
	    	}
	    	
    		else
    		{
    			y_parabola=img.rows-pixelsPerMetre*dist_y_horiz;
    			x_parabola=lanes.a1*y_parabola*y_parabola+lanes.b1*y_parabola+lanes.c1;
    		}
    		pt=get_point(img,lanes,x_parabola,y_parabola,dist_wayp_lane*pixelsPerMetre);
    		 pt.angle=average_angle_l;
    	}
    	if(lanes.a1==0)
    	{	
	    	if(is_vertical(average_angle_r))
	    	{
	    		y_parabola=img.rows-pixelsPerMetre*dist_y_vert;
	    		x_parabola=lanes.a2*y_parabola*y_parabola+lanes.b2*y_parabola+lanes.c2;
	    	}
	    	
    		else
    		{
    			y_parabola=img.rows-pixelsPerMetre*dist_y_horiz;
    			x_parabola=lanes.a2*pt.y*pt.y+lanes.b2*pt.y+lanes.c2;
    		}
    		pt=get_point(img,lanes,x_parabola,y_parabola,dist_wayp_lane*pixelsPerMetre);
    		 pt.angle=average_angle_r;
    	}
	}
    return pt;
}

NavPoint find_waypoint(Parabola2 lanes,Mat img)
{   
    /*Parabola2 lanes;
    int count_check;
    float a1 = lan.a1;
    float a2 = lan.a2;
    float c1 = lan.c1;
    float c2 = lan.c2;
    lanes.numModel=lan.numModel;

    //If no left lane
    if(a1==0)
    {
        lanes.a1 = 0;
        lanes.b1 = 0;
        lanes.c1 = 0;
    }
    else
    {
        //Equation at line 22    
        lanes.a1 = 1/a1;
        lanes.b1 = (-2*img.rows)/a1;
        lanes.c1 = (img.rows*img.rows + a1*c1)/a1;
        
        //if in case parabola is linearly fit into x= my + c
        if(fabs(lanes.a1)<0.00001)
            lanes.c1=c1;
    }

    //If no right lane
    if(a2==0)
    {
        //Equation at line 22
        lanes.a2 = 0;
        lanes.b2 = 0;
        lanes.c2 = 0;
    }
    else
    {
        lanes.a2 = 1/a2;
        lanes.b2 = (-2*img.rows)/a2;
        lanes.c2 = (img.rows*img.rows + a2*c2)/a2;
        
        //if in case parabola is linearly fit into x= my + c
        if(fabs(lanes.a2)<0.00001)
            lanes.c2=c2;
    }*/
    NavPoint way_point;
  	float average_angle_l=avg_angle_rad(lanes,img,true);
	float average_angle_r=avg_angle_rad(lanes,img,false);
	way_point= getCoordinatesxy(img,average_angle_l,average_angle_r,lanes) ;
    // cout<<"lft ang "<<average_angle_l<<" right ang"<<average_angle_r<<endl;
    // cout<<"before "<<way_point.angle<<endl;
    // cout<<"lanes.num"<<lanes.numModel<<endl;
    if(lanes.numModel!=2)
    {
        if(way_point.angle>0)
        {
        way_point.angle=CV_PI/2-way_point.angle;
        }
        else
        way_point.angle=-1*CV_PI/2-way_point.angle;
        // cout<<"after "<<way_point.angle<<endl;
        if(fabs(average_angle_l+CV_PI/2)<0.0001&&fabs(average_angle_r-CV_PI/2)<0.0001)
            way_point.angle=0;
        //way_point.angle=1.2;
       
        return way_point;
    }
    else if(lanes.numModel==2)
    {
        if(average_angle_l>0)
        {
        average_angle_l=CV_PI/2-average_angle_l;
        }
        else
        average_angle_l=-1*CV_PI/2-average_angle_l;

        if(average_angle_r>0)
        {
        average_angle_r=CV_PI/2-average_angle_r;
        }
        else
        average_angle_r=-1*CV_PI/2-average_angle_r;
        way_point.angle=(average_angle_l+ average_angle_r)/2;
        return way_point;

    }
    

}

Mat plotWaypoint(Mat costmap, NavPoint waypoint_image) 
{
    Point origin = Point(waypoint_image.x-1, waypoint_image.y-1);   
    // -1 added to avoid seg fault if img.rows/cols returned
    // cout<<"x"<<origin.x<<"         y"<<origin.y<<endl;
    float x = origin.x - 25*cos(CV_PI/2 - waypoint_image.angle);
    float y = origin.y - 25*sin(CV_PI/2 - waypoint_image.angle);
    Point dest = Point(x,y);
    
    //Drawing waypoint accordingly in the costmap image 
    circle(costmap, origin, 10, Scalar(0,0,255), -1, 8, 0);
    arrowedLine(costmap, origin, dest, Scalar(0,255,0), 3, 8, 0, 0.1);

    return costmap;
}

#endif
