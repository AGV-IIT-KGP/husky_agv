/*
NOTE: For debugging couts:
 * Open rqt_reconfigure
 * Set is_debug to true
 */
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <vision/TutorialsConfig.h>
#include <time.h>
#include <std_msgs/Bool.h>
#include<std_msgs/Float64.h>


//Custom Header files
#include <params.hpp>
#include <matrixTransformation.hpp>
#include <lidar_new.hpp>
#include <lidar_plot.hpp>
#include <waypoint_generator_new2.hpp>
#include <ransac_new_2.hpp>
#include <ransac_second.hpp>

#include <lane_segmentation.hpp>
#include <lane_laser_scan.hpp>

#include <find_pothole.hpp>
#include <hough.hpp>
#include <obstacle_det_vision_lidar.hpp>    

using namespace std;
using namespace cv;
using namespace ros;

bool ramp_detected = false;
int const_frames = 0;
float pitch;

#include <rampdetector.hpp>

//Dynamic Reconfigure callback function
// Link: http://wiki.ros.org/dynamic_reconfigure
void callback(node::TutorialsConfig &config, uint32_t level)
{
    is_debug = config.is_debug;
    is_important = config.is_important;

    wTh = config.wTh;
    iteration = config.iteration;
    maxDist = config.maxDist;
    minLaneInlier = config.minLaneInlier;
    minPointsForRANSAC = config.minPointsForRANSAC;
    grid_size = config.grid_size;
    constantSubtracted=config.constantSubtracted;
    grid_white_thresh=config.grid_white_thresh;
    common_inliers_thresh = config.common_inliers_thresh;

    pixelsPerMetre = config.pixelsPerMetre;
    stepsize = config.stepsize;

    botlength = config.botlength;
    botwidth = config.botwidth;

    xshift = config.xshift;
    yshift = config.yshift;
    angleshift = config.angleshift;
    bins = config.bins;

    obstacleWidth = config.obstacleWidth;

    medianBlurkernel = config.medianBlurkernel;
    neighbourhoodSize = config.neighbourhoodSize;

    lidar_stretch_ratio = config.lidar_stretch_ratio;
    lidar_stretch = config.lidar_stretch;
    inflation_r_waypt=config.inflation_r_waypt;


    rscale = config.rscale;

    hough_min_points = config.hough_min_points; 
    hough_min_line_length = config.hough_min_line_length;
    hough_max_line_gap = config.hough_max_line_gap;

    use_pothole = config.use_pothole;
    use_ramp = config.use_ramp;

    use_odom_lane_classify = config.use_odom_lane_classify;

    costmap_publish_ransac = config.costmap_publish_ransac;

    costmap_median_blur = config.costmap_median_blur;
    costmap_median_blur_no_mans_land = config.costmap_median_blur_no_mans_land;
}

Publisher lanes2Costmap_publisher;  //For putting lanes in costmap
Publisher pot2staticCostmap_publisher;  //For putting potholes in costmap
Mat frame_orig;

//Parameter for GPS switching. Set to false when GPS waypoint starts
bool use_vision_global = true;
bool is_debug_used = false;

//For converting & resizing pointgrey camera node data to image
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);    
	is_image_retrieved=true;
    }
    catch (cv_bridge::Exception& e)
    {
	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    frame_orig = (cv_ptr->image);

    //Resizing Mat to 1/16th area so as to reduce computation
    resize(frame_orig, frame_orig, Size(frame_orig.cols/4, frame_orig.rows/4));
}	

void use_vision_callback(const std_msgs::Bool::ConstPtr& msg) {
    use_vision_global = msg->data;
}

void odomCallBack(const std_msgs::Float64::ConstPtr& msg)
{
    pitch = msg->data;
}

int main(int argc, char **argv)
{ 
    init(argc,argv,"master");
    NodeHandle n;
    image_transport::ImageTransport it(n);

    //Taken from dynamic reconfigure tutorials
    dynamic_reconfigure::Server<node::TutorialsConfig> server;
    dynamic_reconfigure::Server<node::TutorialsConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    Subscriber lidar_subsriber, orientation;
    lidar_subsriber = n.subscribe("/scan", 2, &laserscan);
    image_transport::Subscriber sub = it.subscribe("/camera/image_color", 2, imageCb);  //NOTE Topic
    Publisher waypoint_publisher = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",2);
    lanes2Costmap_publisher = n.advertise<sensor_msgs::LaserScan>("/lanes", 2);       //declared globally
    pot2staticCostmap_publisher = n.advertise<sensor_msgs::LaserScan>("/nav_msgs/OccupancyGrid", 2);       //declared globally
    Subscriber use_vision_subscriber = n.subscribe("/use_vision", 1, &use_vision_callback);
    orientation = n.subscribe("/vn_ins/pitch",100,odomCallBack);

    int waypoint_count = 0;

    Parabola lanes;		//For Ransac implementation(it is a structure)
    lanes.a1=0;         //For definition lookup ransac_new_2.hpp
    lanes.c1=0;
    lanes.a2=0;
    lanes.c2=0;
    lanes.numModel=0;

    Parabola2 lanes_2;
    lanes_2.a1=0;
    lanes_2.b1=0;
    lanes_2.c1=0;
    lanes_2.a2=0;
    lanes_2.b2=0;
    lanes_2.c2=0;
    lanes_2.numModel=0;

    Parabola previous;
    previous.a1=0;         //For definition lookup ransac_new_2.hpp
    previous.c1=0;
    previous.a2=0;
    previous.c2=0;
    previous.numModel=0;


    //Used for finding FPS
    clock_t tic,toc;

    while(ros::ok())
    {

	/* Start time for measuring FPS */
	tic=clock();

	/* Checking if image has been received */
	while ((!is_image_retrieved || !is_laserscan_retrieved) && ros::ok())
	{
	    if (is_debug) {
		cout << "Image or Lidar data not retrieved" << endl;
	    }
	    spinOnce();
	}
	/* Converting frame to top view */
	Mat frame_topview = top_view(frame_orig);

	/* For detecting potholes */
	Mat bw;    
	if (use_pothole == true) {        
	    cvtColor(frame_orig,bw,cv::COLOR_RGB2GRAY);
	    namedWindow("bw", WINDOW_NORMAL);
	    imshow("bw", bw);
	}

	/* 2b-r preprocessing */
	Mat twob_r = twob_rChannelProcessing(frame_orig);
	if (is_debug) {
	    is_debug_used = true;
	    namedWindow("2b-r", WINDOW_NORMAL);
	    imshow("2b-r", twob_r);
	}

	/* 2b-g preprocessing */
	Mat twob_g = twob_gChannelProcessing(frame_orig);
	if (is_debug) {
	    namedWindow("2b-g", WINDOW_NORMAL);
	    imshow("2b-g", twob_g);
	}

	/* blue channel preprocessing */
	Mat b = blueChannelProcessing(frame_orig);
	if (is_debug) {
	    namedWindow("b", WINDOW_NORMAL);
	    imshow("b", b);
	}

	/* Taking intersection of all lane filters */ 
	Mat intersectionImages;
	bitwise_and(twob_r, twob_g, intersectionImages);
	bitwise_and(intersectionImages, b, intersectionImages);

	if(is_debug){
	    namedWindow("intersectionImages", WINDOW_NORMAL);
	    imshow("intersectionImages", intersectionImages);
	}



	/* Plotting obstacles by Lidar */
	Mat remove_obstacles_image = frame_orig.clone();
	vector<Point> obs_by_lidar = lidar_plot(lidar_scan, h, frame_orig.rows, frame_orig.cols);

	Mat chan[3];
	Mat green_grass(remove_obstacles_image.rows, remove_obstacles_image.cols, CV_8UC1, Scalar(255));
	split(remove_obstacles_image, chan);
	chan[0] = chan[0] - 70*(green_grass-chan[0])/255;
	chan[1] = chan[1] + 30*(chan[1])/255;
	merge(chan, 3, remove_obstacles_image);

	/* Ramp detection */
	intersectionImages = remove_obstacles(remove_obstacles_image, intersectionImages, obs_by_lidar, !use_ramp, true); 

	if(is_debug || is_important){
	    namedWindow("Obs_removed", WINDOW_NORMAL);
	    imshow("Obs_removed", intersectionImages);
	}

	/* Fitting a hough line for horizontal lanes */
	Mat hough_image(intersectionImages.rows,intersectionImages.cols, CV_8UC1, Scalar(0));

	if (is_important) {
	    namedWindow("hough", WINDOW_NORMAL);
	}

	if(lanes_2.numModel == 1 && use_vision_global == true)
	{

	    cout << "Hough line detected" << endl;

	    //Checking if lane is left or right
	    if(lanes_2.a1 == 0 && lanes.c1 == 0 && lanes_2.b1==0) {
		side = 'r';
	    }

	    else if(lanes_2.a2 == 0 && lanes_2.c2 == 0 && lanes_2.b2==0) {
		side = 'l';
	    }

	    if(check_whether_hough(hough_image,intersectionImages))
	    {
		if(is_debug) {cout << "Hough Code Initiated" << endl;}
		used_hough = true;

		NavPoint waypoint_image = waypoint_for_hough(hough_image, side, theta);
		//theta is globally declared in hough.hpp

		intersectionImages = plotWaypoint(hough_image, waypoint_image);

		// Giving waypt.'s x, y & z co-ordinates(here z= 1)
		Mat waypt = (Mat_<double>(3,1) << waypoint_image.x , waypoint_image.y , 1);
		//Converting waypt. in top view
		Mat waypt_top = h*waypt;    //NOTE that the h is PRE-multiplied 
		// & it takes a 3-D pt.

		//Actual 2-D x= x/z & y= y/z 
		double x_top = waypt_top.at<double>(0,0)/waypt_top.at<double>(2,0);
		double y_top = waypt_top.at<double>(1,0)/waypt_top.at<double>(2,0);

		//waypoint transform for hough line
		waypoint_image.x=x_top;
		waypoint_image.y=y_top;

		sensor_msgs::LaserScan lane;

		Mat hough_published = intersectionImages.clone();
		// medianBlur(hough_published, hough_published, 3);
		lane = laneLaser(top_view(hough_published));
		lanes2Costmap_publisher.publish(lane);  


		//transforming waypoint to ros convention (x forward, y left, angle from x and positive clockwise) (in metres)
		/* Waypoint from hough */
		geometry_msgs::PoseStamped waypoint_bot;
		waypoint_bot.header.frame_id = "base_link";
		waypoint_bot.header.stamp = ros::Time::now();   //Important

		//changing waypoint position from LIDAR to image frame (conversion of y makes it clear)
		waypoint_bot.pose.position.x = (intersectionImages.rows - waypoint_image.y)/pixelsPerMetre;
		waypoint_bot.pose.position.y = (intersectionImages.cols/2 - waypoint_image.x)/pixelsPerMetre;
		waypoint_bot.pose.position.z = 0;
		float theta = (waypoint_image.angle);
		imshow("hough", hough_image);

		//converting to Quaternion from Yaw 
		tf::Quaternion frame_qt = tf::createQuaternionFromYaw(theta);
		waypoint_bot.pose.orientation.x = frame_qt.x();
		waypoint_bot.pose.orientation.y = frame_qt.y();
		waypoint_bot.pose.orientation.z = frame_qt.z();
		waypoint_bot.pose.orientation.w = frame_qt.w();

		waypoint_publisher.publish(waypoint_bot);

		waitKey(100);
		spinOnce();
		continue;

	    }
	}

	if (is_important) {
	    imshow("hough", hough_image);
	}

	/* Converting costmap (contains intersection images top view) */
	Mat costmap(intersectionImages.rows,intersectionImages.cols,CV_8UC1,Scalar(0));
	costmap=top_view(intersectionImages);

	/* Add the pothole to the costmap if pothole is being used */
	if (use_pothole) {
	    costmap=find_pothole(top_view(bw),costmap);
	}

	if(is_debug)
	{
	    namedWindow("costmap_orig",0);
	    imshow("costmap_orig",costmap);
	}

	/* Apply medianBlur to the costmap copy (costmap_published) ONLY for publishing to the rviz */

	/* For GPS switching */ 
	if (use_vision_global == false || costmap_publish_ransac == false) {
	    Mat costmap_published = costmap.clone();
	    
	    if (use_vision_global == true) {
	    	medianBlur(costmap_published, costmap_published, costmap_median_blur);
		}
		else {
	    	medianBlur(costmap_published, costmap_published, costmap_median_blur_no_mans_land);
		}

	    if (is_debug || is_important) {
		namedWindow("costmap_published", 0);
		imshow("costmap_published", costmap_published);
	    }

	    /* Publishing laserscan of costmap */
	    sensor_msgs::LaserScan lane;
	    lane = laneLaser(costmap_published);
	    lanes2Costmap_publisher.publish(lane);

	    if (use_vision_global == false) {
		cout << "GPS waypoints are being used" << endl;
		spinOnce();
		continue;
	    }
	}

	/* Fitting Ransac */
	// ptArray1 is the array of all points on which ransac will be fit (contents of grid image)
	std::vector<Point> ptArray1;
	previous = lanes;
	lanes = getRansacModel(intersectionImages, previous, ptArray1);

	/* Classification of left and right lanes */
	if(use_odom_lane_classify == false) {
	    lanes=classify_lanes(intersectionImages, lanes, previous);
	}
	else 
	{
	    lanes=classify_lanes_odom(intersectionImages, lanes, previous, ptArray1);
	}


	//Uncomment the next line to disallow sudden changes in lane params
	// lanes = no_sudden_change(lanes, intersectionImages, previous);

	/* Drawing the lanes of front view on original image */
	Mat fitLanes=frame_orig.clone();


	fitLanes=drawLanes(fitLanes,lanes);

	if(is_important || is_debug)
	{
	    namedWindow("front_view_ransac",0);
	    imshow("front_view_ransac",fitLanes);
	}

	/* Fitting Ransac in top view */
	Mat lanes_front_view(frame_orig.rows, frame_orig.cols, CV_8UC3, Scalar(0,0,0));
	lanes_front_view = drawLanes(lanes_front_view, lanes);
	Mat top_ransac = top_view(lanes_front_view);
	lanes_2 = getRansacModel_2(top_ransac,lanes_2);

	/* Drawing lanes in top view */
	frame_topview = drawLanes_top(frame_topview,lanes_2);

	if (costmap_publish_ransac == true) {
	    Mat costmap_ransac(costmap.rows, costmap.cols, CV_8UC1, Scalar(0));
	    costmap_ransac = drawLanes_top(costmap_ransac, lanes_2);
	    sensor_msgs::LaserScan lane;
	    lane = laneLaser(costmap_ransac);
	    lanes2Costmap_publisher.publish(lane);
	}

	if(is_important || is_debug)
	{
	    namedWindow("ransac_topview",0);
	    imshow("ransac_topview",frame_topview);
	}


	/* Finds the waypoint from the 
	/* Returns waypoint assuming origin at bottom left of image (in pixel coordinates)
	& orientation in radians. */ 
	NavPoint waypoint_image = find_waypoint(lanes_2,costmap); 
	frame_topview = plotWaypoint(frame_topview, waypoint_image);

	if (is_important || is_debug) {
	    namedWindow("waypoint", WINDOW_NORMAL);
	    imshow("waypoint", frame_topview);
	}

	/* Publishing waypoint */
	//changing waypoint position from LIDAR to image frame (conversion of y makes it clear)
	geometry_msgs::PoseStamped waypoint_bot;
	waypoint_bot.header.frame_id = "base_link";
	waypoint_bot.header.stamp = ros::Time::now();
	waypoint_bot.pose.position.x = (costmap.rows - waypoint_image.y)/pixelsPerMetre;
	waypoint_bot.pose.position.y = (costmap.cols/2 - waypoint_image.x)/pixelsPerMetre;
	waypoint_bot.pose.position.z = 0;
	float theta = (waypoint_image.angle);

	/* converting to Quaternion from Yaw */
	tf::Quaternion frame_qt = tf::createQuaternionFromYaw(theta);
	waypoint_bot.pose.orientation.x = frame_qt.x();
	waypoint_bot.pose.orientation.y = frame_qt.y();
	waypoint_bot.pose.orientation.z = frame_qt.z();
	waypoint_bot.pose.orientation.w = frame_qt.w();

	//Publishing waypoint
	if (waypoint_count == 3) {
	    waypoint_publisher.publish(waypoint_bot);
	    waypoint_count = 0;        
	}
	waypoint_count++;


	if (is_debug == false && is_debug_used == true) {
	    destroyWindow("2b-g");
	    destroyWindow("2b-r");
	    destroyWindow("b");
	    destroyWindow("costmap_orig");
	    destroyWindow("intersectionImages");
	    is_debug_used = false;
	}


	if (is_debug == false &&  is_important == false) {
	    destroyAllWindows();
	}


	waitKey(400);
	is_image_retrieved = false;
	is_laserscan_retrieved = false;
	used_hough = false;

	toc=clock();

	cout << "FPS: " << CLOCKS_PER_SEC/(toc-tic) << endl;
	spinOnce();
    }

    destroyAllWindows();

    return 0;
}

