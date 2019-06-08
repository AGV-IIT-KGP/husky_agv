#ifndef PARAMS
#define PARAMS

/* config file */
bool is_debug = false;
bool is_important = true;
bool use_pothole = false;
bool use_ramp = false;
bool use_odom_lane_classify = false;
bool costmap_publish_ransac = false;

bool is_image_retrieved = false;
bool use_video = false;
char side;
bool used_hough = false;

int costmap_median_blur = 3;
int costmap_median_blur_no_mans_land = 7;

double wTh = 50; //set threshold for white color

int iteration = 300;  //define no of iteration, max dist squre of pt from our estimated Parabola2

int maxDist = 10; //define threshold distance to remove white pixel near lane1

int minLaneInlier = 65; // 2000 for night

int common_inliers_thresh = 10;

int minPointsForRANSAC = 700;

// float pixelsPerMetre = 37.7834;
float pixelsPerMetre = 25.06265;

float stepsize = 225; //3.5*pixelsPerMeter

int botlength = 30;
int botwidth = 30;

// float yshift = 0.33; // distance from first view point to lidar in metres
float xshift = 1;
float yshift = 0.33;

// float angleshift = 0.0524; // angle between camera and lidar axis in radians
float angleshift = 0.1;

float bins = 1080; // no of bins

int obstacleWidth = 30;

int medianBlurkernel = 7; //kernel size of medianBlur for cleaning intersectionImages

int neighbourhoodSize = 25; //neighbourhood size or block size for adaptive thresholding

int constantSubtracted = -45; //constant subtracted during adaptive thresholding

int grid_size = 3;

int grid_white_thresh = 3;

int brightestPixelThreshold = 100;

int lidar_stretch_ratio = 5;
int lidar_stretch = 5;

int inflation_r_waypt = 5;

float rscale = 2;

int hough_min_points = 100; 
int hough_min_line_length = 160;
int hough_max_line_gap = 50;

/* other global parameters */ 
#endif

