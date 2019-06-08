#ifndef SEGMENTATION
#define SEGMENTATION

#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/core.hpp"
#include<iostream>
#include<bits/stdc++.h>
#include<unistd.h>

using namespace cv;
using namespace std;

/*
This header works with 3 different channel configurations 
    to be finally taken intersection of for effective 
    grass & noise removal.
*/

//Simple B channel
Mat blueChannelProcessing(Mat img)
{
    Mat channels[3];
    //Splitting the image into 3 Mats with 1 channel each
    split(img, channels);   
    Mat b = channels[0];

    GaussianBlur(b , b, Size( 9, 9), 0, 0);     //Based on observation
    adaptiveThreshold(b,b,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,neighbourhoodSize, constantSubtracted);
    medianBlur(b,b,medianBlurkernel);   //To remove salt & pepper noise

    return b;

}

//2B - G channel
Mat twob_gChannelProcessing(Mat img)
{    
    Mat channels[3];
    split(img, channels);
    Mat fin = 2*channels[0] - channels[1];

    adaptiveThreshold(fin, fin,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY, neighbourhoodSize, constantSubtracted);
    medianBlur(fin, fin,medianBlurkernel);

    return fin;
}

//2B - R channel
Mat twob_rChannelProcessing(Mat img)
{
    Mat channels[3];
    split(img, channels);
    Mat fin = 2*channels[0] - channels[2];

    //For storing square of channels
    Mat_<int> b2, g2, r2, mean2;

    multiply(channels[0], channels[0], b2); //multiplies matrices
    multiply(channels[1], channels[1], g2);
    multiply(channels[2], channels[2], r2);

    Mat_<int> mean = (Mat_<int>)((channels[0] + channels[1] + channels[2])/3);
    multiply(mean, mean, mean2);
    
    Mat_<int> zero_moment = (Mat_<int>)(b2 + g2 + r2)/3;
    
    //Variance is being used purely based on observation
    Mat_<float> variance = (Mat_<float>)(zero_moment - mean2);
    //Vairance= (Mean of sq.s) - (sq. of mean)

    Mat mask, result;

    threshold(variance, mask, 1500, 255, THRESH_BINARY);
    mask.convertTo(mask, CV_8U);

    //Taking intersection of original & thresholded variance intensities 
    bitwise_and(fin, mask, result);

    adaptiveThreshold(result,result,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,neighbourhoodSize, constantSubtracted);

    medianBlur(result, result, medianBlurkernel);

    return result;

}

#endif
