#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>


using namespace std;
using namespace cv;



// 27/05/2019 Homographic matrix
Mat h = (Mat_<double>(3,3) << 0.8465920206151547, 2.977162627611639, 26.77787294255721, -0.02373455534549772, 4.633696550480636, 20.92049287176745, -4.762781655265021e-05, 0.01232756896038383, 1);

Mat top_view(Mat img) {

    Mat top_view(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));

    warpPerspective(img,top_view,h,top_view.size());
    // Params: Src_img, dest_img, homographic_matrix,image_size
    return top_view;

}


