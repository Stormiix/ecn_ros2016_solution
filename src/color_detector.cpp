#include <ecn_ros2016/color_detector.h>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>

using std::vector;

ColorDetector::ColorDetector(int r, int g, int b)
{
    // cam param
    u0 = 320;
    v0 = 240;
    ipx = tan(30*M_PI/180)/u0;
    ipy = tan(30*M_PI/180)/v0;

    // convert color to HSV
    const float cmax = std::max(r, std::max(g,b));
    const float cmin = std::min(r, std::min(g,b));
    const float d = cmax - cmin;
    int s = 0;
    if(cmax)
        s = 255*d/cmax;
    int v = cmax;

    int h = 0;
    if(d)
    {
        if(cmax == r)
            h = 30*(fmod((g-b)/d,6));
        else if(cmax == g)
            h = 30*((b-r)/d + 2);
        else
            h = 30*((r-g)/d + 4);
    }

    // build inRange bounds
    const int hthr = 5;
    const int sthr = 50;
    const int vthr = 30;


    // base segmentation
    two_segmentations = false;
    lower1 = cv::Scalar(std::max(h-hthr,0), std::max(s-sthr,0), std::min(v-vthr,0));
    upper1 = cv::Scalar(std::min(h+hthr,179), std::min(s+sthr,255), std::min(v+vthr,255));

    // other segmentation for h
    if(h < hthr)
    {
        two_segmentations = true;
        lower2 = cv::Scalar(179+h-hthr, std::max(s-sthr,0), std::max(v-vthr,0));
        upper2 = cv::Scalar(179, std::min(s+sthr,255), std::min(v+vthr,255));
    }
    else if(h+hthr > 179)
    {
        two_segmentations = true;
        lower2 = cv::Scalar(0, std::max(s+sthr,0), std::max(v+vthr,0));
        upper2 = cv::Scalar(h+hthr-179, std::min(s+sthr,255), std::min(v+vthr,255));
    }
}


void ColorDetector::process(const cv::Mat &_im, cv::Mat &_im_processed)
{
    _im.copyTo(_im_processed);
    cv::cvtColor(_im, img, cv::COLOR_BGR2HSV);

    // initial segmentation
    cv::inRange(img, lower1, upper1, seg1);

    // if needed
    if(two_segmentations)
    {
        cv::inRange(img, lower2, upper2, seg2);
        seg1 += seg2;
    }

    cv::GaussianBlur(seg1, seg1, cv::Size(5,5), 2);

    // edge detection
    cv::Canny(seg1, img, 20, 150);

    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours( img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    // sort contours from largest to smallest
    std::sort(contours.begin(), contours.end(),
              [](const vector<cv::Point> &c1, const vector<cv::Point> &c2)
    {return cv::contourArea(c1) > cv::contourArea(c2);});

    if(contours.size() > 0)
    {
        double x,a;
        cv::Moments m = cv::moments(contours[0], false);
        a = m.m00*ipx*ipy;
        x = (m.m10/m.m00 - u0)* ipx;

        Z_ = 0.106/sqrt(a);
        X_ = x*Z_;

        cv::drawContours(_im_processed, contours, 0, cv::Scalar(255,0,255), 2);
    }
}
