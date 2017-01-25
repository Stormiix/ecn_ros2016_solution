#ifndef COLORDETECTOR_H
#define COLORDETECTOR_H

#include <opencv2/core/core.hpp>

class ColorDetector
{
public:
    ColorDetector(int r, int g, int b);

    inline double X() {return X_;}
    inline double Z() {return Z_;}

    void process(const cv::Mat &_im, cv::Mat &_im_processed);

protected:
    cv::Scalar lower1, lower2, upper1, upper2;
    double ipx, ipy, u0, v0;
    double X_, Z_;
    cv::Mat img, seg1, seg2;
    bool two_segmentations;

};

#endif // COLORDETECTOR_H
