#ifndef UTIL_H
#define UTIL_H 
#include "defs.h"

float Median(std::vector<float>* data);

class FeatureConvexHull {
  public:
    vector < cv::Point > contour;
    double area;

    bool findConvexHullContour(vector<cv::Point2f>& points);
};
#endif /* UTIL_H */
