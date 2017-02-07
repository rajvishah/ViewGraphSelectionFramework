#include "Util.h"

float Median(std::vector<float>* data) {
  int n = data->size();
  std::vector<float>::iterator mid_point = data->begin() + n / 2;
  std::nth_element(data->begin(), mid_point, data->end());
  return *mid_point;
}

bool FeatureConvexHull::findConvexHullContour(vector<cv::Point2f>& points) {
  vector< cv::Point > convexHull;
  double epsilon = 0.001;
  vector<cv::Point2i> pointSetInt;
  cv::Mat(points).copyTo(pointSetInt);
  cv::convexHull(cv::Mat(pointSetInt),convexHull,false);
  cv::approxPolyDP(cv::Mat(convexHull), contour, epsilon, true);
  area = fabs(cv::contourArea(cv::Mat(contour)));
  return true;
}
