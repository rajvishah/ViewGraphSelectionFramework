#ifndef __DRAWINGS_H
#define __DRAWINGS_H
#include "defs.h"

void DrawPairsWithPoints(const string& imageName1, const string& imageName2, vector< cv::Point2f >& pointSet1, vector<cv::Point2f>& pointSet2, double score, string outputFile);


void DrawOutliers(const string& imageName1, vector< cv::Point2f > allPoints,
    vector<cv::Point2f> outlierCoords, string winName);

void DrawScoreBoard( vector<string>& scoreNames, vector<double>& scoreValues);
void DrawTupleWithPoints(const string& imageName1, const string& imageName2, 
    const string& imageName3, vector< cv::Point2f >& pointSet11, 
    vector<cv::Point2f>& pointSet12, 
    vector< cv::Point2f>& pointSet21,
    vector<cv::Point2f>& pointSet22, 
    vector<cv::Point2f>& pointSet31, 
    vector<cv::Point2f>& pointSet32, 
    double score, string outputFile);
#endif //__DRAWINGS_H
