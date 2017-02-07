#include "Drawings.h"

void DrawScoreBoard( vector<string>& scoreNames, vector<double>& scoreValues) {
  cv::Mat canvas( 1000, 800, CV_8UC3, cv::Scalar(255,255,255));
  for(int i=0; i < scoreNames.size(); i++) {
 
    stringstream ss;
    ss << scoreNames[i] << " :  " << scoreValues[i];

    cv::putText(canvas, ss.str(), cv::Point2f(10, (i+1)*80), 
        cv::FONT_HERSHEY_PLAIN, 4, cvScalar(255,0,0), 2, CV_AA); 
  }
 cv::namedWindow("scores", CV_WINDOW_NORMAL);
cv::imshow("scores", canvas);
}

void DrawTupleWithPoints(const string& imageName1, const string& imageName2, 
    const string& imageName3, vector< cv::Point2f >& pointSet11, 
    vector<cv::Point2f>& pointSet12, 
    vector< cv::Point2f>& pointSet22,
    vector<cv::Point2f>& pointSet23, 
    vector<cv::Point2f>& pointSet33, 
    vector<cv::Point2f>& pointSet31, 
    double score, string outputFile) {

  cv::Scalar color1(255,0,0);
  cv::Scalar color2(0,255,0);
  cv::Scalar color3(0,0,255);

  cv::Mat image1 = cv::imread( imageName1, CV_LOAD_IMAGE_COLOR);
  cv::Mat image2 = cv::imread( imageName2, CV_LOAD_IMAGE_COLOR);
  cv::Mat image3 = cv::imread( imageName3, CV_LOAD_IMAGE_COLOR);

  cv::Size sz = image1.size();
  int qWidth = sz.width;
  int qHeight = sz.height;

  cv::Size sz1 = image2.size();
  int rWidth = sz1.width;
  int rHeight = sz1.height;

  cv::Size sz2 = image3.size();
  int tWidth = sz2.width;
  int tHeight = sz2.height;

  for(int i=0; i < pointSet11.size(); i++) {
    cv::circle(image1, pointSet11[i], 8, color1, 5);
  }
  
  for(int i=0; i < pointSet31.size(); i++) {
    cv::circle(image1, pointSet31[i], 8, color1, 5);
  }

  for(int i=0; i < pointSet12.size(); i++) {
    cv::circle(image2, pointSet12[i] + 
        cv::Point2f(qWidth, 0), 8, color2, 5);
  }
  
  for(int i=0; i < pointSet22.size(); i++) {
    cv::circle(image2, pointSet22[i] + cv::Point2f(qWidth, 0), 8, color2, 5);
  }
  
  for(int i=0; i < pointSet23.size(); i++) {
    cv::circle(image3, pointSet23[i] +  cv::Point2f(qWidth+rWidth, 0), 8, color3, 5);
  }
  
  for(int i=0; i < pointSet33.size(); i++) {
    cv::circle(image3, pointSet33[i] +  cv::Point2f(qWidth+rWidth, 0), 8, color3, 5);
  }

  int canvasWidth = qWidth + rWidth + tWidth;
  int canvasHeight = 0;
  if(qHeight >= rHeight) {
    if(qHeight >= tHeight) {
      canvasHeight = qHeight;
    } else {
      canvasHeight = tHeight;
    } 
  }else if( rHeight >= tHeight ) {
    canvasHeight = rHeight;
  } else {
    canvasHeight = tHeight;
  }


  cv::Mat canvas(canvasHeight, canvasWidth, CV_8UC3, cv::Scalar(0,0,0));

  cv::Rect roi1 = cv::Rect(0,0,qWidth,qHeight);
  cv::Mat canvas_roi1 = canvas(roi1);
  image1.copyTo(canvas_roi1);

  cv::Rect roi2 = cv::Rect(qWidth,0,rWidth,rHeight);
  cv::Mat canvas_roi2 = canvas(roi2);
  image2.copyTo(canvas_roi2);

  cv::Rect roi3 = cv::Rect(qWidth+rWidth,0,tWidth,tHeight);
  cv::Mat canvas_roi3 = canvas(roi3);
  image2.copyTo(canvas_roi3);

  char scoreValue1[256];
  sprintf(scoreValue1, "%lf", score);
  
  cv::putText(canvas, scoreValue1, cv::Point2f(qWidth, qHeight - 20), 
      cv::FONT_HERSHEY_COMPLEX_SMALL, 10, cvScalar(200,200,250), 10, CV_AA);

//  cv::Size origSize = canvas.size();
//  cv::Size halfSize(origSize.width/5, origSize.height/5);
//  cv::resize(canvas, canvas, halfSize); 
      
  if(outputFile != "") {
    cv::imwrite(outputFile, canvas);
  } else {
    cv::namedWindow("Canvas", CV_WINDOW_NORMAL);
    cv::imshow("Canvas",canvas);
    cv::waitKey(0);
  }

  return;
}


void DrawPairsWithPoints(const string& imageName1, const string& imageName2, vector< cv::Point2f >& pointSet1, vector<cv::Point2f>& pointSet2, double score, string outputFile) {

  cv::Scalar color1(255,0,0);
  cv::Mat image1 = cv::imread( imageName1, CV_LOAD_IMAGE_COLOR);
  for(int i=0; i < pointSet1.size(); i++) {
    cv::circle(image1, pointSet1[i], 8, color1, 5);
  }
  
  cv::Size sz = image1.size();
  int qWidth = sz.width;
  int qHeight = sz.height;

  cv::Mat image2 = cv::imread( imageName2, CV_LOAD_IMAGE_COLOR);
  for(int i=0; i < pointSet2.size(); i++) {
    cv::circle(image2, pointSet2[i], 8, color1, 5);
    pointSet2[i].x = pointSet2[i].x + (float)(qWidth);
  }


  cv::Size sz1 = image2.size();
  int rWidth = sz1.width;
  int rHeight = sz1.height;



  int canvasWidth = qWidth + rWidth;
  int canvasHeight = qHeight > rHeight ? qHeight : rHeight;
  cv::Mat canvas(canvasHeight, canvasWidth, CV_8UC3, cv::Scalar(0,0,0));

  cv::Rect roi1 = cv::Rect(0,0,qWidth,qHeight);
  cv::Mat canvas_roi1 = canvas(roi1);
  image1.copyTo(canvas_roi1);

  cv::Rect roi2 = cv::Rect(qWidth,0,rWidth,rHeight);
  cv::Mat canvas_roi2 = canvas(roi2);
  image2.copyTo(canvas_roi2);

  /*
  for(int i=0; i < pointSet1.size(); i++) { 
    cv::line(canvas, pointSet1[i], pointSet2[i], cvScalar(255,0,0), 8);
  }*/

  char scoreValue1[256];
  sprintf(scoreValue1, "%lf", score);
  
//  cv::putText(canvas, scoreValue1, cv::Point2f(qWidth, qHeight - 20), 
//      cv::FONT_HERSHEY_COMPLEX_SMALL, 10, cvScalar(200,200,250), 10, CV_AA);

  cv::Size origSize = canvas.size();
  cv::Size halfSize(origSize.width/4, origSize.height/4);
  cv::resize(canvas, canvas, halfSize); 
      
  if(outputFile != "") {
    cv::imwrite(outputFile, canvas);
  } else {
    cv::namedWindow("Canvas", CV_WINDOW_NORMAL);
    cv::imshow("Canvas",canvas);
    cv::waitKey(0);
  }

  return;
}

void DrawOutliers(const string& imageName1, vector< cv::Point2f > allPoints,
    vector<cv::Point2f> outlierCoords, string winName) {

  cv::Scalar color1(255,0,0);
  cv::Scalar color2(0,255,0);
  cv::Scalar color3(0,0,255);

  cv::Mat image1 = cv::imread( imageName1, CV_LOAD_IMAGE_COLOR);
  for(int i=0; i < allPoints.size(); i++) {
    cv::circle(image1, allPoints[i], 8, color1, 5);
  }
  for(int i=0; i < outlierCoords.size(); i++) {
    cv::circle(image1, outlierCoords[i], 8, color2, 15);
  }

//  cv::imwrite(winName, image1);

 cv::namedWindow(winName.c_str(), CV_WINDOW_NORMAL);
  cv::imshow(winName.c_str(), image1);

  //  cv::waitKey(0);


}
