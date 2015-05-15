#include "opencv2/opencv.hpp"
#include <string>
#include <cmath>

#define P_TO_A_THRESH 3
#define AREA_THRESH 50
#define ALPHA 2

enum color_t {COLOR, GRAY, COLOR_MAX};
const char* colorNames[COLOR_MAX] = {"Colored", "Gray"};

enum shape_t {TRIANGLE, RECTANGLE, PENTAGON, ELLIPSE, SHAPE_MAX};
#define POLY_MAX 5
const char* shapeNames[SHAPE_MAX] = {"Triangle", "Rectangle", "Pentagon", "Ellipse"};

const int hThreshLow[COLOR_MAX] =     {0, 0};
const int hThreshHigh[COLOR_MAX] =    {180, 180};
const double sThreshLow[COLOR_MAX] =  {.3, 0};
const double sThreshHigh[COLOR_MAX] = {1, .5};
const double vThreshLow[COLOR_MAX] =  {.7, .4};
const double vThreshHigh[COLOR_MAX] = {1, .8};
const cv::Rect regionOfInterest(170, 20, 330, 240);

int main(void){
  cv::VideoCapture cap(0);
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 1600);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);
  if(!cap.isOpened()){
    return -1;
  }

  cv::Mat frame;
  cap >> frame;
  frame = frame(regionOfInterest);
  cv::imwrite("pics/frame.png", frame);
  cv::Mat new_frame = cv::Mat::zeros( frame.size(), frame.type() );
  for( int y = 0; y < frame.rows; y++ ){
    for( int x = 0; x < frame.cols; x++ ){
      for( int c = 0; c < 3; c++ ){
	new_frame.at<cv::Vec3b>(y,x)[c] = cv::saturate_cast<uchar>(ALPHA*( frame.at<cv::Vec3b>(y,x)[c] ));
      }
    }
  }
  cv::imwrite("pics/new_frame.png", new_frame);
  cv::Mat hsvFrame;
  cv::cvtColor(new_frame, hsvFrame, CV_BGR2HSV);

  for(int color = 0; color < COLOR_MAX; color++){

    cv::Mat lowerBound(hsvFrame.size(), CV_8UC3, cv::Scalar((unsigned char)hThreshLow[color],(unsigned char)(sThreshLow[color]*255),(unsigned char)(vThreshLow[color]*255)));
    cv::Mat upperBound(hsvFrame.size(), CV_8UC3, cv::Scalar((unsigned char)hThreshHigh[color],(unsigned char)(sThreshHigh[color]*255),(unsigned char)(vThreshHigh[color]*255)));
    
    cv::Mat binary;
    cv::inRange(hsvFrame, lowerBound, upperBound, binary);
    char name[20];
    *name = '\0';
    strcat(name, "pics/");
    strcat(name,colorNames[color]);
    strcat(name,"_Binary.png");
    cv::imwrite(name, binary);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(binary, contours, cv::noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for(int i = 0; i < contours.size(); i++){
      if(P_TO_A_THRESH*cv::arcLength(contours[i],true) > cv::contourArea(contours[i]) || cv::contourArea(contours[i]) < AREA_THRESH){
	contours.erase(contours.begin()+i);
	i--;
      }
    }

    for(int i = 0; i < contours.size(); i++){
      cv::convexHull(contours[i], contours[i]);
    }

    cv::Mat contourIm(frame.size(),CV_8UC3,cv::Scalar(0,0,0));
    for(int i = 0; i < contours.size(); i++){
      cv::drawContours(contourIm, contours, i, cv::Scalar(255,255,255));
    }
    *name = '\0';
    strcat(name, "pics/");
    strcat(name,colorNames[color]);
    strcat(name,"_Contour.png");
    cv::imwrite(name, contourIm);

    //Shape recognition
    for(int i = 0; i < contours.size(); i++){
      cv::Moments moments = cv::moments(contours[i]);
      cv::Point2f centroid(moments.m10/moments.m00, moments.m01/moments.m00);
      //find min and max dist from centroid
      //cv::Point2f minDistPoint(-1000,-1000), maxDistPoint(centroid);
      cv::Point2f maxDistPoint(centroid);
      for(int j = 0; j < contours[i].size(); j++){
	cv::Point2f curr = contours[i][j];
	/*if(cv::norm(centroid-curr) < cv::norm(centroid-minDistPoint)){
	  minDistPoint = curr;
	}*/
	if(cv::norm(centroid-curr) > cv::norm(centroid-maxDistPoint)){
	  maxDistPoint = curr;
	}
      }
      double minDist = std::abs(cv::pointPolygonTest(contours[i], centroid, true));
      //Results of distance checking
      std::cout << "Color: " << colorNames[color] << "\n";
      std::cout << "Shape number " << i << "\n";
      std::cout << centroid << "\n";
      std::cout << "max dist " << maxDistPoint << " at dist " << cv::norm(centroid-maxDistPoint) << "\n";
      //std::cout << "min dist " << minDistPoint << " at dist " << cv::norm(centroid-minDistPoint) << "\n";
      std::cout << "min dist " << minDist << "\n";
      //Detect shape
      double realArea = contourArea(contours[i]);
      std::cout << "real area is " << realArea << "\n";
      int bestShape = -1;
      double bestArea = 0;
      for(int shapeID = 0; shapeID < SHAPE_MAX; shapeID++){
	double area;
	if(shapeID <= POLY_MAX-3){
	  //area = (shapeID+3)*2*std::sqrt(std::pow(cv::norm(centroid-maxDistPoint),2)-std::pow(cv::norm(centroid-minDistPoint),2))*cv::norm(centroid-minDistPoint);
	  area = (shapeID+3)*std::sqrt(std::pow(cv::norm(centroid-maxDistPoint),2)-std::pow(minDist,2))*minDist;
	} else if(shapeID == ELLIPSE){
	  //area = M_PI*cv::norm(centroid-maxDistPoint)*cv::norm(centroid-minDistPoint);
	  area = M_PI*cv::norm(centroid-maxDistPoint)*minDist;
	} else{
	  area = 0;
	}
	std::cout << "trying shape " << shapeNames[shapeID] << " getting area " << area << "\n";
	if(std::abs(area - realArea) < std::abs(bestArea-realArea)){
	  bestShape = shapeID;
	  bestArea = area;
	}
      }
      //Results of analysis:
      if(bestShape < 0){
	std::cout << "no good match\n";
      } else{
	std::cout << "Best shape match: " << shapeNames[bestShape] << "\n";
      }
    }
  }
}
