#include "ros/ros.h"
#include "ipdc/GetShapes.h"
#include "opencv2/opencv.hpp"
#include <string>
#include <cmath>

#define P_TO_A_THRESH 5
#define AREA_THRESH 100

enum color_t {COLOR, GRAY, COLOR_MAX};
const char* colorNames[COLOR_MAX] = {"Colored", "Gray"};

enum shape_t {TRIANGLE, RECTANGLE, PENTAGON, ELLIPSE, SHAPE_MAX};
#define POLY_MAX 5
const char* shapeNames[SHAPE_MAX] = {"Triangle", "Rectangle", "Pentagon", "Ellipse"};

const int hThreshLow[COLOR_MAX] =     {0, 0};
const int hThreshHigh[COLOR_MAX] =    {180, 180};
const double sThreshLow[COLOR_MAX] =  {.4, 0};
const double sThreshHigh[COLOR_MAX] = {1, .35};
const double vThreshLow[COLOR_MAX] =  {.3, 0};
const double vThreshHigh[COLOR_MAX] = {1, .5};

bool getShapes(ipdc::GetShapes::Request  &req,
	       ipdc::GetShapes::Response &res)
{
  int color;
  if(req.control){
    color = GRAY;
  } else{
    color = COLOR;
  }

  cv::VideoCapture cap(1);
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
  if(!cap.isOpened()){
    return -1;
  }

  cv::Mat frame;
  cap >> frame;
  cv::imwrite("pics/frame.png", frame);
  cv::Mat hsvFrame;
  cv::cvtColor(frame, hsvFrame, CV_BGR2HSV);

  cv::Mat lowerBound(hsvFrame.size(), CV_8UC3, cv::Scalar((unsigned char)hThreshLow[color],(unsigned char)(sThreshLow[color]*255),(unsigned char)(vThreshLow[color]*255)));
  cv::Mat upperBound(hsvFrame.size(), CV_8UC3, cv::Scalar((unsigned char)hThreshHigh[color],(unsigned char)(sThreshHigh[color]*255),(unsigned char)(vThreshHigh[color]*255)));
    
  cv::Mat binary;
  cv::inRange(hsvFrame, lowerBound, upperBound, binary);

  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(binary, contours, cv::noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  for(int i = 0; i < contours.size(); i++){
    if(P_TO_A_THRESH*cv::arcLength(contours[i],true) > cv::contourArea(contours[i]) || cv::contourArea(contours[i]) < AREA_THRESH){
      contours.erase(contours.begin()+i);
      i--;
    }
  }

  /*for(int i = 0; i < contours.size(); i++){
    cv::convexHull(contours[i], contours[i]);
    }*/

  cv::Mat contourIm(frame.size(),CV_8UC3,cv::Scalar(0,0,0));
  for(int i = 0; i < contours.size(); i++){
    cv::drawContours(contourIm, contours, i, cv::Scalar(255,255,255));
  }

  std::vector<int> shapeIDs (-1,contours.size());
  std::vector<double> centerX (-1,contours.size());
  std::vector<double> centerY (-1,contours.size());
  std::vector<double> areas (-1, contours.size());
  std::vector<double> directions (-1, contours.size());
  
  //Shape recognition
  for(int i = 0; i < contours.size(); i++){
    cv::Moments moments = cv::moments(contours[i]);
    cv::Point2f centroid(moments.m10/moments.m00, moments.m01/moments.m00);
    centerX[i] = centroid.x;
    centerY[i] = centroid.x;
    
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
    cv::Point2f dirVec = maxDistPoint-centroid;
    directions[i] = std::atan(dirVec.y/dirVec.x);

    double minDist = std::abs(cv::pointPolygonTest(contours[i], centroid, true));
    //Results of distance checking
    ROS_INFO("Color: %s", colorNames[color]);
    ROS_INFO("Shape number %d", i);
    ROS_INFO("%f", centroid);
    ROS_INFO("max dist %f at dist %f", maxDistPoint, cv::norm(centroid-maxDistPoint));
    ROS_INFO("min dist %f", minDist);
    //Detect shape
    areas[i] = contourArea(contours[i]);
    ROS_INFO("real area is %f");
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
      ROS_INFO("trying shape %s getting area %f", shapeNames[shapeID], area);
      if(std::abs(area - areas[i]) < std::abs(bestArea-areas[i])){
	bestShape = shapeID;
	bestArea = area;
      }
    }
    shapeIDs[i] = bestShape;
    //Results of analysis:
    if(bestShape < 0){
      ROS_INFO("no good match");
    } else{
      ROS_INFO("Best shape match: %s", shapeNames[bestShape]);
    }
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_shapes_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("get_shapes", getShapes);
  ROS_INFO("Ready to find shapes.");
  ros::spin();

  return 0;
}
