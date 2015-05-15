#include "ros/ros.h"
#include "ipdc/GetShapes.h"
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

cv::VideoCapture cap(0);

bool getShapes(ipdc::GetShapes::Request  &req,
	       ipdc::GetShapes::Response &res)
{
  int color;
  if(req.control){
    color = GRAY;
  } else{
    color = COLOR;
  }
  cv::Mat frame;
  for(int i = 0; i < 5; i++){
    cap >> frame;
  }
  frame = frame(regionOfInterest);
  cv::Mat new_frame = cv::Mat::zeros( frame.size(), frame.type() );
  for( int y = 0; y < frame.rows; y++ ){
    for( int x = 0; x < frame.cols; x++ ){
      for( int c = 0; c < 3; c++ ){
	new_frame.at<cv::Vec3b>(y,x)[c] = cv::saturate_cast<uchar>(ALPHA*( frame.at<cv::Vec3b>(y,x)[c] ));
      }
    }
  }
  
  //ROS_INFO("Successfully captured frame");
  //cv::imwrite("pics/frame.png", frame);
  cv::Mat hsvFrame;
  cv::cvtColor(new_frame, hsvFrame, CV_BGR2HSV);
  //ROS_INFO("Converted to HSV");

  cv::Mat lowerBound(hsvFrame.size(), CV_8UC3, cv::Scalar((unsigned char)hThreshLow[color],(unsigned char)(sThreshLow[color]*255),(unsigned char)(vThreshLow[color]*255)));
  cv::Mat upperBound(hsvFrame.size(), CV_8UC3, cv::Scalar((unsigned char)hThreshHigh[color],(unsigned char)(sThreshHigh[color]*255),(unsigned char)(vThreshHigh[color]*255)));
    
  cv::Mat binary;
  cv::inRange(hsvFrame, lowerBound, upperBound, binary);
  //ROS_INFO("Found colored pixels");

  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(binary, contours, cv::noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  //ROS_INFO("Found shape contours");

  for(int i = 0; i < contours.size(); i++){
    if(P_TO_A_THRESH*cv::arcLength(contours[i],true) > cv::contourArea(contours[i]) || cv::contourArea(contours[i]) < AREA_THRESH){
      contours.erase(contours.begin()+i);
      i--;
    }
  }
  //ROS_INFO("Trimmed artifacts");

  /*for(int i = 0; i < contours.size(); i++){
    cv::convexHull(contours[i], contours[i]);
    }*/

  //cv::Mat contourIm(frame.size(),CV_8UC3,cv::Scalar(0,0,0));
  //for(int i = 0; i < contours.size(); i++){
  //  cv::drawContours(contourIm, contours, i, cv::Scalar(255,255,255));
  //}

  std::vector<int> shape_ids (contours.size(), -1);
  std::vector<double> center_x (contours.size(), -1);
  std::vector<double> center_y (contours.size(), -1);
  std::vector<double> areas (contours.size(), -1);
  std::vector<double> directions (contours.size(), -1);
  //ROS_INFO("allocated some primitive vectors");
  
  //Shape recognition
  for(int i = 0; i < contours.size(); i++){
    cv::Moments moments = cv::moments(contours[i]);
    cv::Point2f centroid(moments.m10/moments.m00, moments.m01/moments.m00);
    center_x[i] = centroid.x;
    center_y[i] = centroid.y;
    //ROS_INFO("found centroid");
    
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
    ROS_INFO("x: %f, y: %f", centroid.x, centroid.y);
    //ROS_INFO("max dist %s at dist %f", maxDistPoint, cv::norm(centroid-maxDistPoint));
    ROS_INFO("min dist %f", minDist);
    //Detect shape
    areas[i] = contourArea(contours[i]);
    ROS_INFO("real area is %f", areas[i]);
    int bestShape = -1;
    double bestArea = 0;
    for(int shape_id = 0; shape_id < SHAPE_MAX; shape_id++){
      double area;
      if(shape_id <= POLY_MAX-3){
	//area = (shape_id+3)*2*std::sqrt(std::pow(cv::norm(centroid-maxDistPoint),2)-std::pow(cv::norm(centroid-minDistPoint),2))*cv::norm(centroid-minDistPoint);
	area = (shape_id+3)*std::sqrt(std::pow(cv::norm(centroid-maxDistPoint),2)-std::pow(minDist,2))*minDist;
      } else if(shape_id == ELLIPSE){
	//area = M_PI*cv::norm(centroid-maxDistPoint)*cv::norm(centroid-minDistPoint);
	area = M_PI*cv::norm(centroid-maxDistPoint)*minDist;
      } else{
	area = 0;
      }
      //ROS_INFO("trying shape %s getting area %f", shapeNames[shape_id], area);
      if(std::abs(area - areas[i]) < std::abs(bestArea-areas[i])){
	bestShape = shape_id;
	bestArea = area;
      }
    }
    shape_ids[i] = bestShape;
    //Results of analysis:
    if(bestShape < 0){
      ROS_INFO("no good match");
    } else{
      ROS_INFO("Best shape match: %s", shapeNames[bestShape]);
    }
  }

  res.shape_id = shape_ids;
  res.center_x = center_x;
  res.center_y = center_y;
  res.area = areas;
  res.direction = directions;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_shapes_server");
  ros::NodeHandle n;

  cap.set(CV_CAP_PROP_FRAME_WIDTH, 1600);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);
  if(!cap.isOpened()){
    return -1;
  }
  ROS_INFO("Successfully opened camera");

  ros::ServiceServer service = n.advertiseService("get_shapes", getShapes);
  ROS_INFO("Ready to find shapes.");
  ros::spin();

  return 0;
}
