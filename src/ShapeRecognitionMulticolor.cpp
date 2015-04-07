#include "opencv2/opencv.hpp"
#include <string>

#define P_TO_A_THRESH 5
#define AREA_THRESH 100

enum color {RED, DARKPINK, LIGHTPINK, ORANGE, YELLOW, DARKBLUE, LIGHTBLUE, DARKGREEN, LIGHTGREEN, GRAY, COLOR_MAX};
const char* colorNames[COLOR_MAX] = {"Red", "Dark_Pink", "Light_Pink", "Orange", "Yellow", "Dark_Blue", "Light_Blue", "Dark_Green", "Light_Green", "Gray"};

const int hShifts[COLOR_MAX] =        {40,   40,  40,  0,   0,   0,   0,   0,   0,   0};
const int hThreshLow[COLOR_MAX] =     {350,  338, 330, 0,   0,   205, 180, 0,   35,  0};
const int hThreshHigh[COLOR_MAX] =    {361,  350, 350, 0,   0,   225, 205, 0,   55,  360};
const double sThreshLow[COLOR_MAX] =  {.575, .45, .1,  0,   0,   .65, .4,   0,   .6,  0};
const double sThreshHigh[COLOR_MAX] = {.8,   .8,  .45, 0,   0,   .85, .7,   0,   .8,  .1};
const double vThreshLow[COLOR_MAX] =  {.5,   .5,  .5,  .5,  .5,  .5,  .5,  .5,  .4,  .15};
const double vThreshHigh[COLOR_MAX] = {1,    1,   1,   1,   1,   1,   1,   1,   1,   .25};

int main(void){
  cv::VideoCapture cap(1);
  if(!cap.isOpened()){
    return -1;
  }

  cv::Mat frame;
  cap >> frame;
  cv::imwrite("frame.png", frame);
  cv::Mat hsvFrame;
  cv::cvtColor(frame, hsvFrame, CV_BGR2HSV);

  for(int color = 0; color < COLOR_MAX; color++){
    cv::Mat hsvCopy;
    if(hShifts[color] != 0){
      hsvCopy = hsvFrame.clone();

      for(int i = 0; i < hsvCopy.rows; i++){
	unsigned char* ptr = hsvCopy.ptr(i);
	for(int j = 0; j < hsvCopy.cols; j++){
	  *ptr = (*ptr + hShifts[color]/2)%180;
	  ptr += 3;
	}
      }
    } else{
      hsvCopy = hsvFrame;
    }

    cv::Mat lowerBound(hsvCopy.size(), CV_8UC3, cv::Scalar((unsigned char)(((hThreshLow[color]+hShifts[color])/2)%180),(unsigned char)(sThreshLow[color]*255),(unsigned char)(vThreshLow[color]*255)));
    //std::cout << (((hThreshLow[color]+hShifts[color])/2)%180) << '\n' << (sThreshLow[color]*255) << '\n' << (vThreshLow[color]*255) << '\n';
    //std::cout << (((hThreshHigh[color]+hShifts[color])/2)%180) << '\n' << (sThreshHigh[color]*255) << '\n' << (vThreshHigh[color]*255) << '\n';
    cv::Mat upperBound(hsvCopy.size(), CV_8UC3, cv::Scalar((unsigned char)(((hThreshHigh[color]+hShifts[color]+1)/2)%180),(unsigned char)(sThreshHigh[color]*255),(unsigned char)(vThreshHigh[color]*255)));
    
    cv::Mat binary;
    cv::inRange(hsvCopy, lowerBound, upperBound, binary);
    char name[20];
    *name = '\0';
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

    /*std::cout << colorNames[color] << '\n';
    int centroids[contours.size()];
    for(int i = 0; i < contours.size(); i++){
      centroids[i] = cv::mc(contours[i]);
      std::cout << centroids[i] << '\n';
      }*/

    cv::Mat contourIm(frame.size(),CV_8UC3,cv::Scalar(0,0,0));
    for(int i = 0; i < contours.size(); i++){
      cv::drawContours(contourIm, contours, i, cv::Scalar(255,255,255));
    }
    *name = '\0';
    strcat(name,colorNames[color]);
    strcat(name,"_Contour.png");
    cv::imwrite(name, contourIm);
  }
}
