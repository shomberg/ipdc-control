#include "opencv2/opencv.hpp"

#define HSHIFT 10

#define REDHTHRESHHIGH (181+HSHIFT)%180
#define REDHTHRESHLOW (170+HSHIFT)%180
#define REDSTHRESHHIGH 179
#define REDSTHRESHLOW 127
#define REDVTHRESHHIGH 255
#define REDVTHRESHLOW 0

int main(){
  cv::VideoCapture cap(1);
  if(!cap.isOpened()){
    return -1;
  }

  cv::Mat frame;
  cap >> frame;
  //std::cout << frame;
  cv::imwrite("frame.png", frame);

  cv::Mat hsvFrame;
  cv::cvtColor(frame, hsvFrame, CV_BGR2HSV);

  for(int i = 0; i < hsvFrame.rows; i++){
    unsigned char* ptr = hsvFrame.ptr(i);
    for(int j = 0; j < hsvFrame.cols; j++){
      *ptr = (*ptr + HSHIFT)%180;
      ptr += 3;
    }
  }

  cv::Mat lowerBound(frame.size(), CV_8UC3, cv::Scalar(REDHTHRESHLOW,REDSTHRESHLOW,REDVTHRESHLOW));
  cv::Mat upperBound(frame.size(), CV_8UC3, cv::Scalar(REDHTHRESHHIGH,REDSTHRESHHIGH,REDVTHRESHHIGH));

  cv::Mat redBinary;
  cv::inRange(hsvFrame, lowerBound, upperBound, redBinary);
  cv::imwrite("redBinary.png", redBinary);

  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(redBinary, contours, cv::noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  
  cv::Mat contourIm(frame.size(),CV_8UC3,cv::Scalar(0,0,0));
  for(int i = 0; i < contours.size(); i++){
    //std::cout << contours[i] << '\n';
    cv::drawContours(contourIm, contours, i, cv::Scalar(0,0,255));
  }
  cv::imwrite("contour.png", contourIm);
}
