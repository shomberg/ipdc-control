#include "opencv2/opencv.hpp"
#define BTHRESH 1.6
#define GTHRESH 1.75

int main(){
  cv::VideoCapture cap(1);
  if(!cap.isOpened()){
    return -1;
  }

  cv::Mat frame;
  cap >> frame;
  //std::cout << frame;
  cv::imwrite("frame.png", frame);

  cv::Mat binary;
  cv::Mat lowerBound = frame.clone();
  for(int i = 0; i < lowerBound.rows; i++){
    unsigned char* ptr = lowerBound.ptr(i);
    for(int j = 0; j < lowerBound.cols; j++){
      /*if(*(ptr+2) > 200){
	std::cout << "b:" << *ptr << " g:" << *(ptr+1) << " r:" << *(ptr+2);
	}*/
      *(ptr+2) = (unsigned char)std::max(std::min(std::max(BTHRESH*(*ptr),GTHRESH*(*(ptr+1))),245.0),0.0);
      *ptr = 0;
      *(ptr+1) = 0;
      ptr += 3;
    }
  }
  //std::cout << lowerBound;
  cv::Mat upperBound(frame.size(), CV_8UC3, cv::Scalar(255,255,255));
  inRange(frame, lowerBound, upperBound, binary);
  cv::imwrite("binary.png", binary);

  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(binary, contours, cv::noArray(), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  
  cv::Mat contourIm(frame.size(),CV_8UC3,cv::Scalar(0,0,0));
  for(int i = 0; i < contours.size(); i++){
    //std::cout << contours[i] << '\n';
    cv::drawContours(contourIm, contours, i, cv::Scalar(0,0,255));
  }
  cv::imwrite("contour.png", contourIm);
}
