#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
void whitelinedetection(Mat dewarpedImage){
  Mat grayImage;
  cvtColor(dewarpedImage, grayImage, COLOR_BGR2GRAY);

  Mat binaryImage;
  threshold(grayImage, binaryImage, 200, 255, THRESH_BINARY);

  vector<vector<Point>> contours;
  findContours(binaryImage, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  vector<vector<Point>> whiteLinesCoordinates;
  for (const auto& contour : contours) {
      whiteLinesCoordinates.push_back(contour);
  }

  for (size_t i = 0; i < whiteLinesCoordinates.size(); i++) {
      cout << "White Line " << i + 1 << " Coordinates:" << endl;
      for (const auto& point : whiteLinesCoordinates[i]) {
          cout << "(" << point.x << ", " << point.y << ")" << endl;
      }
  }

}
int main() {
    Mat dewarpedImage = imread("dewarped_image.jpg");

    if (dewarpedImage.empty()) {
        cerr << "Error: Unable to load image." << endl;
        return -1;
    }
    whitelinedetection(dewarpedImage);
    
    return 0;
}
