#ifndef LINEDETECTION_HPP
#define LINEDETECTION_HPP
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

std::vector<std::vector<cv::Point>> linedetection(cv::Mat image);

#endif // LINEDETECTION_HPP
