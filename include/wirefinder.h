#ifndef STEREOWIRE_H
#define STEREOWIRE_H
#include <chrono>
#include <string>
#include <ros/ros.h>
#include <stereo_msgs/DisparityImage.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>

class WireFinder {
public:
    WireFinder(const std::string &topic = "stereo/disparity");
    double a_, b_, wireWidth_, disparity_; // koefficients a, b
private:
    void disparityCallback(const stereo_msgs::DisparityImage& msg);
    void selectRange(const cv::Mat &disparity, int *min, int *max);
    ros::Subscriber disparitySub_;
    image_transport::Publisher stateImagePub_;
    cv::Mat drawHistogram(const cv::Mat &hist, int min, int max, const size_t histogramWidth = 512, const size_t histogramHeight = 600);
    void scanHistogram(const cv::Mat &hist, int *min, int *max);
    void drawResult(cv::Mat *image, const cv::Size &size, const cv::Mat &disparity, const int min, const int max);
    std::chrono::milliseconds lastOut_, minOutDiff_, lastDisparity_;
    float currentFps_;
};

#endif // STEREOWIRE_H
