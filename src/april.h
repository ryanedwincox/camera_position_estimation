#ifndef APRIL_H
#define APRIL_H

#include <Eigen/Core>

// April tag includes
#include "apriltags/AprilTags/TagDetector.h"
#include "apriltags/AprilTags/Tag36h11.h"

// ROS includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class April
{
public:
    April();
    void setup(ros::NodeHandle nh);
    void loop();
    void processImage(cv::Mat& image, cv::Mat& image_gray);
    void publishCameraTF(cv::Mat rMat, cv::Mat tvec);
    void publishMarkerTF();
    cv::Mat projectAxis(cv::Mat img, Eigen::Matrix3d rot, Eigen::Vector3d trans);

private:
    AprilTags::TagDetector* tagDetector;
    cv::VideoCapture* cap;
    ros::NodeHandle nh;
};

#endif // APRIL_H
