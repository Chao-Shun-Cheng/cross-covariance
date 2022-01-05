#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf/transform_listener.h>
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

using namespace std;

#define DISTANCE_THRESHOLD 1 // meters, if distance between two objects is less than this threshold 
#define BETA 0.4 

class cross_covariance
{
public:
    Eigen::MatrixXd a_state;
    Eigen::MatrixXd a_cov;
    Eigen::MatrixXd b_state;
    Eigen::MatrixXd b_cov;
    Eigen::MatrixXd c_state;
    Eigen::MatrixXd c_cov;
    float beta;
    cross_covariance(autoware_msgs::DetectedObject &a, autoware_msgs::DetectedObject &b, float beta);
    void cross_cov();
};