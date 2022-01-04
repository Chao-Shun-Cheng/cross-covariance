#include "Eigen/Dense"
#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "autoware_msgs/DetectedObject.h"

using namespace std;

class cross_covariance
{
public:
    Eigen::MatrixXd c_state;
    Eigen::MatrixXd c_cov;
    cross_covariance(const Eigen::MatrixXd a_s, const Eigen::MatrixXd a_cov, 
                    const Eigen::MatrixXd b_s, const Eigen::MatrixXd b_cov, float beta);
}