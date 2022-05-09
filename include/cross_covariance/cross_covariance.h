#include <sys/types.h>
#include <unistd.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

using namespace std;

#define distance2points(a, b) sqrt(pow(b.pose.position.x - a.pose.position.x, 2) + pow(b.pose.position.y - a.pose.position.y, 2))

#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

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
    cross_covariance();
    void cross_cov();
    void init(autoware_msgs::DetectedObject &a, autoware_msgs::DetectedObject &b, float beta);
};

/*
This function can transform pose from local frame to map frame.
*/
geometry_msgs::Pose getTransformedPose(const geometry_msgs::Pose &in_pose, const tf::StampedTransform &tf_stamp);

/*
This function can find the transformation matrix from local frame to map frame.
local2global_ will save the matrix including translation and rotation.
*/
bool updateNecessaryTransform(const string &input, tf::StampedTransform &local2global_);

void transformPoseToGlobal(const autoware_msgs::DetectedObjectArray &input,
                           autoware_msgs::DetectedObjectArray &transformed_input,
                           tf::StampedTransform &local2global_);

void get_logfilename(ofstream &logfile, string &save_path_, string &logfile_name_);

void saveResult(ofstream &logfile,
                string &save_path_,
                string &logfile_name_,
                const autoware_msgs::DetectedObjectArray &ground_truth,
                const autoware_msgs::DetectedObjectArray &obu,
                const autoware_msgs::DetectedObjectArray &rsu,
                const autoware_msgs::DetectedObjectArray &output);