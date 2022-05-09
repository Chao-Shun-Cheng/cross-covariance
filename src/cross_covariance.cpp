#include <cross_covariance/cross_covariance.h>

cross_covariance::cross_covariance()
{
    this->a_state = Eigen::MatrixXd(5, 1);
    this->b_state = Eigen::MatrixXd(5, 1);
    this->a_cov = Eigen::MatrixXd(5, 5);
    this->b_cov = Eigen::MatrixXd(5, 5);
}

void cross_covariance::init(autoware_msgs::DetectedObject &a, autoware_msgs::DetectedObject &b, float beta)
{
    this->a_state << a.pose.position.x, a.pose.position.y, a.velocity.linear.x, tf::getYaw(a.pose.orientation), a.acceleration.linear.y;
    this->b_state << b.pose.position.x, b.pose.position.y, b.velocity.linear.x, tf::getYaw(b.pose.orientation), b.acceleration.linear.y;
    this->a_cov = Eigen::Map<Eigen::MatrixXd>(&a.covariance[5], 5, 5);
    this->b_cov = Eigen::Map<Eigen::MatrixXd>(&b.covariance[5], 5, 5);
    this->beta = beta;
}

void cross_covariance::cross_cov()
{
    Eigen::MatrixXd P = this->a_cov.cwiseProduct(this->b_cov);
    P = P.llt().matrixL();
    P = this->beta * P;
    Eigen::MatrixXd U = this->a_cov + this->b_cov - P - P.transpose();
    Eigen::MatrixXd alpha = (this->a_cov - P) * U.inverse();
    this->c_state = Eigen::MatrixXd(5, 1);
    this->c_state = this->a_state + alpha * (this->b_state - this->a_state);
    this->c_cov = Eigen::MatrixXd(5, 5);
    this->c_cov = this->a_cov - (this->a_cov - P) * U.inverse() * (this->a_cov - P).transpose();
}

/*
This function can transform pose from local frame to map frame.
*/
geometry_msgs::Pose getTransformedPose(const geometry_msgs::Pose &in_pose, const tf::StampedTransform &tf_stamp)
{
    tf::Transform transform;
    geometry_msgs::PoseStamped out_pose;
    transform.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
    transform.setRotation(tf::Quaternion(in_pose.orientation.x, in_pose.orientation.y, in_pose.orientation.z, in_pose.orientation.w));
    geometry_msgs::PoseStamped pose_out;
    tf::poseTFToMsg(tf_stamp * transform, out_pose.pose);
    return out_pose.pose;
}

/*
This function can find the transformation matrix from local frame to map frame.
local2global_ will save the matrix including translation and rotation.
*/
bool updateNecessaryTransform(const string &input, tf::StampedTransform &local2global_)
{
    bool success = true;
    tf::TransformListener tf_listener_;
    try {
        tf_listener_.waitForTransform(input, "world", ros::Time(0), ros::Duration(3.0));
        tf_listener_.lookupTransform("world", input, ros::Time(0), local2global_);
    } catch (tf::TransformException ex)  // check TF
    {
        ROS_ERROR("%s", ex.what());
        success = false;
    }
    return success;
}

void transformPoseToGlobal(const autoware_msgs::DetectedObjectArray &input,
                           autoware_msgs::DetectedObjectArray &transformed_input,
                           tf::StampedTransform &local2global_)
{
    transformed_input.objects.clear();
    transformed_input.header = input.header;
    transformed_input.header.frame_id = "world";
    for (auto const &object : input.objects) {
        geometry_msgs::Pose out_pose = getTransformedPose(object.pose, local2global_);

        autoware_msgs::DetectedObject dd;
        dd = object;
        dd.header.frame_id = "world";
        dd.pose = out_pose;

        transformed_input.objects.push_back(dd);
    }
}

void get_logfilename(ofstream &logfile, string &save_path_, string &logfile_name_)
{
    logfile.open(save_path_ + logfile_name_, std::ofstream::out | std::ofstream::app);
    if (!logfile.is_open()) {
        cerr << RED << "failed to open " << save_path_ << logfile_name_ << RESET << '\n';
    } else {
        logfile << "Time,Ground_x,Ground_y,obu_x,obu_y,"
                << "rsu_x,rsu_y,cross_x,cross_y,corss_vel,cross_yaw,cross_yawrate,"
                << "covariance_x,covariance_y,covariance_velocity,covariance_yaw,covariance_yaw_rate\n";
        logfile.close();
        cout << YELLOW << "save path : " << save_path_ + logfile_name_ << RESET << endl;
    }
    return;
}

void saveResult(ofstream &logfile,
                string &save_path_,
                string &logfile_name_,
                const autoware_msgs::DetectedObjectArray &ground_truth,
                const autoware_msgs::DetectedObjectArray &obu,
                const autoware_msgs::DetectedObjectArray &rsu,
                const autoware_msgs::DetectedObjectArray &output)
{
    logfile.open(save_path_ + logfile_name_, std::ofstream::out | std::ofstream::app);
    if (!logfile.is_open()) {
        std::cerr << RED << "failed to open " << save_path_ << logfile_name_ << RESET << '\n';
    } else {
        // Time
        logfile << std::to_string(ground_truth.header.stamp.toSec()) << ",";

        // Ground x, Ground y
        if (ground_truth.objects.size() == 0) 
            logfile << "nan,nan,";
        else
            logfile << std::to_string(ground_truth.objects[0].pose.position.x) << "," << std::to_string(ground_truth.objects[0].pose.position.y) << ",";

        // obu_x, obu_y
        if (obu.objects.size() == 0)
            logfile << "nan,nan,";
        else
            logfile << std::to_string(obu.objects[0].pose.position.x) << "," << std::to_string(obu.objects[0].pose.position.y) << ",";

        // rsu_x, rsu_y
        if (rsu.objects.size() == 0)
            logfile << "nan,nan,";
        else
            logfile << std::to_string(rsu.objects[0].pose.position.x) << "," << std::to_string(rsu.objects[0].pose.position.y) << ",";

        // cross covariance
        if (output.objects.size() == 0) {
            logfile << "nan,nan,nan,nan,nan,nan,nan,nan,nan,nan";
            logfile << std::endl;
            logfile.close();
            cout << RED << "No RESULT in Track to Track Fusion !!!\n" << RESET << endl;
            return;
        } else {
            // cross_x, cross_y, cross_velocity, cross_yaw, cross yaw rate
            logfile << std::to_string(output.objects[0].pose.position.x) << ","
                    << std::to_string(output.objects[0].pose.position.y) << "," << std::to_string(output.objects[0].velocity.linear.x) << ","
                    << std::to_string(tf::getYaw(output.objects[0].pose.orientation)) << "," << std::to_string(output.objects[0].acceleration.linear.y)
                    << ",";
            // covariance x, covariance y, covariance velocity, covariance yaw, covariance yaw rate
            logfile << std::to_string(output.objects[0].covariance[0]) << "," << std::to_string(output.objects[0].covariance[6]) << ","
                    << std::to_string(output.objects[0].covariance[12]) << "," << std::to_string(output.objects[0].covariance[18]) << ","
                    << std::to_string(output.objects[0].covariance[24]) << std::endl;
        }
        logfile.close();
    }
}
