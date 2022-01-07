#include <cross_covariance/cross_covariance.h>

cross_covariance::cross_covariance(autoware_msgs::DetectedObject &a,
                                   autoware_msgs::DetectedObject &b,
                                   float beta)
{
    this->a_state = Eigen::MatrixXd(5, 1);
    this->a_state << a.pose.position.x, a.pose.position.y, a.velocity.linear.x,
        tf::getYaw(a.pose.orientation), a.acceleration.linear.y;
    this->b_state = Eigen::MatrixXd(5, 1);
    this->b_state << b.pose.position.x, b.pose.position.y, b.velocity.linear.x,
        tf::getYaw(b.pose.orientation), b.acceleration.linear.y;
    this->a_cov = Eigen::MatrixXd(5, 5);
    this->a_cov = Eigen::Map<Eigen::MatrixXd>(&a.covariance[5], 5, 5);
    this->b_cov = Eigen::MatrixXd(5, 5);
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
    this->c_cov = this->a_cov - (this->a_cov - P) * U.inverse() *
                                    (this->a_cov - P).transpose();
}