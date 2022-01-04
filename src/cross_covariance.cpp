#include <cross_covariance/cross_covariance.h>

cross_covariance::cross_covariance(const Eigen::MatrixXd a_s, const Eigen::MatrixXd a_cov, 
                                    const Eigen::MatrixXd b_s, const Eigen::MatrixXd b_cov, float beta)
{
    Eigen::MatrixXd cross_cov = a_cov.cwiseProduct(b_cov);
    cross_cov = beta * cross_cov.llt().matrixL();
    Eigen::MatrixXd U = a_cov + b_cov - cross_cov - cross_cov.transpose();
    Eigen::MatrixXd alpha = (a_cov - cross_cov) * U.inverse();
    this->c_state = a_s + alpha * (b_s - a_s);
    this->c_cov = a_cov - (a_cov - cross_cov) * U.inverse() * (a_cov - cross_cov).transpose();
}