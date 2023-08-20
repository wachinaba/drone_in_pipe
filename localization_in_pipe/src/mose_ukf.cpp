#include <ros/ros.h>
#include <Eigen/Dense>

#include "localization_in_pipe/mose_ukf.h"

MoseEKF::MoseEKF(Eigen::VectorXd x, Eigen::MatrixXd p_mat, Eigen::MatrixXd q_mat, Eigen::MatrixXd r_mat)
{
  x_ = x;
  p_mat_ = p_mat;
  q_mat_ = q_mat;
  r_mat_ = r_mat;
}

MoseEKF::~MoseEKF()
{
}
