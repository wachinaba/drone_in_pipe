#ifndef MOSE_EKF_H
#define MOSE_EKF_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <ukf_ros/core.h>

class MoseEKF
{
public:
  /*
   * @brief Constructor
   * @param x Initial state vector
   * @param p_mat Initial state covariance matrix
   * @param q_mat Process noise covariance matrix
   * @param r_mat Measurement noise covariance matrix
   */
  MoseEKF(Eigen::VectorXd x, Eigen::MatrixXd p_mat, Eigen::MatrixXd q_mat, Eigen::MatrixXd r_mat);

  /*
   * destructor
   */
  virtual ~MoseEKF();

  /*
   * @brief Predict the state vector and covariance matrix
   * @param dt Time step
   */
  void predict(double dt);

  /*
   * @brief Update the state vector and covariance matrix
   * @param z Measurement vector
   */
  void update(Eigen::VectorXd z);

  /*
   * @brief Get the state vector
   * @return State vector
   */
  Eigen::VectorXd getState();

  /*
   * @brief Get the state covariance matrix
   * @return State covariance matrix
   */
  Eigen::MatrixXd getCovariance();

private:
  Eigen::VectorXd x_;      // state vector
  Eigen::MatrixXd p_mat_;  // state covariance matrix
  Eigen::MatrixXd q_mat_;  // process noise covariance matrix
  Eigen::MatrixXd r_mat_;  // measurement noise covariance matrix
};

#endif  // MOSE_EKF_H