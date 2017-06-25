#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  while (y(1) < -M_PI)  //added per forus discusssion RMSE values of EKF project
    y(1) += 2 * M_PI;
  while (y(1) > M_PI)
    y(1) -= 2* M_PI;


  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;


  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();  //not sure about this
  MatrixXd I = MatrixXd::Identity(x_size, x_size);  //or this
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  //float x = ekf_.x_(0);
  //float yy = ekf_.x_(1);  //changed "y" to "yy" vs lessons
  //float vx = ekf_.x_(2);
  //float vy = ekf_.x_(3);

  float x = x_(0);
  float yy = x_(1);  //changed "y" to "yy" vs lessons
  float vx = x_(2);
  float vy = x_(3);

  //float elon = 0.001;  // added per forum Nan values in P_
  //if (x < elon && yy < elon) {
    //x = elon;
    //yy = elon;
  //}

  float rho = sqrt(x*x+yy*yy);  //ch to yy
  float theta = atan2(yy,x);  // ch to yy
  float dfrnce = z(1) - theta;

  if (dfrnce > 6.2) {
    theta = theta + 6.28318530717959;
  } // new if stmt per discussion on forum

  float ro_dot = (x*vx+yy*vy)/rho;  //ch to yy
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, ro_dot;

  VectorXd y = z - z_pred;

  while (y(1) < -M_PI)  //added per forus discusssion RMSE values of EKF project
    y(1) += 2 * M_PI;
  while (y(1) > M_PI)
    y(1) -= 2* M_PI;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();  //not sure about this (again)
  MatrixXd I = MatrixXd::Identity(x_size, x_size);  //or this (again)
  P_ = (I - K * H_) * P_;

  //x_ = F_ * x_ + u;
  //MatrixXd Ft = F_.transpose();
  //P_ = F_ * P_ * Ft + Q_;
  

  
}
