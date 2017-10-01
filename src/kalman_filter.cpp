#include "kalman_filter.h"
#include <iostream>
#include "tools.h"
#include <math.h>
using namespace std;

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
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  //cout << "KalmanFilter::Update" << z << endl;
  VectorXd z_pred = H_ * x_; //
  //cout << "z_pred" << z_pred << endl;
	VectorXd y = z - z_pred;
  //cout << "y" << y << endl;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
  //cout << "S" << S << endl;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
  //cout << "PHt" << PHt << endl;
	MatrixXd K = PHt * Si;
  //cout << "K" << K << endl;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
  Tools tools;
  MatrixXd Hj = tools.CalculateJacobian(x_);

  float x  = x_(0);
  float y  = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(x*x + y *y);
  float theta = atan2(y, x);
  float ro_dot = (x*vx + y*vy)/rho;
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, ro_dot;
  VectorXd y_diff = z - z_pred;
  // normalize the phi angle into range [-Pi; Pi]

  y_diff(1) = atan2(sin(y_diff(1)), cos(y_diff(1)));

  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj * P_ * Ht + R_radar_;
  //cout << "EKF S" << S << endl;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  //cout << "EKF PHt" << PHt << endl;
  MatrixXd K = PHt * Si;
  //cout << "EKF K" << K << endl;
  //new estimate
  //cout << " K * y " << K * y << endl;
  //cout << " x_ " << x_ << endl;
  x_ = x_ + (K * y_diff);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;
}
