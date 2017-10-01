#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if(estimations.size() == 0 || ground_truth.size() == 0){
      return rmse;
  } else if ( estimations.size() != ground_truth.size()){
      cout << "estimations size does not equal to ground_truth size" << endl;
      return rmse;
  }
  //  * the estimation vector size should equal ground truth vector size
  //accumulate squared residuals
  for(int i=0; i < estimations.size(); i++){

    VectorXd est = estimations[i];
    VectorXd tru = ground_truth[i];
    VectorXd diff = est - tru;
    VectorXd dt2 = diff.array() * diff.array();

    rmse += dt2;

  }
  //calculate the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();
  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
  */

  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  Hj << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;
  float px2py2 = px * px + py * py;
  float sqrt_pxpy2 = sqrt(px2py2);
  // cout << px2py2 << " square root: " << sqrt_pxpy2 << endl;
  //check division by zero
  if(sqrt_pxpy2 == 0){
    return Hj;
  }
  //compute the Jacobian matrix
  Hj(0, 0) = px / sqrt_pxpy2;
  Hj(0, 1) = py / sqrt_pxpy2;
  Hj(1, 0) = - py / px2py2;
  Hj(1, 1) = px / px2py2;
  Hj(2, 0) = py * (vx * py - vy * px) / pow(px2py2, 1.5);
  Hj(2, 1) = px * (vy * px - vx * py) / pow(px2py2, 1.5);
  Hj(2, 2) = Hj(0, 0);
  Hj(2, 3) = Hj(0, 1);
  return Hj;
}
