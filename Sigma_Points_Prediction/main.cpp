#include "Eigen/Dense"
#include "ukf.h"

using Eigen::MatrixXd;

int main() {

  // Create a UKF instance
  UKF ukf;

  /**
   * Programming assignment calls
   */
  MatrixXd Xsig_aug = MatrixXd(15, 5);
  ukf.SigmaPointPrediction(&Xsig_aug);

  return 0;
}