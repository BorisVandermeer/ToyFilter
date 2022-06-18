// Author : BorisVandermeer
// Zhejiang University
// Date : 2022.06.17

#include<iostream>
#include<vector>
#include"KalmanFilter.h"

using namespace std;
using namespace ToyFilter;

int main() {
  double dt = 1.0/30; // Time step

  Eigen::MatrixXd A(1,1); // System dynamics matrix
  Eigen::MatrixXd H(1,1); // Output matrix
  Eigen::MatrixXd Q(1,1); // Process noise covariance
  Eigen::MatrixXd R(1,1); // Measurement noise covariance
  Eigen::MatrixXd P(1,1); // Estimate error covariance

  A << 1;
  H << 1;
  Q << 0;
  R << 0.1;
  P << 1;

  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << H << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilter kf(A, H, Q, R);

  // List of noisy position measurements (y)
  std::vector<double> measurements = {
    0.39,0.50,0.48,0.29,0.25,0.32,0.34,0.48,0.41,0.45
    // 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,1,1,1,1,1,0,1,1,1,1,1,1
  };

  // Best guess of initial states
  Eigen::VectorXd x0(1);
  x0 << 0;
  kf.init(x0,P);

  // Feed measurements into filter, output estimated states

  Eigen::VectorXd y(1);
  std::cout <<"x_hat[0] = " << kf.state().transpose() << std::endl;
  for(int i = 0; i < measurements.size(); i++) {
    y << measurements[i];
    kf.update(y);
    std::cout << "x_hat[" << i << "] = " << kf.state().transpose() << std::endl;
  }

  return 0;
}