#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
      cout << "Error in CalculateRMSE" << endl;
      return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i) {
      VectorXd diff = estimations[i] - ground_truth[i];
      diff = diff.array() * diff.array();
      rmse += diff;
  }

  rmse = rmse / estimations.size(); // calculate the mean
  rmse = rmse.array().sqrt();
  return rmse;
}
