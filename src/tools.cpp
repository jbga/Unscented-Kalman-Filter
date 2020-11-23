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
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if(estimations.size()==0 || estimations.size() != ground_truth.size())
  {
    cerr << "Invalid data"  << endl;
    return rmse;
  }

  //accumulate squared residuals
  VectorXd error_sum(4);
  error_sum << 0,0,0,0;
  for(int i=0; i < estimations.size(); ++i){
    VectorXd residual = estimations[i]-ground_truth[i];
    residual = residual.array() * residual.array();
    error_sum+=residual;
  }

  //calculate the mean
  VectorXd mean = error_sum/estimations.size();

  //calculate the squared root

  rmse = mean.array().sqrt();

  return rmse;

}
