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

  if(estimations.size() != ground_truth.size()
  		|| estimations.size() == 0){
  	cout << "invalid estimation or ground_truth data" << endl;
  	return rmse;
  }

  for(unsigned int i=0; i < estimations.size(); ++i){

  	VectorXd residual = estimations[i] - ground_truth[i];

  	residual = residual.array()*residual.array();
  	rmse += residual;
  }

  rmse = rmse/estimations.size();

  rmse = rmse.array().sqrt();

  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  //MatrixXd Hj(3, 4);
  MatrixXd Hj = MatrixXd::Zero(3, 4); //new version 0625 per forum mentioned below
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float denone = px*px+py*py;

  if(fabs(denone) < 0.0001){  //new per forums RADAR updates are messing up
    denone = 0.001;
  }

  float dentwo = sqrt(denone);
  float denthree = denone*dentwo;

  if(fabs(denone) < 0.0001){
  	cout << "CalculateJacobian () - Error - Division by Zero" << endl;
  	return Hj;
  }

  Hj << px/dentwo, py/dentwo, 0, 0,
  	 -py/denone, px/denone, 0, 0, 
  	 (py*(vx*py-vy*px))/denthree, (px*(vy*px-vx*py))/denthree, px/dentwo, py/dentwo;

  return Hj;


}
