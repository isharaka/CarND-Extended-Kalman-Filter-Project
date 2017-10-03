#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd error = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		error = error.array()*error.array();
		rmse += error;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 
	float c2 = px*px + py*py;
	float c1 = sqrt(c2);
	float c3 = c1 * c2;

	//check division by zero
	if(abs(c2) < 0.0001){
	    cout << "Error - division by 0" << endl;
	    return Hj;
	}
	
	//compute the Jacobian matrix
	Hj << px/c1, py/c1, 0, 0,
	    -py/c2, px/c2, 0, 0,
	    py * (vx*py - vy*px)/c3, px * (vy*px - vx*py)/c3, px/c1, py/c1;

	return Hj;
}

float Tools::NormaliseAngle(const float angle)
{
	float normalised = angle;

	while( normalised > M_PI || normalised < -M_PI )
		normalised -= 2 * M_PI;

	return normalised;
}

int Tools::Cartesian2Polar(const VectorXd& cartesian, VectorXd& polar)
{
	float px = cartesian[0];
	float py = cartesian[1];
	float vx = cartesian[2];
	float vy = cartesian[3];

	polar[0] = sqrt(px*px + py*py);
	polar[1] = NormaliseAngle(atan2(py, px));

	if(abs(polar[0]) < 0.0001){
	    cout << "Error - division by 0" << endl;
	    return -1;
	}

	polar[2] = (px*vx + py*vy)/polar[0];

	return 0;
}

int Tools::Polar2Cartesian(const VectorXd polar, VectorXd& cartesian)
{
	float rho = polar[0];
	float phi = polar[1];

	cartesian[0] = rho * cos(phi);
	cartesian[1] = rho * sin(phi);
	cartesian[2] = 0;
	cartesian[3] = 0;

	return 0;
}
