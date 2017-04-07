/*
 * main.cpp
 *
 *  Created on: Apr 7, 2017
 *      Author: amehrpour
 */

#include <iostream>
#include <vector>
#include "math.h"
#include "../Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd CalculateJacobian(const VectorXd &x_state);

int main() {

	// Predicted state
	VectorXd x_predicted(4);
	x_predicted << 1, 2, 0.2, 0.4;

	MatrixXd Hj = CalculateJacobian(x_predicted);

	cout << "Hj: " << endl << Hj << endl;

	return 0;
}

MatrixXd CalculateJacobian(const VectorXd &x_state) {
	MatrixXd Hj(3, 4);

	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	if (px == 0 || py == 0) {
		cout << "Error - Division By Zero" << endl;
		return Hj;
	}

	float pxpy = px*px + py*py;
	float s_pxpy = sqrt(pxpy);
	float c3 = pxpy * s_pxpy;

	Hj(0, 0) = px / s_pxpy;
	Hj(0, 1) = py / s_pxpy;
	Hj(0, 2) = 0;
	Hj(0, 3) = 0;

	Hj(1, 0) = -py/pxpy;
	Hj(1, 1) = px/pxpy;
	Hj(1, 2) = 0;
	Hj(1, 3) = 0;

	Hj(2, 0) = (py * ((vx*py) - (vy*px))) / pow(pxpy, 3/2);
	Hj(2, 1) = (px * ((vy*px) - (vx*py))) / pow(pxpy, 3/2);
	Hj(2, 2) = px / sqrt(pxpy);
	Hj(2, 3) = py / sqrt(pxpy);

	return Hj;
}
