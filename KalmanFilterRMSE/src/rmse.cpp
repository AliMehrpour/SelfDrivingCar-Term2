/*
 * rmse.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: amehrpour
 */

#include <iostream>
#include <vector>
#include "../../Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

VectorXd CalculateRSME(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

int main() {
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	// estimations
	VectorXd e(4);
	e << 1, 1, 0.2, 0.1;
	estimations.push_back(e);
	e << 2, 2, 0.3, 0.2;
	estimations.push_back(e);
	e << 3, 3, 0.4, 0.3;
	estimations.push_back(e);

	// ground truth
	VectorXd t(4);
	t << 1.1, 1.1, 0.3, 0.2;
	ground_truth.push_back(t);
	t << 2.1, 2.1, 0.4, 0.3;
	ground_truth.push_back(t);
	t << 3.1, 3.1, 0.5, 0.4;
	ground_truth.push_back(t);

	cout << CalculateRSME(estimations, ground_truth) << endl;

	return 0;
}

VectorXd CalculateRSME(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
	VectorXd rsme(4);
	rsme << 0, 0, 0, 0;

	int size_estimations = estimations.size();
	int size_ground_truth = ground_truth.size();

	if (size_estimations == 0 && size_estimations != size_ground_truth) {
		cout << "Error in input" << endl;
	}
	else {
		for (int i = 0; i < size_estimations; ++i) {
			VectorXd residual = estimations[i] - ground_truth[i];
			residual = (residual.array() * residual.array());
			rsme = rsme + residual;
		}

		rsme = (rsme.array() / size_estimations).array().sqrt();
	}

	return rsme;
}

