/*
 * kalman_filter.h
 *
 *  Created on: Apr 5, 2017
 *      Author: amehrpour
 */

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:
	VectorXd x_; // State vector
	MatrixXd P_; // State covariance matrix
	MatrixXd F_; // State transition matrix
	MatrixXd Q_; // Process covariance matrix
	MatrixXd H_; // Measurement matrix
	MatrixXd R_; // Measurement covariance matrix

	KalmanFilter();

	virtual ~KalmanFilter();

	/**
	 * Predict the state and state covariance using the process model
	 */
	void Predict();

	/**
	 * Update the state
	 * @param z The mesurement at k+1
	 */
	void Update(const VectorXd &z);
};

#endif /* KALMAN_FILTER_H_ */
