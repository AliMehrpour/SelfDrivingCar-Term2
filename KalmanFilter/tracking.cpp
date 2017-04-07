/*
 * tracking.cpp
 *
 *  Created on: Apr 5, 2017
 *      Author: amehrpour
 */

#include <iostream>
#include "Eigen/Dense"
#include "tracking.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tracking::Tracking() {
	is_initialized_ = false;
	previous_timestamp_ = 0;

	// Create a 4D state vector. We don't know the values of the x state
	kf_.x_ = VectorXd(4);

	// State covariance P
	kf_.P_ = MatrixXd(4, 4);
	kf_.P_ << 1, 0, 0, 0,
			 0, 1, 0, 0,
			 0, 0, 1000, 0,
			 0, 0, 0, 1000;

	// Measurement convariance
	kf_.R_ = MatrixXd(2, 2);
	kf_.R_ << 0.0225, 0,
			  0, 0.0225;

	// Measurement matrix
	kf_.H_ = MatrixXd(2, 4);
	kf_.H_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

	// Initial transition matrix
	kf_.F_ = MatrixXd(4, 4);
	kf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

	noise_ax = 5;
	noise_ay = 5;
}

Tracking::~Tracking() { }

void Tracking::ProcessMeasurement(const Measurement &measurement) {
	if (!is_initialized_) {
		kf_.x_ << measurement.raw_measurements_[0], measurement.raw_measurements_[1], 0, 0;
		previous_timestamp_ = measurement.timestamp_;
		is_initialized_ = true;
	}
	else {
		// Compute the time elapsed between current and previous measurements
		long timestamp = measurement.timestamp_;
		float dt = (timestamp - previous_timestamp_) / 1000000.0; // seconds
		previous_timestamp_ = timestamp;

		// Update F matrix with delta time
		kf_.F_(0, 2) = dt;
		kf_.F_(1, 3) = dt;

		// Set process covariance Q
		float dt_2 = dt * dt;
		float dt_3 = dt_2 * dt;
		float dt_4 = dt_3 * dt;

		kf_.Q_ = MatrixXd(4, 4);
		kf_.Q_ << (dt_4/4*noise_ax), 0, (dt_3/2*noise_ax), 0,
				  0, (dt_4/4*noise_ay), 0, (dt_3/2*noise_ay),
				  (dt_3/2*noise_ax), 0, (dt_2*noise_ax), 0,
				  0, (dt_3/2*noise_ay), 0, (dt_2*noise_ay);

		// Predict
		kf_.Predict();

		// Update
		kf_.Update(measurement.raw_measurements_);

		cout << "x_" << endl << kf_.x_ << endl;
		cout << "P_" << endl << kf_.P_ << endl;
	}
}
