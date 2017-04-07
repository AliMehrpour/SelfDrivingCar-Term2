/*
 * tracking.h
 *
 *  Created on: Apr 5, 2017
 *      Author: amehrpour
 */

#ifndef TRACKING_H_
#define TRACKING_H_

#include <iostream>
#include <fstream>
#include <string>
#include "kalman_filter.h"
#include "measurement.h"

class Tracking {
public:
	Tracking();
	virtual ~Tracking();
	void ProcessMeasurement(const Measurement &measurement);
	KalmanFilter kf_;

private:
	bool is_initialized_;
	long previous_timestamp_;

	// Acceleration noise components
	float noise_ax;
	float noise_ay;
};

#endif /* TRACKING_H_ */
