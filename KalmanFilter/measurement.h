/*
 * measurement_package.h
 *
 *  Created on: Apr 5, 2017
 *      Author: amehrpour
 */

#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include <iostream>
#include "Eigen/Dense"

using namespace std;

class Measurement {
public:
	long timestamp_;
	enum SensorType {
		LASER, RADAR
	} sensor_type_;
	Eigen::VectorXd raw_measurements_;

	void toString();
};

#endif /* MEASUREMENT_H_ */
