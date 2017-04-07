/*
 * measurement.cpp
 *
 *  Created on: Apr 5, 2017
 *      Author: amehrpour
 */

#include "measurement.h"

void Measurement::toString() {
	cout << "SensorType: " << sensor_type_ << ", Location: " << raw_measurements_[0] << "," << raw_measurements_[1] << " at " << timestamp_;
}



