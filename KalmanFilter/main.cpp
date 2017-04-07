/*
 * main.cpp
 *
 *  Created on: Apr 5, 2017
 *      Author: amehrpour
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "Eigen/Dense"
#include "measurement.h"
#include "tracking.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

int main() {
	// Step 1. Read measurements
	vector<Measurement> measurements;

	string file_name = "./data/obj_pose-laser-radar-synthetic-input.txt";
	ifstream in_file(file_name, std::ifstream::in);
	if (!in_file.is_open()) {
		cout << "Cannot open input file: " << file_name << endl;
	}

	string line;
	int i = 0;
	int size = 10;
	while (getline(in_file, line) && i <= size) {
		Measurement measurement;

		istringstream iss(line);
		string sensor_type;
		iss >> sensor_type;
		long timestamp;

		if (sensor_type.compare("L") == 0) {
			measurement.sensor_type_ = Measurement::LASER;

			measurement.raw_measurements_ = VectorXd(2);
			float x;
			float y;
			iss >> x;
			iss >> y;
			measurement.raw_measurements_ << x, y;

			iss >> timestamp;
			measurement.timestamp_ = timestamp;

			measurements.push_back(measurement);
		}
		else if (sensor_type.compare("R")) {
			// Skip radar measurement
			continue;
		}

		i++;
	}

	if (in_file.is_open()) {
		in_file.close();
	}

	// Step 2. Process measurements
	Tracking tracking;
	size_t N = measurements.size();
	for (size_t i = 0; i < N; ++i) {
		Measurement m = measurements[i];
		cout << endl << "Process measurement " << i + 1 << " : ";
		//m.toString();
		cout << endl;
		tracking.ProcessMeasurement(m);
	}

	return 0;
}



