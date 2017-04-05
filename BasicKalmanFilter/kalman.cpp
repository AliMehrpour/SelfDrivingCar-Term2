#include <iostream>
#include <vector>
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

// Kalman filter variables
VectorXd x; // Object state
MatrixXd P; // Object covariance matrix
VectorXd u; // External motion (Noise)
MatrixXd F; // State transition matrix
MatrixXd H; // Measurement matrix
MatrixXd R; // Measurement covariance matrix
MatrixXd I; // Identity matrix
MatrixXd Q; // Process covariance martix

vector<VectorXd> measurements;
void filter(VectorXd &x, MatrixXd &P);

int main() {

	x = VectorXd(2);
	x << 0, 0;

	P = MatrixXd(2, 2);
	P << 1000, 0, 0, 1000;

	u = VectorXd(2);
	u << 0, 0;

	F = MatrixXd(2, 2);
	F << 1, 1, 0, 1;

	H = MatrixXd(1, 2);
	H << 1, 0;

	R = MatrixXd(1, 1);
	R << 1;

	I = MatrixXd::Identity(2, 2);

	Q = MatrixXd(2, 2);
	Q << 0, 0, 0, 0;


	// Create a list of measurements
	VectorXd measurement(1);
	measurement << 1;
	measurements.push_back(measurement);
	measurement << 2;
	measurements.push_back(measurement);
	measurement << 3;
	measurements.push_back(measurement);

	// Call Kalman filter algorithm
	filter(x, P);

	return 0;
}

void filter(VectorXd &x, MatrixXd &P) {
	for (int n = 0; n < measurements.size(); n++) {
		// Measurement update
		VectorXd Z = measurements[n];
		MatrixXd y = Z - H * x; // Error calculation
		MatrixXd S = H * P * H.transpose() + R;
		MatrixXd K = P * H.transpose() * S.inverse();
		
		// New state update
		x = x + (K * y);
		P = (I - K * H) * P;

		// Prediction
		x = (F * x) + u;
		P = F * P * F.transpose();

		cout << "x = " << endl << x << endl;
		cout << "P = " << endl << P << endl;
	}
}