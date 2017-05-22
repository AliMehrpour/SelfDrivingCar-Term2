/*
 * main.cpp
 *
 *  Created on: Apr 30, 2017
 *      Author: amehrpour
 */

#include <iostream>
#include "../Eigen/Dense"
#include <vector>
#include "UnscentedKalmanFilter.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

int main() {

  UnscentedKalmanFilter ukf;

  int choice = 4;

  if (choice == 0) {
    // Generate sigma points
    MatrixXd Xsig = MatrixXd(5, 11);
    ukf.GenerateSigmaPoints(Xsig);
    cout << "Xsig = " << endl << Xsig << endl;
  }
  else if (choice == 1) {
    // Augment sigma points
    MatrixXd Xsig_aug = MatrixXd(7, 15);
    ukf.AugmentedSigmaPoints(Xsig_aug);
    cout << "Xsig_aug = " << endl << Xsig_aug << endl;
  }
  else if (choice == 2) {
    // Predict sigma points
    MatrixXd Xsig_pred = MatrixXd(5, 15);
    ukf.SigmaPointPrediction(Xsig_pred);
    cout << "Xsig_pred = " << endl << Xsig_pred << endl;
  }
  else if (choice == 3) {
    // Predict mean and convariance
    VectorXd x_pred = VectorXd(5);
    MatrixXd P_pred = MatrixXd(5, 5);
    ukf.PredictMeanAndCovariance(x_pred, P_pred);

    cout << "Predicted state" << endl;
    cout << x_pred << endl;
    cout << "Predicted covariance matrix" << endl;
    cout << P_pred << endl;
  }
  else if (choice == 4) {
    // Predict radar measurement
    VectorXd z_out = VectorXd(3);
    MatrixXd S_out = MatrixXd(3, 3);
    ukf.PredictRadarMeasurement(z_out, S_out);

    cout << "Z_pred: " << endl;
    cout << z_out << endl;
    cout << "S: " << endl;
    cout << S_out << endl;
  }

  return 0;
}



