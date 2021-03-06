//============================================================================
// Name        : UnscentedKalmanFilter.cpp
// Author      : Ali Mehrpour
// Version     :
// Copyright   : Volcano@ 2017
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "UnscentedKalmanFilter.h"

using namespace std;

UnscentedKalmanFilter::UnscentedKalmanFilter() { }

UnscentedKalmanFilter::~UnscentedKalmanFilter() { }

void UnscentedKalmanFilter::Init() { }

void UnscentedKalmanFilter::GenerateSigmaPoints(MatrixXd &Xsig_out) {
  // Set state dimension
  int n_x = 5;

  // Define spreading parameter
  double lambda = 3 - n_x;

  // Set example state
  VectorXd x = VectorXd(n_x);
  x << 5.7441,
       1.3800,
       2.2049,
       0.5015,
       0.3528;

  // Set example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P <<  0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
       -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
        0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
       -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
       -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  // Create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

  // Calculate square root of P
  MatrixXd A = P.llt().matrixL();

  // Set first column of sigma point matrix
  Xsig.col(0) = x;

  // Set remaining sigma points
  for (int i = 0; i < n_x; i++) {
    Xsig.col(i+1) = x + sqrt(lambda + n_x) * A.col(i);
    Xsig.col(i+1 + n_x) = x - sqrt(lambda + n_x) * A.col(i);
  }

  // Set the output matrix
  Xsig_out = Xsig;
}

void UnscentedKalmanFilter::AugmentedSigmaPoints(MatrixXd &Xsig_out) {
  // Set state dimension
  int n_x = 5;

  // Set augmented dimension
  int n_aug = 7;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yaw = 0.2;

  // Define spreading parameter
  double lambda = 3 - n_aug;

  // Set example state
  VectorXd x = VectorXd(n_x);
  x <<   5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;

  // Create example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  // Create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug);

  // Create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug, n_aug);

  // Create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  // Create augmented mean state
  x_aug.head(5) = x;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // Created augmented covariance matrix
  MatrixXd Q = MatrixXd(2, 2);
  Q << std_a*std_a, 0,
       0, std_yaw*std_yaw;

  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x, n_x) = P;
  P_aug.bottomRightCorner(2, 2) = Q;

  // Calculate square root of P_aug
  MatrixXd L = P_aug.llt().matrixL();

  // Set first column of sigma point matrix
  Xsig_aug.col(0) = x_aug;

  // Set remaining sigma points
  for (int i = 0; i < n_aug; i++) {
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda + n_x) * L.col(i);
    Xsig_aug.col(i+1 + n_aug) = x_aug - sqrt(lambda + n_x) * L.col(i);
  }

  Xsig_out = Xsig_aug;
}

void UnscentedKalmanFilter::SigmaPointPrediction(MatrixXd &Xsig_out) {
  // Set state dimension
  int n_x = 5;

  // Set augmented dimension
  int n_aug = 7;

  // Create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
     Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  // Create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  double delta_t = 0.1; // Time diff in sec

  for(int i = 0; i < 2 * n_aug + 1; i++) {
    // Extract values for better readability
    double px = Xsig_aug(0,i);
    double py = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // Predicted state values
    double px_p, py_p;
    if (fabs(yawd) > 0.001) { // avoid division by zero
      px_p = px + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
      py_p = py + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
    }
    else {
      px_p = px + v*delta_t*cos(yaw);
      py_p = py + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // Add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t*cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t*sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // Write predicted simga into right columns
    Xsig_pred(0, i) = px_p;
    Xsig_pred(1, i) = py_p;
    Xsig_pred(2, i) = v_p;
    Xsig_pred(3, i) = yaw_p;
    Xsig_pred(4, i) = yawd_p;

  }

  Xsig_out = Xsig_pred;
}

void UnscentedKalmanFilter::PredictMeanAndCovariance(VectorXd &x_out, MatrixXd &P_out) {
  // Set state dimension
  int n_x = 5;

  // Set augmented dimension
  int n_aug = 7;

  // Define spreading paramter
  double lambda = 3 - n_aug;

  //create example matrix with predicted sigma points
   MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
   Xsig_pred <<
          5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
           2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
          0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
           0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

   //create vector for weights
   VectorXd weights = VectorXd(2*n_aug+1);

   //create vector for predicted state
   VectorXd x = VectorXd(n_x);

   //create covariance matrix for prediction
   MatrixXd P = MatrixXd(n_x, n_x);

   // Set weights
   double weight_0 = lambda/(lambda+n_aug);
   weights(0) = weight_0;
   for(int i = 1; i < 2*n_aug+1; i++) {
     double weight = 0.5/(lambda+n_aug);
     weights(i) = weight;
   }

   // Predict state mean
   x.fill(0.0);
   for(int i = 0; i < 2*n_aug+1; i++) { // Iterate over sigma points
     x = x + weights(i)*Xsig_pred.col(i);
   }

   // Predict state covariance matrix
   P.fill(0.0);
   for(int i = 0; i < 2*n_aug+1; i++) {
     VectorXd x_diff = Xsig_pred.col(i) - x;

     while(x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
     while(x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;

     P = P + weights(i) * x_diff * x_diff.transpose();
   }

   x_out = x;
   P_out = P;
}

void UnscentedKalmanFilter::PredictRadarMeasurement(VectorXd &z_out, MatrixXd &S_out) {
  // Set state dimension
  int n_x = 5;

  // Set augmented dimension
  int n_aug = 7;

  // Define spreading paramter
  double lambda = 3 - n_aug;

  // Set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // Set weights
  VectorXd weights = VectorXd(2*n_aug+1);
  double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for(int i = 1; i < 2*n_aug+1; i++) {
   double weight = 0.5/(lambda+n_aug);
   weights(i) = weight;
  }

  // Radar measurement noise standard deviation radius in m
  double std_radius_r = 0.3;

  // Radar measurement noise standard deviation angle in rad
  double std_radius_phi = 0.0175;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radius_r_dot = 0.1;

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
          5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
           2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
          0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
           0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  // Transform sigma points into measurement space
  for(int i = 0; i < 2*n_aug+1; i++) {
    double px = Xsig_pred(0, i);
    double py = Xsig_pred(1, i);
    double v = Xsig_pred(2, i);
    double yaw = Xsig_pred(3, i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // Measurement model
    Zsig(0, i) = sqrt(px*px + py*py);                      // r
    Zsig(1, i) = atan2(py, px);                            // phi
    Zsig(2, i) = (px*v1 + py*v2 ) / sqrt(px*px + py*py);   //r_dot
  }

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for(int i = 0; i < 2*n_aug+1; i++) {
    z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for(int i = 0; i < 2*n_aug+1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    while(z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
    while(z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  // Add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radius_r*std_radius_r, 0, 0,
       0, std_radius_phi*std_radius_phi, 0,
       0, 0, std_radius_r_dot*std_radius_r_dot;
  S = S + R;

  z_out = z_pred;
  S_out = S;
}
