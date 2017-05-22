/*
 * UnscentedKalmanFilter.h
 *
 *  Created on: Apr 30, 2017
 *      Author: amehrpour
 */

#ifndef UNSCENTEDKALMANFILTER_H_
#define UNSCENTEDKALMANFILTER_H_

#include "../Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UnscentedKalmanFilter {
public:
  UnscentedKalmanFilter();

  virtual ~UnscentedKalmanFilter();

  void Init();

  void GenerateSigmaPoints(MatrixXd &Xsig_out);

  void AugmentedSigmaPoints(MatrixXd &Xsig_out);

  void SigmaPointPrediction(MatrixXd &Xsig_out);

  void PredictMeanAndCovariance(VectorXd &x_out, MatrixXd &P_out);

  void PredictRadarMeasurement(VectorXd &z_out, MatrixXd &S_out);
};

#endif /* UNSCENTEDKALMANFILTER_H_ */
