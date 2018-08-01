#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Defining H matrix for laser updates
  MatrixXd H_laser_;

  //* Defining measurement covariance matrix - laser
  MatrixXd R_laser_;

  // The measurement noise matrix for RADAR
  MatrixXd R_radar_;

  ///* State dimension
  int n_x_;

  ///* Radar measurement space dimension
  int n_z_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* create augmented sigma point matrix
  MatrixXd Xsig_aug_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  //create matrix for sigma points in radar measurement space (for radar update)
  MatrixXd Zsig_;

  unsigned long long counter_;
  
  ///* NIS (Normalized Innovation Squared)
  double NIS_laser_;
  double NIS_radar_;

  ///* File to write NIS values into
  std::ofstream NIS_data_file_;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  //void ProcessMeasurement(const MeasurementPackage &meas_package);
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);


  /**
   * Generates Augmented Sigma Points
   */
  void GenerateAugSigmaPoints();

  /**
   * Prediction of new sigma points at time k+1.
   */
  void SigmaPointPrediction(double delta_t);
  
  
  /**
   * Predict Mean and Covariance based on predicted sigma points at k+1.
   */
  void PredictMeanAndCovariance();
  


  /**
   * Updates the state and the state covariance matrix using a laser measurement
   */
  void UpdateLidar(const VectorXd &z);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   */
  void UpdateRadar(const VectorXd &z);
};

#endif /* UKF_H */
