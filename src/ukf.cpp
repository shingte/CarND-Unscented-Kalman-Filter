#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI / 4; 

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  is_initialized_ = false;

  time_us_ = 0;

  // Define the measurement noise matrix for LIDAR
  R_laser_ = MatrixXd(2, 2);
  R_laser_.fill(0.0);
  R_laser_(0,0) = std_laspx_ * std_laspx_;
  R_laser_(1,1) = std_laspy_ * std_laspy_;
  

  H_laser_ = MatrixXd(2, 5);
  H_laser_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;

  // Define the measurement noise matrix for RADAR
  R_radar_ = MatrixXd(3,3);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0, std_radrd_ * std_radrd_;

  ///* State dimension
  n_x_ = 5;

  ///* Radar measurement space dimension
  n_z_ = 3;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  ///* Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);

  //create augmented sigma point matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2*n_aug_+1);

  // Set the weights for the sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(0.5 / (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);


  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  //create matrix for sigma points in radar measurement space (for radar update)
  Zsig_ = MatrixXd(n_z_, 2*n_aug_+1);

  // number of a measurement being processed
  counter_ = 0;

  ///* NIS (Normalized Innovation Squared)
  NIS_laser_ = 0;
  NIS_radar_ = 0;

// file to write NIS values into for latter analysis
  NIS_data_file_.open( "NIS_data.csv", ios::out );
  if (!NIS_data_file_.is_open()) {
    std::cerr << "failed to open NIS_data.csv file" << std::endl;
    exit(1);
  }

  // write comment and column headers
  NIS_data_file_ << "# SENSOR_TYPE: " << MeasurementPackage::LASER << " - lidar; "
                                      << MeasurementPackage::RADAR << " - radar\n"
                 << "# NIS: normalized innovation squared floating point value\n\n"
                 << "SENSOR_TYPE NIS" << std::endl;
}

UKF::~UKF() {
  NIS_data_file_.close();
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  double delta_t_s;
  if (!is_initialized_) {
    // the first measurement is handled here
    double px, py, v, yaw, yawd; // these variables will be initialized in radar or lidar conditional branch

    if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // convert from polar to cartesian coordinate system
      double meas_rho     = meas_package.raw_measurements_[0];
      double meas_phi     = meas_package.raw_measurements_[1];

      px   = meas_rho     * cos(meas_phi);
      py   = meas_rho     * sin(meas_phi);
      v    = meas_package.raw_measurements_[2];  
      yaw  = M_PI_2 - meas_phi; // phi is measured relative to Y axis while yaw is relative to X axis
      yawd = 0;                 // not enough measurements to determine
    }
    else if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // initial state in case the first measurement comes from lidar sensor
      px   = meas_package.raw_measurements_[0];
      py   = meas_package.raw_measurements_[1];
      v    = 0; // not enough measurements to determine
      yaw  = 0; // not enough measurements to determine
      yawd = 0; // not enough measurements to determine
    } else {
      std::cerr << "unknown sensor type of measurement or all sensors are turned off"
                << std::endl;
      exit(2);
    }

    // initial state vector
    x_ << px, py, v, yaw, yawd;

    // initial state covariance matrix
    P_.fill(0.0);
    P_.diagonal().setOnes();

    // done initializing, no need to predict or update
    is_initialized_ = true;
  } else {
    // difference in seconds between the current measurement and the previous one
    delta_t_s = (meas_package.timestamp_ - time_us_) / 1000000.0; // us -> s

    Prediction(delta_t_s);

    // the first step in the UKF processing chain (prediction)
    // the second step in the UKF processing chain (update)
    if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      UpdateRadar(meas_package.raw_measurements_);
    } else if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
      UpdateLidar(meas_package.raw_measurements_);
    } else {
      std::cerr << "unknown sensor type of measurement or all sensors are turned off"
                << std::endl;
      exit(3);
    }

    // write normalized innovation squared value to file
    NIS_data_file_ << meas_package.sensor_type_ << ' '
                   << (meas_package.sensor_type_ == MeasurementPackage::LASER ? NIS_laser_ : NIS_radar_)
                   << std::endl;
  }

  // we do not want to do any updates in cases <use_laser=false, sensor=LASER> and <use_radar=false, sensor=RADAR>
  if ( (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) ||
       (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) ) {
    // initialize the time at which the measurement was taken
    time_us_ = meas_package.timestamp_;
    ++counter_; // increase counter of processed measurements

    // print statistics and intermediate results
    std::cout << "Measurement number = " << counter_ << "\n"
              << "Measurement type   = " <<
                  (meas_package.sensor_type_ == MeasurementPackage::LASER ? "LASER": "RADAR") << "\n"
              << "Elapsed Time (s)   = " << delta_t_s * counter_ << "\n"
              << "NIS " << (meas_package.sensor_type_ == MeasurementPackage::LASER ?
                            "lidar          = " : "radar          = ")
                        << (meas_package.sensor_type_ == MeasurementPackage::LASER ? NIS_laser_ : NIS_radar_) << "\n"
              << "x                  =\n" << x_ << "\n"
              << "P                  =\n" << P_ << "\n"
              << "====================" << std::endl;
  }
}



/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  GenerateAugSigmaPoints();

  SigmaPointPrediction(delta_t);

  PredictMeanAndCovariance();


}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const VectorXd &z) {
  MatrixXd H = H_laser_;
  // Compute y, S, and K
  VectorXd y = z - H * x_;
  MatrixXd PHt = P_ * H.transpose();
  MatrixXd S = H * PHt + R_laser_;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = PHt * S_inv;

  // Compute the new estimates for x_ and P_
  x_ = x_ + (K * y);
  P_ -= K * H * P_; // This is more efficient than P = (I - K * H) * P

  // Compute the normalized innovation squared (NIS)
  NIS_laser_ = y.transpose() * S_inv * y;
  //std::cout << "LIDAR NIS: " << NIS_laser_ << std::endl;

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const VectorXd &z) {

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;
    double c0 = sqrt(p_x * p_x + p_y * p_y);
    // Protect against px and py both close to zero
    if(c0 < 0.0001) {
      std::cout << "Warning: L2 norm of (px, py) is too close to zero. Skipping this measurement.";
      return;
    }

    // measurement model
    Zsig_(0,i) = c0;                        //r
    Zsig_(1,i) = atan2(p_y,p_x);            //phi
    Zsig_(2,i) = (p_x*v1 + p_y*v2 ) / c0;   //r_dot
  }

  // Compute the state mean vector in the measurement space, z_pred
  VectorXd z_pred = VectorXd(n_z_);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig_.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z_,n_z_);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points
    //residual
    VectorXd z_diff = Zsig_.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S += R_radar_;

  //* Update Radar Step:

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z_);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

    // compute residual
    VectorXd z_diff = Zsig_.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // Perform angle normalization to make sure that the angle phi is within -Pi and Pi
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = Tc * S_inv;

  // residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ += K * z_diff;
  P_ =  P_ - K*S*K.transpose();

  //calculate NIS
  NIS_radar_ = z_diff.transpose() * S_inv * z_diff;
  //std::cout << "RADAR NIS: " << NIS_radar_ << std::endl;

}

/** Generate the sigma points from the current x_ and P_ */
void UKF::GenerateAugSigmaPoints() {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(7, 7);

  //create augmented covariance matrix
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();
  double scale = sqrt(lambda_+n_aug_);

  //create augmented sigma points
  Xsig_aug_.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug_.col(i+1)        = x_aug + scale * L.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug - scale * L.col(i);
  }

}

/** redict the sigma points */
void UKF::SigmaPointPrediction(const double delta_t) {

  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
}

/** Predict the state mean vector and covariance matrix */
void UKF::PredictMeanAndCovariance() {

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ += weights_(i) * x_diff * x_diff.transpose() ;
  }

}
