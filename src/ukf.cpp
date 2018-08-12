#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


double normalize_angle(double angle);

UKF::UKF() {
  is_initialized_ = false;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.2; // TODO: WRONG

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2; // TODO: WRONG

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  // DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  // Project the current state -> LIDAR measurement space
  H_laser_ = MatrixXd(2, 5);
  H_laser_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  P_ = MatrixXd(5, 5); // Initialize the state covariance matrix
  P_ << 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1,
        1, 1, 1, 1, 1,
        1, 1, 1, 1, 1,
        1, 1, 1, 1, 1;
        // 0,   0.1, 0,   0,   0,
        // 0,   0,   0.1, 0,   0,
        // 0,   0,   0,   0.1, 0,
        // 0,   0,   0,   0,   0.1;


  previous_t = 0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  cout << "Entering ProcessMeasurement" << endl;
  if (is_initialized_ == false) {
    cout << "Init..." << endl;
    x_ = VectorXd(5);
    x_ << 0, 0, 0, 0, 0;

    n_x_ = 5;
    n_aug_ = 7;
    lambda_ = 3 - n_x_;

    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      int rho = meas_package.raw_measurements_(0);
      int theta = meas_package.raw_measurements_(1);
      x_(0) = rho * cos(theta);
      x_(1) = rho * sin(theta);
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
    }

    previous_t = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  double delta_t = (meas_package.timestamp_ - previous_t) / 1000000.0;
  previous_t = meas_package.timestamp_;
  Prediction(delta_t);
  cout << "Finished Prediction" << endl;

  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    // UpdateLidar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  }
  cout << "Leaving ProcessMeasurement" << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // TODO: See 7.18 for sigma point creation
  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  cout << "x_ size:" << x_.size() << endl << x_ << endl;
  cout << "P_ size:" << P_.size() << endl << P_ << endl;

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  // Predict using sigma pts
  // 7.21
  // Take sigma pts, pass them through CTRV model, predict new sigma pts
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1); // 5x15
  for (int i = 0; i < 2*n_aug_+1; i++) {
    // extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    // The last three state variables are unchanged if yawd is zero
    double v_p = v + nu_a*delta_t;
    double yaw_p = yaw + yawd * delta_t + 0.5*nu_yawdd*delta_t*delta_t;
    double yawd_p = yawd + nu_yawdd*delta_t;

    // add noise
    px_p += 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p += 0.5*nu_a*delta_t*delta_t * sin(yaw);

    // write predicted sigma point into each column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }


  // You deterministically create a set of sigma pts that represent your distribution
  // Two sigma pts for every axis of your covariance
  // CTRV has a state vector with 5 components (or 7, but 2 are const?)

  // -> Predict mean and covariance
  // 7.24
  weights_ = VectorXd(2*n_aug_+1);
  double weight_0 = lambda_ / (lambda_+n_aug_);
  weights_(0) = weight_0; // Special case for the first weight
  for (int i=1; i<2*n_aug_+1; i++) {
    double weight = 1/(2*(n_aug_+lambda_));
    weights_(i) = weight; // Normal formula for every other weight
  }

  // predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // iterate over sigma points
    x_ = weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = normalize_angle(x_diff(3));

    P_ = weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  // VectorXd z = meas_package.raw_measurements_;
  // VectorXd y = z - H_laser_ * x_; // 2x1 = 2x1 - 2x5 * 5x1
  // MatrixXd Ht = H_laser_.transpose(); // 4x2 = 2x4.T
  // MatrixXd S = H_laser_ * P_ * Ht + R_laser_; // 2x2 = 2x4 * 4x4 * 4x2 + 2x2
  // MatrixXd Si = S.inverse(); // 2x2 = 2x2.inv()
  // MatrixXd PHt = P_ * Ht; // 4x2 = 4x4 * 4x2
  // MatrixXd K = PHt * Si; // 4x2 = 4x2 * 2x2

  // x_ = x_ + (K * y); // 4x1 = 4x1 + (4x2 * 2x1)
  // long x_size = x_.size(); // 4
  // MatrixXd I = MatrixXd::Identity(x_size, x_size); // 4x4
  // P_ = (I - K * H_laser_) * P_; // 4x4 = (4x4 - 4x2 * 2x4) * 4x4

  // TODO: Calculate LIDAR RIS

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  // 7.27
  // Transform sigma pts into measurement space (need to copy sigma pt generation code?)
  //

  int n_z = 3;

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = normalize_angle(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }



  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;


  MatrixXd Tc = MatrixXd(n_x_, n_z); // 5x3
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 sigma points

    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = normalize_angle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    x_diff(3) = normalize_angle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose(); // 5x3 + num * 5x1 * 1x3
  }

  // Kalman gain K
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
  z_diff(1) = normalize_angle(z_diff(1));

  // update state mean and covariance matrix
  return;
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
  cout << "001" << endl;

}


// void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) { }


double normalize_angle(double angle) {
  while (angle > M_PI) {
    angle -= 2*M_PI;
  }
  while (angle < -M_PI) {
    angle += 2*M_PI;
  }
  return angle;
}
