#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>

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

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI/8;

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

  n_x_ = x_.size();
  n_aug_ = n_x_ + 2; //+ : add process noise to augment : [v_a, v_phi_dd]
  lambda_ = 3 - n_aug_;
  int n_sigma = 2 * n_aug_+1;

  is_initialized_ = false;

  weights_ = VectorXd(n_sigma);

  Xsig_pred_ = MatrixXd(n_x_,n_sigma);
  time_us_ = 0;

  std::ofstream lidar_out_file ("lidarNIS.txt");

  lidar_out_file << 0 << std::endl;

  std::ofstream radar_out_file ("radarNIS.txt");

  radar_out_file << 0 << std::endl;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
   */
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_)
  {
    /**
      TODO:
     * Initialize the state x_ with the first measurement.
     * Create the covariance matrix.
     * Remember: you'll need to convert radar from polar to cartesian coordinates.
     */
    // first measurement
    cout << "UKF: " << endl;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      /**
        Convert radar from polar to cartesian coordinates and initialize state.
       */
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];

      float x = rho * cos(phi);
      float y = rho * sin(phi);

      x_ << x,y,0,0,0;

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
        Initialize state.
       */
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    }
    double px = x_[0];
    double py = x_[1];

    if(fabs(px) < 0.0001){
        x_[0] = 0.01;
        cout << "init px too small" << endl;
    }

    if(fabs(py) < 0.0001){
        x_[1] = 0.01;
        cout << "init py too small" << endl;
    }

    P_ = MatrixXd::Identity(5,5);

    //set weights
    weights_[0]=lambda_/(lambda_+n_aug_);
    int n_sigma = 2 * n_aug_ + 1;
    for(int i=1;i<n_sigma;++i)
    {
      weights_[i]=0.5f/(lambda_+n_aug_);
    }

    time_us_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt - expressed in seconds
  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  {
    UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
  {
    // Laser updates
    UpdateLidar(meas_package);
  }

  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;



}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  int n_sigma = 2*n_aug_+1;

  /*****************************************************
   * Generate sigma Points
   *****************************************************/

  //set first column of sigma point matrix
  Xsig_pred_.col(0)  = x_;

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  //set remaining sigma points
  for (int i = 0; i < n_x_; i++)
  {
    Xsig_pred_.col(i+1)     = x_ + sqrt(lambda_+n_x_) * A.col(i);
    Xsig_pred_.col(i+1+n_x_) = x_ - sqrt(lambda_+n_x_) * A.col(i);
  }

  /*****************************************************
   * UKF augmentation
   *****************************************************/

  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.fill(0);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sigma);

  //create augmented mean state

  x_aug << x_ , 0 ,0 ;

  //create augmented covariance matrix

  P_aug.topLeftCorner(P_.cols(),P_.rows()) = P_;


  MatrixXd Q = MatrixXd(2, 2);
  Q << std_a_*std_a_, 0,
      0, std_yawdd_*std_yawdd_;

  P_aug.bottomRightCorner(Q.rows(),Q.cols()) = Q;

  //create square root matrix
  A = P_aug.llt().matrixL();

  //create augmented sigma points

  //set first column of sigma point matrix
  Xsig_aug.col(0)  = x_aug;

  //set remaining sigma points
  for (int i = 0; i < n_aug_; ++i)
  {
    Xsig_aug.col(i+1)     = x_aug + sqrt(lambda_+n_aug_) * A.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * A.col(i);
  }

  /*****************************************************
   * Sigma Point Prediction
   *****************************************************/

   float delta_t_squared = delta_t*delta_t;
  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

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
    px_p = px_p + 0.5*nu_a*delta_t_squared * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t_squared * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t_squared;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  /*****************************************************
   * Predict Mean and Covariance
   *****************************************************/



  //predict state mean
  VectorXd mean = Xsig_pred_*weights_;
  x_ = mean;
  //predict state covariance matrix
  MatrixXd dev = MatrixXd(n_x_, n_sigma);
  MatrixXd tmp = MatrixXd(n_x_, n_sigma);

  for(int i=0;i< n_sigma;++i)
  {
    dev.col(i) = Xsig_pred_.col(i)-mean;
    tmp.col(i) = dev.col(i)*weights_[i];
  }
  P_ = tmp*dev.transpose();

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

  /*****************************************************
   * Predict measurement
   *****************************************************/

  int n_z = 2;
  int n_sigma = 2 * n_aug_ + 1;

  MatrixXd Zsig = Xsig_pred_.block(0,0,n_z,n_sigma);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //calculate mean predicted measurement
  MatrixXd mean = Zsig*weights_;
  z_pred = mean;

  //calculate measurement covariance matrix S
  MatrixXd R(2,2);
  R << std_laspx_*std_laspx_, 0,
      0, std_laspy_*std_laspy_;

  MatrixXd dev = MatrixXd(n_z, n_sigma);
  MatrixXd tmp = MatrixXd(n_z, n_sigma);

  for(int i=0;i< n_sigma;++i)
  {
    dev.col(i) = Zsig.col(i)-mean;
    //angle normalization
    while (dev(1,i)> M_PI) dev(1,i)-=2.*M_PI;
    while (dev(1,i)<-M_PI) dev(1,i)+=2.*M_PI;

    tmp.col(i) = dev.col(i)*weights_[i];
  }
  S = tmp*dev.transpose() + R;

  /*****************************************************
   * Update State
   *****************************************************/

  //create measurement vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);

  z <<
      meas_package.raw_measurements_[0],   //x in m
      meas_package.raw_measurements_[1];   //y in rad

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  MatrixXd x_dev = MatrixXd(n_x_, n_sigma);
  MatrixXd z_dev = MatrixXd(n_z, n_sigma);
  tmp = MatrixXd(n_x_, n_sigma);

  for(int i=0;i< n_sigma;++i)
  {
    x_dev.col(i) = Xsig_pred_.col(i)-x_;
    //angle normalization
    while (x_dev(3,i)> M_PI) x_dev(3,i)-=2.*M_PI;
    while (x_dev(3,i)<-M_PI) x_dev(3,i)+=2.*M_PI;
    tmp.col(i) = x_dev.col(i)*weights_[i];

    z_dev.col(i) = Zsig.col(i)-z_pred;

  }
  Tc = tmp*z_dev.transpose();

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //update state mean and covariance matrix
  VectorXd y = z-z_pred;
  x_ = x_ + K*y;

  P_ = P_ - K*S*K.transpose();

  /*****************************************************
   * Calculate NIS
   *****************************************************/

   double NIS = y.transpose()*S.inverse()*y;

   ofstream outfile;

  outfile.open("lidarNIS.txt", std::ios_base::app);
  outfile << NIS << endl;
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

  /*****************************************************
   * Predict measurement
   *****************************************************/

  int n_z = 3;
  int n_sigma = 2 * n_aug_ + 1;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sigma);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //transform sigma points into measurement space
  for(int i=0;i < Xsig_pred_.cols();++i)
  {
    VectorXd tmp = Xsig_pred_.col(i);
    float px = tmp[0];
    float py = tmp[1];
    float v = tmp[2];
    float phi = tmp[3];
    float px_squared = px*px;
    float py_squared = py*py;

    Zsig(0,i) = sqrt(px_squared+py_squared);
    Zsig(1,i) = atan2(py,px);
    Zsig(2,i) = (px*cos(phi)*v + py*sin(phi)*v)/Zsig(0,i);
  }

  //calculate mean predicted measurement
  MatrixXd mean = Zsig*weights_;
  z_pred = mean;

  //calculate measurement covariance matrix S
  MatrixXd R(3,3);
  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
       0, 0, std_radrd_*std_radrd_;

  MatrixXd z_dev = MatrixXd(n_z, n_sigma);
  MatrixXd tmp = MatrixXd(n_z, n_sigma);

  for(int i=0;i< n_sigma;++i)
  {
    z_dev.col(i) = Zsig.col(i)-mean;
    //angle normalization
    while (z_dev(1,i)> M_PI) z_dev(1,i)-=2*M_PI;
    while (z_dev(1,i)<-M_PI) z_dev(1,i)+=2*M_PI;


    tmp.col(i) = z_dev.col(i)*weights_[i];
  }
  S = tmp*z_dev.transpose() + R;


  /*****************************************************
   * Update State
   *****************************************************/

  //create measurement vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);

  z <<
      meas_package.raw_measurements_[0],   //rho in m
      meas_package.raw_measurements_[1],   //phi in rad
      meas_package.raw_measurements_[2];   //rho_dot in m/s

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  MatrixXd x_dev = MatrixXd(n_x_, n_sigma);
  tmp = MatrixXd(n_x_, n_sigma);

  for(int i=0;i< n_sigma;++i)
  {
    x_dev.col(i) = Xsig_pred_.col(i)-x_;
    //angle normalization
    while (x_dev(3,i)> M_PI)
    {
      x_dev(3,i)-=2*M_PI;
    }
    while (x_dev(3,i)<-M_PI)
    {
      x_dev(3,i)+=2*M_PI;
      sleep(1);
    }
    tmp.col(i) = x_dev.col(i)*weights_[i];
  }
  Tc = tmp*z_dev.transpose();

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //update state mean and covariance matrix
  VectorXd y = z-z_pred;
  while (y(1)> M_PI) y(1)-=2.*M_PI;
  while (y(1)<-M_PI) y(1)+=2.*M_PI;
  x_ = x_ + K*y;

  P_ = P_ - K*S*K.transpose();

  /*****************************************************
   * Calculate NIS
  *****************************************************/

  double NIS = y.transpose()*S.inverse()*y;

  ofstream outfile;

  outfile.open("radarNIS.txt", std::ios_base::app);
  outfile << NIS << endl;

}
