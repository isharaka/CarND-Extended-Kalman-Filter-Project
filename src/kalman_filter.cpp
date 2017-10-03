#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
  x_ = VectorXd(4);
  P_ = MatrixXd(4, 4);

  F_ = MatrixXd(4, 4);
  Q_ = MatrixXd(4, 4);

  H_ = MatrixXd(2, 4);
  //R_ = MatrixXd(2, 2);

  x_ << 1, 1, 1, 1;

  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
        
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  MatrixXd Hj = MatrixXd(3, 4);
  Hj = Tools::CalculateJacobian(x_);

  VectorXd Hx = VectorXd(3);
  Tools::Cartesian2Polar(x_, Hx);

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  
  VectorXd y = z - Hx;
  y[1] = Tools::NormaliseAngle(y[1]);
  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  x_ = x_ + (K * y);
  P_ = (I - K * Hj) * P_;
}
