#include "extended_kalman_filter/kalman_filter.h"

#include <iostream>
#include <math.h>

// Constructor
ExtKalmanFilter::ExtKalmanFilter(){}
// Destructor
ExtKalmanFilter::~ExtKalmanFilter(){}

void ExtKalmanFilter::Predict()
{
    x_ = F_ * x_ ;
    Eigen::MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void ExtKalmanFilter::Update(const Eigen::VectorXd &z)
{
    Eigen::VectorXd z_pred =  H_ * x_;
    Eigen::VectorXd y = z - z_pred;
    
    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd PHt = P_ * Ht;
    Eigen::MatrixXd S = H_ * P_ * Ht + R_;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd K = PHt * Si;

    // Calculate new state
    x_ = x_ + (K * y);
    int x_size = x_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size,x_size);
    P_ = (I - K * H_) * P_;
}
