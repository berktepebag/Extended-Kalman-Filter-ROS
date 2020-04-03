#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

class ExtKalmanFilter{
    public:

    ExtKalmanFilter();

    virtual ~ExtKalmanFilter();

    // Predict the x_kp and P_;
    void Predict();

    std::string sensorType_;

    /**
     *Measurement-Update, updates the state vector x_k
     *@param z The measurement at k
     *using the process model
    **/
    void Update(const Eigen::VectorXd &z);

    // state vector at k-1
    Eigen::VectorXd x_;

    // state transition matrix
    Eigen::MatrixXd F_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;


};

#endif 