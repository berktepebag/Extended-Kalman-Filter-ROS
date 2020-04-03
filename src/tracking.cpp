#include "extended_kalman_filter/tracking.h"
#include <iostream>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <math.h>

bool DEBUG = false;

Tracking::Tracking()
{
    ros::NodeHandle nh;

    is_initialized_ = false;
    previous_timestamp_  = ros::Time::now(); // zero

    // State Matrix
    ekf_.x_ = Eigen::VectorXd(8);

    // State Covarianc Matrix
    ekf_.P_ = Eigen::MatrixXd(8,8);
    ekf_.P_ <<  1, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0, 0, 0,
                0, 0, 0, 10000, 0, 0, 0, 0,
                0, 0, 0, 0, 10000, 0, 0, 0,
                0, 0, 0, 0, 0, 10000, 0, 0,
                0, 0, 0, 0, 0, 0, 10000, 0,
                0, 0, 0, 0, 0, 0, 0, 10000;
    
    // State Transition Matrix
    ekf_.F_ = Eigen::MatrixXd(8,8);
    ekf_.F_ <<  1, 0, 0, 1, 0, 0, 1, 0,
                0, 1, 0, 0, 1, 0, 0, 1,
                0, 0, 1, 0, 0, 1, 0, 0,
                0, 0, 0, 1, 0, 0, 1, 0,
                0, 0, 0, 0, 1, 0, 0, 1,
                0, 0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 0, 0, 1;


    // Process Covariance Matrix
    if (!nh.getParam ("ekf/Qv_ax", Qv_ax)){ Qv_ax = 1;}       
    if (!nh.getParam ("ekf/Qv_ay", Qv_ay)){ Qv_ay = 1;}       
    if (!nh.getParam ("ekf/Qv_psi", Qv_psi)){ Qv_psi = 1;}      

    // Wheel Encoder Measurement Covariance Matrix
    if (!nh.getParam ("ekf/R_WE_ax", R_WE_ax)){ R_WE_ax = 0.00225;}       
    if (!nh.getParam ("ekf/R_WE_ay", R_WE_ay)){ R_WE_ay = 0.00225;}       
    if (!nh.getParam ("ekf/R_WE_psidot", R_WE_psidot)){ R_WE_psidot = 0.00225;}   

    // IMU Measurement Covariance Matrix
    if (!nh.getParam ("ekf/R_IMU_ax", R_IMU_ax)){ R_IMU_ax = 0.00225;}       
    if (!nh.getParam ("ekf/R_IMU_ay", R_IMU_ay)){ R_IMU_ay = 0.00225;}       
    if (!nh.getParam ("ekf/R_IMU_psidot", R_IMU_psidot)){ R_IMU_psidot = 0.00225;}   


    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 30);   

}

Tracking::~Tracking(){}

void Tracking::ProcessMeasurement(const std::string sensorType, const Eigen::VectorXd measurements)
{
    // Set sensor type
    ekf_.sensorType_ = sensorType;

    if(!is_initialized_)
    {
        if(DEBUG) ROS_INFO("Initializing Tracking");
        // Set current everything to zero
        ekf_.x_ << 0, 0, 0, 0, 0, 0, 0, 0;

        // previous_timestamp_ = measurement_package.timestamp_;
        previous_timestamp_ = ros::Time::now();

        is_initialized_ = true;
        return;
    }
    // time between last sensor reading and current reading
    // float dt = (measurement_package.timestamp_ - previous_timestamp_) / 1000000.0;
    current_timestamp_ = ros::Time::now();
    time_diff_ = current_timestamp_ - previous_timestamp_;
    float dt = time_diff_.toSec();
    previous_timestamp_ = current_timestamp_;
    // ROS_INFO("dt: %.2f",dt);

    double dt2_2 = dt * dt / 2;

    //x
    ekf_.F_(0,3) = dt;
    ekf_.F_(0,6) = dt2_2;
    //v_x
    ekf_.F_(3,6) = dt;
    //y
    ekf_.F_(1,4) = dt;
    ekf_.F_(1,7) = dt2_2;
    //v_y
    ekf_.F_(4,7) = dt;
    // psi
    ekf_.F_(2,5) = dt;

    Eigen::MatrixXd G = Eigen::MatrixXd(8,3);
    G <<    dt2_2 ,0 ,0,
            0, dt2_2 ,0 ,
            0, 0, dt2_2,
            dt, 0, 0,
            0, dt, 0,
            0, 0, dt,
            1, 0, 0,
            0, 1, 0;

    Eigen::MatrixXd Qv = Eigen::MatrixXd(3,3);
    Qv <<   Qv_ax*Qv_ax, 0, 0,
            0, Qv_ay*Qv_ay, 0,
            0, 0, Qv_psi*Qv_psi;

    ekf_.Q_ = G * Qv * G.transpose();

    if(DEBUG) ROS_INFO("Predicting...");
    ekf_.Predict();
    if(DEBUG) ROS_INFO("Predict Complete...");

    
    if(sensorType == "WHEEL_ENCODER")
    {
        if(DEBUG) ROS_INFO("Setting WHEEL_ENCODER");
        ekf_.H_ = Eigen::MatrixXd(3,8);
        ekf_.H_ <<  0,0,0,1,0,0,0,0,
                    0,0,0,0,1,0,0,0,
                    0,0,0,0,0,1,0,0; 
              
        ekf_.R_ = Eigen::MatrixXd(3,3);
        ekf_.R_ <<  R_WE_ax,0, 0,
                    0, R_WE_ay, 0,
                    0, 0, R_WE_psidot;
                    
    }    
    else if(sensorType == "IMU")
    {
        if(DEBUG) ROS_INFO("Setting IMU");
        //H for IMU
        ekf_.H_ = Eigen::MatrixXd(3,8);
        ekf_.H_ <<  0, 0, 0, 0, 0, 0, 1, 0,
                    0, 0, 0, 0, 0, 0, 0, 1,
                    0, 0, 0, 0, 0, 1, 0, 0;

        
        ekf_.R_ = Eigen::MatrixXd(3,3);
        ekf_.R_ <<  R_IMU_ax, 0, 0,
                    0, R_IMU_ay, 0,
                    0, 0, R_IMU_psidot;
        // std::cout << "imu " << ekf_.H_ << std::endl;

    }
    
    // ROS_INFO("Updating...");
    ekf_.Update(measurements);

    // Set psi to zero if bigger than 360 dgr
    if(fabs(ekf_.x_[2]) > 2*M_PI) ekf_.x_[2] = 0;
    
    ROS_INFO("x: %.2f y: %.2f psi: %.2f dgr", ekf_.x_[0],ekf_.x_[1],ekf_.x_[2]*180/3.14);
    ROS_INFO("v_x: %.2f v_y: %.2f v_psi: %.4f dgr/s", ekf_.x_[3],ekf_.x_[4],ekf_.x_[5]*180/3.14);
    ROS_INFO("***********");
    // std::cout << "x_= " << ekf_.x_ << std::endl;
    // std::cout << "P_= " << ekf_.P_ << std::endl;

    PublishOdom();
}

void Tracking::PublishOdom()
{
    // Publish transformation and position
    odom_quat = tf::createQuaternionMsgFromYaw(ekf_.x_[2]);
    
    odom_trans.header.stamp = current_timestamp_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = ekf_.x_[0];
    odom_trans.transform.translation.y = ekf_.x_[1];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    odom.header.stamp = current_timestamp_;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = ekf_.x_[0];
    odom.pose.pose.position.y = ekf_.x_[1];
    odom.pose.pose.position.z = 0.0;    
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = ekf_.x_[3];
    odom.twist.twist.linear.y = ekf_.x_[4];
    odom.twist.twist.angular.z = ekf_.x_[5];

    //publish the message
    odom_pub.publish(odom);
}