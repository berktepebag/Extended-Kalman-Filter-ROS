# Extended Kalman Filter ROS

## A basic implementation of EKF to fuse IMU and Wheel Encoders for Self-Driving-RC-car project

## How to use:
1. Clone the project
2. Modify demo.cpp with your needs. 

**Note:** Demo will not work unless you clone the main self-driving-car-ROS project. 

## Parameters

Modify the parameters as you need: (They can also be found under tracking.cpp)

#### Process Covariance Matrix
1. ekf/Qv_ax 
2. ekf/Qv_ay    
3. ekf/Qv_psi      

#### Wheel Encoder Measurement Covariance Matrix
1. ekf/R_WE_ax    
2. ekf/R_WE_ay  
3. ekf/R_WE_psidot

#### IMU Measurement Covariance Matrix
1. ekf/R_IMU_ax
2. ekf/R_IMU_ay       
3. ekf/R_IMU_psidot   
