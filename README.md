### *** Archieved: As this repo is and will not updated anymore, it is archieved. ***

# Extended Kalman Filter ROS

## A basic implementation of EKF to fuse IMU and Wheel Encoders for Self-Driving-RC-car project

## How to use:
1. Clone the project
2. Modify demo.cpp. 
3. This package depends on Eigen library. Follow the instructions on the <a href="http://eigen.tuxfamily.org/index.php?title=Main_Page" target="_blank">official website</a>. Easiest way to setup is, extracting and copying the files to "usr/include/" folder. 

**Note:** Demo.cpp will not work unless you clone the main self-driving-car-ROS project or change the message types. 

## Parameters

Modify the parameters as you need: (Parameters can be found under tracking.cpp if modifying is necessary.)

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
