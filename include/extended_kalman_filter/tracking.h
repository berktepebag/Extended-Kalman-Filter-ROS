#ifndef TRACKING_H_
#define TRACKING_H_

#include "extended_kalman_filter/kalman_filter.h"
#include "extended_kalman_filter/measurement_package.h"
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class Tracking{
    public:
        Tracking();
        virtual ~Tracking();

        void ProcessMeasurement(const std::string sensorType,const Eigen::VectorXd measurements);
        ExtKalmanFilter ekf_;

        void PublishOdom();

    private:

        bool is_initialized_;
        // int64_t previous_timestamp_;
        
        ros::Time current_timestamp_, previous_timestamp_;
        ros::Duration time_diff_;

        //acceleration and rotation noise components
        float Qv_ax,Qv_ay,Qv_psi;
        // Wheel Encoder noise components
        float R_WE_ax, R_WE_ay, R_WE_psidot;
        // IMU noise components
        float R_IMU_ax, R_IMU_ay, R_IMU_psidot;

        ros::Publisher odom_pub;   

        // Odometry Publisher
        geometry_msgs::Quaternion odom_quat;
        geometry_msgs::TransformStamped odom_trans;
        
        tf::TransformBroadcaster odom_broadcaster;

        nav_msgs::Odometry odom;

};

#endif