#pragma once

//#define BT_EULER_DEFAULT_ZYX // sets the default euler to quaternion transform to zyx. 

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pluginlib/class_list_macros.h>

#include <mini_ahrs_driver/MiniAHRSDriver.h>

namespace mini_ahrs_ros {

class MiniAHRSNodelet : public nodelet::Nodelet {
public:
    void onInit();

private:
    void IMUDataCallback(const mini_ahrs_driver::AHRSOrientationData& data);

    std::string serial_port_path_;
    int baudrate_;
    std::string frame_id_;
    std::string parent_frame_id_;
    long long int callback_counter_;
    double KA_;
    double KG_;

    bool connected_;

    ros::Time time_initial;
    double time_diff;
    bool time_initialized;

    ros::Publisher imu_data_publisher_;
    ros::Publisher temperature_publisher_;
    ros::Publisher magnetic_field_publisher_;

    std::unique_ptr<mini_ahrs_driver::MiniAHRSDriver> driver_ptr_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;
}; // class MiniAHRSNodelet

} // namespace miniahrs_ros

PLUGINLIB_EXPORT_CLASS(mini_ahrs_ros::MiniAHRSNodelet, nodelet::Nodelet);
