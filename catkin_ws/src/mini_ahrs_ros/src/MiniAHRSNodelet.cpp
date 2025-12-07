#include <cmath>

#include <mini_ahrs_ros/MiniAHRSNodelet.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <chrono>
#include <time.h>

namespace mini_ahrs_ros {

void MiniAHRSNodelet::IMUDataCallback(const mini_ahrs_driver::AHRSOrientationData& data)
{
    if (!connected_) {
        return;
    }
    if (data.time < 0) {
        return;
    }

    if (!time_initialized) {
        time_initial = ros::Time::now();
        time_initialized = true;
        time_diff = time_diff + data.time;
    }
    // auto stamp = ros::Time::now();
    auto stamp = time_initial + ros::Duration(data.time - time_diff);
    // auto stamp_now = ros::Time::now();
    // std::cout<< "time_initial: " << time_initial.toSec() << "data.time: " << data.time << "time_diff: " << time_diff << std::endl;
    // std::cout << std::fixed << std::setprecision(6) <<"Time now: " << stamp_now.toSec() << ", time imu: "<< stamp.toSec() << std::endl;

    double degree_to_radian = M_PI / 180.0;
    double g_to_m_ss = 9.80665;
    double nano_tesla_to_tesla = 1.0 / 1.0e9;

    tf2::Quaternion orientation_quat( // this ordering of the angles produces the correct result when viewed in rviz.
        data.roll_degrees * degree_to_radian,
        data.pitch_degrees * degree_to_radian,
        -data.yaw_degrees * degree_to_radian
    );

    sensor_msgs::Imu imu_output;
    imu_output.header.stamp = stamp;
    imu_output.header.frame_id = frame_id_;
    imu_output.header.seq = callback_counter_;

    imu_output.orientation.x = orientation_quat.x();
    imu_output.orientation.y = orientation_quat.y();
    imu_output.orientation.z = orientation_quat.z();
    imu_output.orientation.w = orientation_quat.w();

    imu_output.linear_acceleration.x = data.acc_x * g_to_m_ss;
    imu_output.linear_acceleration.y = data.acc_y * g_to_m_ss;
    imu_output.linear_acceleration.z = data.acc_z * g_to_m_ss;

    imu_output.angular_velocity.x = data.gyro_x * degree_to_radian;
    imu_output.angular_velocity.y = data.gyro_y * degree_to_radian;
    imu_output.angular_velocity.z = data.gyro_z * degree_to_radian;

    sensor_msgs::Temperature temperature_output;
    temperature_output.temperature = data.temperature;
    temperature_output.header.stamp = stamp;
    temperature_output.header.frame_id = frame_id_;
    temperature_output.header.seq = callback_counter_;

    sensor_msgs::MagneticField mag_output;
    mag_output.header.stamp = stamp;
    mag_output.header.frame_id = frame_id_;
    mag_output.header.seq = callback_counter_;
    mag_output.magnetic_field.x = data.mag_x * nano_tesla_to_tesla;
    mag_output.magnetic_field.y = data.mag_y * nano_tesla_to_tesla;
    mag_output.magnetic_field.z = data.mag_z * nano_tesla_to_tesla;

    imu_data_publisher_.publish(imu_output);
    temperature_publisher_.publish(temperature_output);
    magnetic_field_publisher_.publish(mag_output);

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.seq = callback_counter_;
    transform.header.frame_id = parent_frame_id_;
    transform.child_frame_id = frame_id_;
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = orientation_quat.x();
    transform.transform.rotation.y = orientation_quat.y();
    transform.transform.rotation.z = orientation_quat.z();
    transform.transform.rotation.w = orientation_quat.w();
    tf_broadcaster_.sendTransform(transform);

    ++callback_counter_;
}

void MiniAHRSNodelet::onInit()
{
    connected_ = false;
    
    time_initialized = false;
    time_diff = 0.02;

    ros::NodeHandle& private_node_handle = getMTPrivateNodeHandle();
    callback_counter_ = 0;

    if (!private_node_handle.hasParam("serial_port_path") || !private_node_handle.hasParam("baudrate")) {
        ROS_ERROR_STREAM("The serial_port_path and baudrate parameters must be set.");
        throw std::runtime_error("The serial_port_path and baudrate parameters must be set.");
    }

    private_node_handle.getParam("serial_port_path", serial_port_path_);
    private_node_handle.getParam("baudrate", baudrate_);

    if (!private_node_handle.hasParam("KA") || !private_node_handle.hasParam("KG")) {
        ROS_ERROR_STREAM("The KA and KG parameters must be set! Check the documentation of the AHRS Orientation Data format in the ICD.");
        throw std::runtime_error("The KA and KG parameters must be set! Check the documentation of the AHRS Orientation Data format in the ICD.");
    }

    private_node_handle.getParam("KA", KA_);
    private_node_handle.getParam("KG", KG_);

    bool verbose = true;
    private_node_handle.param<std::string>("frame_id", frame_id_, "MiniAHRS");
    private_node_handle.param<bool>("verbose", verbose, false);
    private_node_handle.param<std::string>("parent_frame_id", parent_frame_id_, "world");

    if (verbose) {
        ROS_INFO_STREAM("Starting miniAHRS driver in verbose mode.");
    }

    imu_data_publisher_ = private_node_handle.advertise<sensor_msgs::Imu>("imu", 1);
    temperature_publisher_ = private_node_handle.advertise<sensor_msgs::Temperature>("temperature", 1);
    magnetic_field_publisher_ = private_node_handle.advertise<sensor_msgs::MagneticField>("magnetic_field", 1);

    ROS_INFO_STREAM("Initializing MiniAHRS driver at serial port " << serial_port_path_ << " with baudrate " << baudrate_);
    try {
	ROS_INFO_STREAM("Initializing the driver with verbose: "<< verbose);
        driver_ptr_ = std::unique_ptr<mini_ahrs_driver::MiniAHRSDriver>(
            new mini_ahrs_driver::MiniAHRSDriver(serial_port_path_, baudrate_, KA_, KG_, verbose)
        ); 
    } catch (const std::exception& error) {
        ROS_ERROR_STREAM("Could not construct driver instance! Error: " << error.what() << std::endl);
        throw error;
    }

    std::function<void(const mini_ahrs_driver::AHRSOrientationData&)> data_callback
        = std::bind(&MiniAHRSNodelet::IMUDataCallback, this, std::placeholders::_1);

    driver_ptr_->setCallback(data_callback);

    bool start_success = driver_ptr_->start();

    if (!start_success) {
        ROS_ERROR_STREAM("Unable to start MiniAHRS driver!");
        throw std::runtime_error("Unable to start MiniAHRS driver!");
    }

    ROS_INFO_STREAM(
        "MiniAHRS unit s/n (" << driver_ptr_->device_serial_number_ << ") publishing at " << driver_ptr_->data_rate_hz_ << " Hz connected!"
    );

    connected_ = true;
}

}
