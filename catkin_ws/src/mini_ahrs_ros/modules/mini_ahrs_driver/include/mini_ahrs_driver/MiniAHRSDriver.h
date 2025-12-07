#include <string>
#include <serial/serial.h>
#include <mini_ahrs_driver/config.h>
#include <functional>
#include <iostream>
#include <thread>
#include <chrono>

#include <mini_ahrs_driver/MiniAHRSPacket.h>

namespace mini_ahrs_driver {

struct AHRSUserDefinednData {
    double low_res_time; 
    double hi_res_time; 

    double roll_degrees;
    double pitch_degrees;
    double yaw_degrees;

    double roll_hr_degrees;
    double pitch_hr_degrees;
    double yaw_hr_degrees;

    double qx;
    double qy;
    double qz;
    double qw;

    double gyro_x;
    double gyro_y;
    double gyro_z;

    double gyro_hr_x;
    double gyro_hr_y;
    double gyro_hr_z;

    double acc_x;
    double acc_y;
    double acc_z;

    double acc_hr_x;
    double acc_hr_y;
    double acc_hr_z;

    double mag_x;
    double mag_y;
    double mag_z;

    double voltage;
    double temperature;
    bool usw[16];
};

struct AHRSOrientationData {
    double roll_degrees;
    double pitch_degrees;
    double yaw_degrees;
    double gyro_x;
    double gyro_y;
    double gyro_z;
    double acc_x;
    double acc_y;
    double acc_z;
    double mag_x;
    double mag_y;
    double mag_z;
    double temperature;

    double time;
}; // struct AHRSOrientationData

class MiniAHRSDriver
{
public:
    bool have_device_info_;
    bool run_polling_thread_;
    bool have_device_params_;

    // can be configured in the windows UI
    int data_rate_hz_;
    int initial_alignment_time_seconds_;

    // these depend on the measurement ranges from your gyroscopre (see the ICD)
    double KA_;
    double KG_;

    std::string device_serial_number_;
    std::string device_firmware_version_;

    MiniAHRSDriver(std::string serial_port_path, int baudrate, double KA, double KG, bool verbose);
    bool start();
    void stop();
    void setCallback(std::function<void(const AHRSOrientationData&)> cb);

    void AHRSDataPollingLoop();

    bool isRunning();

    ~MiniAHRSDriver();
    serial::Serial serial_connection_;
private:
    bool verbose_;

    std::function<void(AHRSOrientationData)> data_callback_;
    std::thread worker_thread_;

    HeaderData parseHeader(const std::vector<uint8_t>& data);
    void handleGetDeviceInfoResponse(const std::vector<uint8_t>& message_body);
    void handleReadParamsCommandResponse(const std::vector<uint8_t>& message_body);
    AHRSOrientationData parseOrientationData(const std::vector<uint8_t>& data);
    
    AHRSOrientationData parseUserDefinedData(const std::vector<uint8_t>& data);
    AHRSUserDefinednData parseUserDefinedDataBody (const std::vector<uint8_t>& data, const std::vector<uint8_t>& data_list);

    void workerThreadMain();
}; // class MiniAHRSDriver

} // namespace mini_ahrs_driver
