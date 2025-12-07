#include "mini_ahrs_driver/MiniAHRSDriver.h"
#include "mini_ahrs_driver/MiniAHRSPacket.h"

#include <cstdint>
#include <iomanip>

#include <chrono>
#include <iostream>
#include <cstdint>
#include <cmath>

namespace mini_ahrs_driver
{

MiniAHRSDriver::MiniAHRSDriver(std::string serial_port_path, int baudrate, double KA, double KG, bool verbose) : serial_connection_(serial_port_path, baudrate, serial::Timeout::simpleTimeout(SERIAL_CONNECT_TIMEOUT_MS)), run_polling_thread_(false), have_device_params_(false), KA_(KA), KG_(KG), verbose_(verbose), have_device_info_(false)
{
    if (!serial_connection_.isOpen()) {
        throw std::runtime_error(std::string("Could not open serial port at path ") + serial_port_path);
    }
}

bool MiniAHRSDriver::isRunning() {
    return worker_thread_.joinable();
}

void MiniAHRSDriver::stop()
{
    if (isRunning()) {
        run_polling_thread_ = false;
        worker_thread_.join();

        StopDeviceCommandPacket packet;
        serial_connection_.write(packet.buffer);
    }
}

MiniAHRSDriver::~MiniAHRSDriver()
{
    stop();
    serial_connection_.close();
}

void MiniAHRSDriver::setCallback(std::function<void(const AHRSOrientationData&)> cb) {
    data_callback_ = cb;
}

bool MiniAHRSDriver::start() {
    if (verbose_) {
        std::cout << "Entering start() function. serial port available: " << serial_connection_.available() << '\n';
        std::cout << "Writing stop packet" << '\n';
    }

    StopDeviceCommandPacket stop_command;
    serial_connection_.write(stop_command.buffer);

    if (verbose_) {
        std::cout << "Waiting for stop packet to be processed" << '\n';
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    if (verbose_) {
        std::cout << "After stop packet avaiable bytes: " << serial_connection_.available() << '\n';
    }

    serial_connection_.read(serial_connection_.available()); // clear the buf


    GetDeviceInfoCommandPacket get_device_info_command;
    serial_connection_.write(get_device_info_command.buffer);

    if (verbose_) {
        std::cout << "Starting polling thread." << '\n';
    }

    run_polling_thread_ = true;
    worker_thread_ = std::thread(&MiniAHRSDriver::workerThreadMain, this); 
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    if (!have_device_info_) {
        if (verbose_) {
            std::cout << "No device info after GetDeviceInfo packet sent." << '\n';
        }
        run_polling_thread_ = false;
        worker_thread_.join();

        return false;
    }

    ReadMiniAHRSParamsCommandPacket read_params_packet;
    serial_connection_.write(read_params_packet.buffer); 

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    if (!have_device_params_) {
        run_polling_thread_ = false;
        worker_thread_.join();

        return false;
    }

    // StartOrientationDataStreamCommandPacket start_command;
    StartUserDefineDataStreamCommandPacket start_command;
    serial_connection_.write(start_command.buffer);

    std::this_thread::sleep_for(
        std::chrono::seconds(initial_alignment_time_seconds_)
    );

    return true;
}



HeaderData MiniAHRSDriver::parseHeader(const std::vector<uint8_t>& data) {
    // std::cout<< "msg header: ";
    // for (uint8_t b : data) {
    //     std::cout << static_cast<int>(b) << " ";
    // }
    // std::cout<<"; "<< std::hex;
    // for (uint8_t b : data) {
    //     std::cout << static_cast<int>(b) << " ";
    // }
    // std::cout<< std::dec <<std::endl;
    if (data[0] != 0xAA || data[1] != 0x55) {
        throw std::runtime_error("Malformed header!");
    }

    HeaderData result;
    result.message_type = data[2];
    result.data_identifier = data[3];

    uint8_t message_size_raw[2];
    message_size_raw[0] = data[4];
    message_size_raw[1] = data[5];

    uint16_t message_length;
    std::memcpy(&message_length, message_size_raw, sizeof message_length);

    result.message_length = message_length;

    return result;
}

AHRSUserDefinednData MiniAHRSDriver::parseUserDefinedDataBody (const std::vector<uint8_t>& message_body, const std::vector<uint8_t>& data_list) {
    AHRSUserDefinednData result;
    int cnt = 0;
    // std::cout<< "data body: ";
    // for (uint8_t b : message_body) {
    //    std::cout << static_cast<int>(b) << " ";
    // }
    // std::cout<< std::dec<<std::endl;
    for (auto data_type : data_list) {
        // std::cout << "data_type: " << std::hex<<static_cast<int>(data_type) <<std::dec <<std::endl;
        switch (data_type) {
        case 0x01: {
	    uint8_t low_res_time_raw[4];
            low_res_time_raw[0] = message_body[cnt];
            low_res_time_raw[1] = message_body[cnt+1];
            low_res_time_raw[2] = message_body[cnt+2];
            low_res_time_raw[3] = message_body[cnt+3];

            uint32_t low_res_time;
            std::memcpy(&low_res_time, &low_res_time_raw, sizeof low_res_time);
            result.low_res_time = double(low_res_time) * 0.001;
            cnt += 4;
            // std::cout << "result.low_res_time: "<< result.low_res_time <<std::endl;
            break;
        }
        case 0x03: {
            uint8_t hi_res_time_raw[8];
            hi_res_time_raw[0] = message_body[cnt];
            hi_res_time_raw[1] = message_body[cnt+1];
            hi_res_time_raw[2] = message_body[cnt+2];
            hi_res_time_raw[3] = message_body[cnt+3];
            hi_res_time_raw[4] = message_body[cnt+4];
            hi_res_time_raw[5] = message_body[cnt+5];
            hi_res_time_raw[6] = message_body[cnt+6];
            hi_res_time_raw[7] = message_body[cnt+7];

            uint64_t hi_res_time;
            std::memcpy(&hi_res_time, &hi_res_time_raw, sizeof hi_res_time);
            result.hi_res_time = double(hi_res_time) * 1.0e-9;
            // std::cout<< "result.hi_res_time: "<< result.hi_res_time <<std::endl;
            cnt += 8;
            break;
        }
        case 0x07: {
	    uint8_t yaw_raw[2];
	    uint8_t pitch_raw[2];
	    uint8_t roll_raw[2];
            yaw_raw[0] = message_body[cnt];
            yaw_raw[1] = message_body[cnt+1];

            pitch_raw[0] = message_body[cnt+2];
            pitch_raw[1] = message_body[cnt+3];

            roll_raw[0] = message_body[cnt+4];
            roll_raw[1] = message_body[cnt+5];

            uint16_t yaw;
            int16_t pitch;
            int16_t roll;

            std::memcpy(&yaw, &yaw_raw, sizeof yaw);
            std::memcpy(&pitch, &pitch_raw, sizeof pitch);
            std::memcpy(&roll, &roll_raw, sizeof roll);

	    result.yaw_degrees = double(yaw) / 100.0;
	    result.pitch_degrees = double(pitch) / 100.0;
	    result.roll_degrees = double(roll) / 100.0;
            // std::cout<< "result.yaw_degrees: "<< result.yaw_degrees <<std::endl;
            // std::cout<< "result.pitch_degrees: "<< result.pitch_degrees <<std::endl;
            // std::cout<< "result.roll_degrees: "<< result.roll_degrees <<std::endl;
            cnt += 6;
            break;
        }
        case 0x08: {
            uint8_t yaw_raw[4];
	    uint8_t pitch_raw[4];
	    uint8_t roll_raw[4];
            yaw_raw[0] = message_body[cnt];
            yaw_raw[1] = message_body[cnt+1];
            yaw_raw[2] = message_body[cnt+2];
            yaw_raw[3] = message_body[cnt+3];

            pitch_raw[0] = message_body[cnt+4];
            pitch_raw[1] = message_body[cnt+5];
            pitch_raw[2] = message_body[cnt+6];
            pitch_raw[3] = message_body[cnt+7];

            roll_raw[0] = message_body[cnt+8];
            roll_raw[1] = message_body[cnt+9];
            roll_raw[2] = message_body[cnt+10];
            roll_raw[3] = message_body[cnt+11];

            uint32_t yaw;
            int32_t pitch;
            int32_t roll;

            std::memcpy(&yaw, &yaw_raw, sizeof yaw);
            std::memcpy(&pitch, &pitch_raw, sizeof pitch);
            std::memcpy(&roll, &roll_raw, sizeof roll);

	    result.yaw_hr_degrees = double(yaw) / 1000.0;
	    result.pitch_hr_degrees = double(pitch) / 1000.0;
	    result.roll_hr_degrees = double(roll) / 1000.0;
            // std::cout<< "result.yaw_hr_degrees: "<< result.yaw_hr_degrees <<std::endl;
            // std::cout<< "result.pitch_hr_degrees: "<< result.pitch_hr_degrees <<std::endl;
            // std::cout<< "result.roll_hr_degrees: "<< result.roll_hr_degrees <<std::endl;
            cnt += 12;
            break;
        }
 	case 0x09: {
	    uint8_t qx_raw[2];
	    uint8_t qy_raw[2];
	    uint8_t qz_raw[2];
            uint8_t qw_raw[2];

            qx_raw[0] = message_body[cnt];
            qx_raw[1] = message_body[cnt+1];

            qy_raw[0] = message_body[cnt+2];
            qy_raw[1] = message_body[cnt+3];

            qz_raw[0] = message_body[cnt+4];
            qz_raw[1] = message_body[cnt+5];

            qw_raw[0] = message_body[cnt+6];
            qw_raw[1] = message_body[cnt+7];

            int16_t qx, qy, qz, qw;

            std::memcpy(&qx, &qx_raw, sizeof qx);
            std::memcpy(&qy, &qy_raw, sizeof qy);
            std::memcpy(&qz, &qz_raw, sizeof qz);
            std::memcpy(&qw, &qw_raw, sizeof qw);

	    result.qx = double(qx) / 10000.0;
	    result.qy = double(qy) / 10000.0;
	    result.qz = double(qz) / 10000.0;
            result.qz = double(qw) / 10000.0;
            // std::cout<< "result.qx: "<< result.qx <<std::endl;
            // std::cout<< "result.qy: "<< result.qy <<std::endl;
            // std::cout<< "result.qz: "<< result.qz <<std::endl;
            // std::cout<< "result.qw: "<< result.qw <<std::endl;
            cnt += 8;
            break;
        }
        case 0x20: {
	    uint8_t gyro_x_raw[2];
	    uint8_t gyro_y_raw[2];
	    uint8_t gyro_z_raw[2];

	    gyro_x_raw[0] = message_body[cnt];
	    gyro_x_raw[1] = message_body[cnt+1];

	    gyro_y_raw[0] = message_body[cnt+2];
	    gyro_y_raw[1] = message_body[cnt+3];

	    gyro_z_raw[0] = message_body[cnt+4];
	    gyro_z_raw[1] = message_body[cnt+5];

    	    int16_t gyro_x, gyro_y, gyro_z;

	    std::memcpy(&gyro_x, &gyro_x_raw, sizeof gyro_x);
	    std::memcpy(&gyro_y, &gyro_y_raw, sizeof gyro_y);
	    std::memcpy(&gyro_z, &gyro_z_raw, sizeof gyro_z);

	    result.gyro_x = double(gyro_x) / KG_;
	    result.gyro_y = double(gyro_y) / KG_;
	    result.gyro_z = double(gyro_z) / KG_;
            // std::cout<< "result.gyro_x: "<< result.gyro_x <<std::endl;
            // std::cout<< "result.gyro_y: "<< result.gyro_y <<std::endl;
            // std::cout<< "result.gyro_z: "<< result.gyro_z <<std::endl;
            cnt += 6;            
            break;
        }
        case 0x21: {
            uint8_t gyro_x_raw[4];
	    uint8_t gyro_y_raw[4];
	    uint8_t gyro_z_raw[4];

	    gyro_x_raw[0] = message_body[cnt];
	    gyro_x_raw[1] = message_body[cnt+1];
	    gyro_x_raw[2] = message_body[cnt+2];
	    gyro_x_raw[3] = message_body[cnt+3];

	    gyro_y_raw[0] = message_body[cnt+4];
	    gyro_y_raw[1] = message_body[cnt+5];
	    gyro_y_raw[2] = message_body[cnt+6];
	    gyro_y_raw[3] = message_body[cnt+7];

	    gyro_z_raw[0] = message_body[cnt+8];
	    gyro_z_raw[1] = message_body[cnt+9];
	    gyro_z_raw[2] = message_body[cnt+10];
	    gyro_z_raw[3] = message_body[cnt+11];

    	    int32_t gyro_x, gyro_y, gyro_z;

	    std::memcpy(&gyro_x, &gyro_x_raw, sizeof gyro_x);
	    std::memcpy(&gyro_y, &gyro_y_raw, sizeof gyro_y);
	    std::memcpy(&gyro_z, &gyro_z_raw, sizeof gyro_z);
	    
            double param = 180.0 / 1.0e5 / M_PI;
	    result.gyro_hr_x = double(gyro_x) * param;
	    result.gyro_hr_y = double(gyro_y) * param;
	    result.gyro_hr_z = double(gyro_z) * param;
            // std::cout<< "result.gyro_hr_x: "<< result.gyro_hr_x <<std::endl;
            // std::cout<< "result.gyro_hr_y: "<< result.gyro_hr_y <<std::endl;
            // std::cout<< "result.gyro_hr_z: "<< result.gyro_hr_z <<std::endl;
            cnt += 12;            
            break;
        }
	case 0x22: {
            uint8_t accel_x_raw[2];
            uint8_t accel_y_raw[2];
            uint8_t accel_z_raw[2];

            accel_x_raw[0] = message_body[cnt];
            accel_x_raw[1] = message_body[cnt+1];

            accel_y_raw[0] = message_body[cnt+2];
            accel_y_raw[1] = message_body[cnt+3];

            accel_z_raw[0] = message_body[cnt+4];
            accel_z_raw[1] = message_body[cnt+5];

            int16_t acc_x, acc_y, acc_z;

            std::memcpy(&acc_x, &accel_x_raw, sizeof acc_x);
            std::memcpy(&acc_y, &accel_y_raw, sizeof acc_y);
            std::memcpy(&acc_z, &accel_z_raw, sizeof acc_z);


            result.acc_x = double(acc_x) / KA_;
            result.acc_y = double(acc_y) / KA_;
            result.acc_z = double(acc_z) / KA_;
            // std::cout<< "result.acc_x: "<< result.acc_x <<std::endl;
            // std::cout<< "result.acc_y: "<< result.acc_y <<std::endl;
            // std::cout<< "result.acc_z: "<< result.acc_z <<std::endl;
            cnt += 6;
            break;
        }
        case 0x23: {
            uint8_t accel_x_raw[4];
            uint8_t accel_y_raw[4];
            uint8_t accel_z_raw[4];

            accel_x_raw[0] = message_body[cnt];
            accel_x_raw[1] = message_body[cnt+1];
            accel_x_raw[2] = message_body[cnt+2];
            accel_x_raw[3] = message_body[cnt+3];

            accel_y_raw[0] = message_body[cnt+4];
            accel_y_raw[1] = message_body[cnt+5];
            accel_y_raw[2] = message_body[cnt+6];
            accel_y_raw[3] = message_body[cnt+7];

            accel_z_raw[0] = message_body[cnt+8];
            accel_z_raw[1] = message_body[cnt+9];
            accel_z_raw[2] = message_body[cnt+10];
            accel_z_raw[3] = message_body[cnt+11];

            int32_t acc_x, acc_y, acc_z;

            std::memcpy(&acc_x, &accel_x_raw, sizeof acc_x);
            std::memcpy(&acc_y, &accel_y_raw, sizeof acc_y);
            std::memcpy(&acc_z, &accel_z_raw, sizeof acc_z);


            result.acc_hr_x = double(acc_x) / 1.0e6;
            result.acc_hr_y = double(acc_y) / 1.0e6;
            result.acc_hr_z = double(acc_z) / 1.0e6;
            // std::cout<< "result.acc_hr_x: "<< result.acc_hr_x <<std::endl;
            // std::cout<< "result.acc_hr_y: "<< result.acc_hr_y <<std::endl;
            // std::cout<< "result.acc_hr_z: "<< result.acc_hr_z <<std::endl;
            cnt += 12;            
            break;
        }
        case 0x24: {

            uint8_t mag_x_raw[2];
            uint8_t mag_y_raw[2];
            uint8_t mag_z_raw[2];

            mag_x_raw[0] = message_body[cnt];
            mag_x_raw[1] = message_body[cnt+1];

            mag_y_raw[0] = message_body[cnt+2];
            mag_y_raw[1] = message_body[cnt+3];

            mag_z_raw[0] = message_body[cnt+4];
            mag_z_raw[1] = message_body[cnt+5];

            int16_t mag_x, mag_y, mag_z;

            std::memcpy(&mag_x, &mag_x_raw, sizeof mag_x);
            std::memcpy(&mag_y, &mag_y_raw, sizeof mag_y);
            std::memcpy(&mag_z, &mag_z_raw, sizeof mag_z);

            double nano_tesla_to_tesla = 1.0 / 1.0e-9; 
            result.mag_x = double(mag_x) * 10.0;
            result.mag_y = double(mag_y) * 10.0;
            result.mag_z = double(mag_z) * 10.0;
            // std::cout<< "result.mag_x: "<< result.mag_x <<std::endl;
            // std::cout<< "result.mag_y: "<< result.mag_y <<std::endl;
            // std::cout<< "result.mag_z: "<< result.mag_z <<std::endl;
            cnt += 6;            
            break;
        }
        case 0x50: {
            uint8_t temp_volt[2];

	    temp_volt[0] = message_body[cnt];
	    temp_volt[1] = message_body[cnt+1];

            int16_t voltage;

	    std::memcpy(&voltage, &temp_volt, sizeof voltage);

            result.voltage = double(voltage) / 100.0;
            // std::cout<< "result.voltage: "<< result.voltage <<std::endl;
            cnt += 2;
            break;
        }
        case 0x52: {
            uint8_t temp_raw[2];

	    temp_raw[0] = message_body[cnt];
	    temp_raw[1] = message_body[cnt+1];

            int16_t temperature;

	    std::memcpy(&temperature, &temp_raw, sizeof temperature);

            result.temperature = double(temperature) / 10.0;
            // std::cout<< "result.temperature: "<< result.temperature <<std::endl;
            cnt += 2;            
            break;
        }
        case 0x53: {
            uint8_t usw_raw[2];

	    usw_raw[0] = message_body[cnt];
	    usw_raw[1] = message_body[cnt+1];

            int16_t usw;

	    std::memcpy(&usw, &usw_raw, sizeof usw);


    	   // Fill the array with bits from the status
    	   for (int i = 0; i < 16; ++i) {
    	       result.usw[i] = (usw >> i) & 1;
    	   }
           // std::cout<<"usw: ";
           for (int i = 0; i  < 13; i++) {
               if (result.usw[i] != 0){
                   std::cout<< "Non zero in usw, contact Inrtial Labs Support!" << '\n';
               }
	   //    std::cout<< result.usw[i]<< " ";
           }
           // std::cout<<std::endl;
           cnt += 2;            
           break;
        }
        default: {
            std::cout << "Unrecognized user defined data format!" << '\n';
            break;
        }
        } 
    }
    
    return result;


}

AHRSOrientationData MiniAHRSDriver::parseUserDefinedData(const std::vector<uint8_t>& message_body) {
    bool DEBUG = 0;
    AHRSOrientationData result;
    if (DEBUG) {
	std::cout<< "msg body: ";
	for (uint8_t b : message_body) {
	    std::cout << static_cast<int>(b) << " ";
	}
	std::cout<<"; "<< std::hex;
	for (uint8_t b : message_body) {
	    std::cout << static_cast<int>(b) << " ";
	}
	std::cout<< std::dec <<std::endl;
    }
    uint8_t pkg_num = message_body[0];
    std::vector<uint8_t> data_list(pkg_num);
    for (uint8_t i = 0; i < pkg_num; i++) {
        data_list[i] = message_body[i+1];
    }
    if (DEBUG) {
	std::cout<< "data list: ";
	for (uint8_t b : data_list) {
	    std::cout << static_cast<int>(b) << " ";
	}
	std::cout<< std::dec <<std::endl;
    }
    
    if (pkg_num > message_body.size()){
        result.time = -1.0;
        return result;
    }


    std::vector<uint8_t> data_body(message_body.begin() + pkg_num + 1, message_body.end());
    
    AHRSUserDefinednData custom_result = parseUserDefinedDataBody(data_body, data_list);
    result.acc_x = custom_result.acc_hr_x;
    result.acc_y = custom_result.acc_hr_y;
    result.acc_z = custom_result.acc_hr_z;

    result.gyro_x = custom_result.gyro_hr_x;
    result.gyro_y = custom_result.gyro_hr_y;
    result.gyro_z = custom_result.gyro_hr_z;

    result.mag_x = custom_result.mag_x;
    result.mag_y = custom_result.mag_y;
    result.mag_z = custom_result.mag_z;

    result.yaw_degrees = custom_result.yaw_hr_degrees;
    result.pitch_degrees = custom_result.pitch_hr_degrees;
    result.roll_degrees = custom_result.roll_hr_degrees;

    result.temperature = custom_result.temperature;
    
    result.time = custom_result.hi_res_time;

    return result;
}

AHRSOrientationData MiniAHRSDriver::parseOrientationData(const std::vector<uint8_t>& message_body) {
    AHRSOrientationData result;

    uint8_t yaw_raw[2];
    uint8_t pitch_raw[2];
    uint8_t roll_raw[2];

    uint8_t gyro_x_raw[2];
    uint8_t gyro_y_raw[2];
    uint8_t gyro_z_raw[2];

    uint8_t accel_x_raw[2];
    uint8_t accel_y_raw[2];
    uint8_t accel_z_raw[2];

    uint8_t mag_x_raw[2];
    uint8_t mag_y_raw[2];
    uint8_t mag_z_raw[2];

    uint8_t temp_raw[2];

    uint16_t yaw;
    int16_t pitch;
    int16_t roll;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;

    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;

    int16_t temperature;

    yaw_raw[0] = message_body[0];
    yaw_raw[1] = message_body[1];

    pitch_raw[0] = message_body[2];
    pitch_raw[1] = message_body[3];

    roll_raw[0] = message_body[4];
    roll_raw[1] = message_body[5];

    gyro_x_raw[0] = message_body[6];
    gyro_x_raw[1] = message_body[7];

    gyro_y_raw[0] = message_body[8];
    gyro_y_raw[1] = message_body[9];

    gyro_z_raw[0] = message_body[10];
    gyro_z_raw[1] = message_body[11];

    accel_x_raw[0] = message_body[12];
    accel_x_raw[1] = message_body[13];

    accel_y_raw[0] = message_body[14];
    accel_y_raw[1] = message_body[15];

    accel_z_raw[0] = message_body[16];
    accel_z_raw[1] = message_body[17];

    mag_x_raw[0] = message_body[18];
    mag_x_raw[1] = message_body[19];

    mag_y_raw[0] = message_body[20];
    mag_y_raw[1] = message_body[21];

    mag_z_raw[0] = message_body[22];
    mag_z_raw[1] = message_body[23];

    temp_raw[0] = message_body[32];
    temp_raw[1] = message_body[33];

    std::memcpy(&yaw, &yaw_raw, sizeof yaw);
    std::memcpy(&pitch, &pitch_raw, sizeof pitch);
    std::memcpy(&roll, &roll_raw, sizeof roll);

    std::memcpy(&gyro_x, &gyro_x_raw, sizeof gyro_x);
    std::memcpy(&gyro_y, &gyro_y_raw, sizeof gyro_y);
    std::memcpy(&gyro_z, &gyro_z_raw, sizeof gyro_z);

    std::memcpy(&mag_x, &mag_x_raw, sizeof mag_x);
    std::memcpy(&mag_y, &mag_y_raw, sizeof mag_y);
    std::memcpy(&mag_z, &mag_z_raw, sizeof mag_z);

    std::memcpy(&acc_x, &accel_x_raw, sizeof acc_x);
    std::memcpy(&acc_y, &accel_y_raw, sizeof acc_y);
    std::memcpy(&acc_z, &accel_z_raw, sizeof acc_z);

    std::memcpy(&temperature, &temp_raw, sizeof temperature);

    result.acc_x = double(acc_x) / KA_;
    result.acc_y = double(acc_y) / KA_;
    result.acc_z = double(acc_z) / KA_;

    result.gyro_x = double(gyro_x) / KG_;
    result.gyro_y = double(gyro_y) / KG_;
    result.gyro_z = double(gyro_z) / KG_;

    double nano_tesla_to_tesla = 1.0 / 1.0e-9; 
    result.mag_x = double(mag_x) * 10.0;
    result.mag_y = double(mag_y) * 10.0;
    result.mag_z = double(mag_z) * 10.0;

    result.yaw_degrees = double(yaw) / 100.0;
    result.pitch_degrees = double(pitch) / 100.0;
    result.roll_degrees = double(roll) / 100.0;

    result.temperature = double(temperature) / 10.0;

    return result;
}

void MiniAHRSDriver::handleGetDeviceInfoResponse(const std::vector<uint8_t>& message_body) {
    if (have_device_info_) {
        std::cerr << "Already have device info. Skipping." << '\n';
        return;
    }

    std::string serial_number("");
    for (int i = 0; i < 8; ++i) {
        serial_number.push_back(message_body[i]);
    }

    std::string firmware_version("");
    for (int i = 8; i < 48; ++i) {
        firmware_version.push_back(message_body[i]);
    }

    have_device_info_ = true;
    device_serial_number_ = serial_number;
    device_firmware_version_ = firmware_version;
}

void MiniAHRSDriver::workerThreadMain() {
    try {
        while (run_polling_thread_) {
            AHRSDataPollingLoop();
        }
    } catch (const std::runtime_error e) {
        StopDeviceCommandPacket packet;
        serial_connection_.write(packet.buffer);
        return;
    }
}

void MiniAHRSDriver::handleReadParamsCommandResponse(const std::vector<uint8_t>& message_body)
{
    uint8_t data_rate_raw[2];
    data_rate_raw[0] = message_body[0];
    data_rate_raw[1] = message_body[1];

    uint8_t initial_alignment_time_raw[2];
    initial_alignment_time_raw[0] = message_body[2];
    initial_alignment_time_raw[1] = message_body[3];

    uint16_t data_rate;
    uint16_t alignment_time;

    std::memcpy(
        &data_rate,
        data_rate_raw,
        sizeof data_rate
    );

    std::memcpy(
        &alignment_time,
        initial_alignment_time_raw,
        sizeof alignment_time
    );
    initial_alignment_time_seconds_ = alignment_time;
    data_rate_hz_ = data_rate;
    have_device_params_ = true;
}

void MiniAHRSDriver::AHRSDataPollingLoop() {
    
    // std::cout<< "AHRSDataPollingLoop: "  << serial_connection_.available() << " ";
    std::this_thread::sleep_for(std::chrono::nanoseconds(1000));
    
    // read the first five bytes
    if (serial_connection_.available() < HEADER_LENGTH_BYTES) {
        return;
    }

    // look for the first two bytes of a message
    std::vector<uint8_t> message_header;
    serial_connection_.read(message_header, HEADER_LENGTH_BYTES);
    std::this_thread::sleep_for(std::chrono::nanoseconds(1000));

    HeaderData parsed_header;
    try {
        parsed_header = parseHeader(message_header);
    } catch (std::runtime_error e) {
        if (verbose_) {
            std::cout << "Invalid header!" << '\n';
            for (auto& byte : message_header) {
                std::cout << std::hex << int(byte) << " ";
            }
            std::cout << '\n';
        }
        throw std::runtime_error("Invalid header received. Cannot continue parsing!");
    }

    std::vector<uint8_t> message_body;
    serial_connection_.read(message_body, parsed_header.message_length - 4);
    std::this_thread::sleep_for(std::chrono::nanoseconds(1000));
    // std::cout<< "parsed_header.data_identifier" <<parsed_header.data_identifier <<" ";
    

    switch (parsed_header.data_identifier) {
        case 0x12: {
            handleGetDeviceInfoResponse(message_body);
            break;
        }
        case 0x33: {
            AHRSOrientationData data = parseOrientationData(message_body);

            if (data_callback_) {
                data_callback_(data);
            }

            break;
        }
        case 0x41: {
            handleReadParamsCommandResponse(message_body);
            break;
        }
	// parse user defined data
        case 0x95: {
            AHRSOrientationData data = parseUserDefinedData(message_body);

            if (data_callback_) {
                data_callback_(data);
            }

            break;
        }
        default: {
            std::cout << "Unrecognized command format!" << '\n';
            break;
        }
    }
}

} // mini_ahrs_driver
