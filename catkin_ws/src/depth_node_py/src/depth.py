#!/usr/bin/env python3

"""
  Author: Alberto Quattrini Li
  Affiliation: AFRL - University of South Carolina
  Date: 06/20/2016
  Modified for gpiod and smbus2

  Description:
  Publishes sensor reading from the depth sensor.

  Usage:
  sudo python3 depth.py

  or

  rosrun depth_node_py run_node.sh
"""

import signal
import sys
import gpiod
import smbus2

import rospy
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Temperature
from depth_node_py.msg import Depth

# MACROS
SENSOR_NAME = 'bar30'
TOPIC_SEPARATOR = '/'
PRESSURE_TOPIC_NAME = 'pressure'
TEMPERATURE_TOPIC_NAME = 'temperature'
DEPTH_TOPIC_NAME = 'depth'

# Parameter for hardware.
GPIO_CHIP = '/dev/gpiochip0'  # Adjust if needed
I2C_GPIO_PIN = 8  # GPIO Pin for power control (if needed)
I2C_BUS = 1  # I2C bus number (usually 1)
MS5837_ADDR = 0x76  # BAR30 I2C address

# Parameters for pressure sensor.
DENSITY_DEFAULT = "sea"

# Parameters for node.
POLL_FREQUENCY = 10

# Fluid densities
DENSITY_FRESHWATER = 997
DENSITY_SALTWATER = 1029

# MS5837 Commands
MS5837_RESET = 0x1E
MS5837_ADC_READ = 0x00
MS5837_PROM_READ = 0xA0
MS5837_CONVERT_D1_8192 = 0x4A
MS5837_CONVERT_D2_8192 = 0x5A


class MS5837:
    """MS5837 pressure sensor driver using smbus2."""
    
    def __init__(self, bus=I2C_BUS, address=MS5837_ADDR, density=DENSITY_DEFAULT):
        self._bus = smbus2.SMBus(bus)
        self._address = address
        
        if density == "sea":
            self.fluid_density = DENSITY_SALTWATER
        elif density == "fresh":
            self.fluid_density = DENSITY_FRESHWATER
        else:
            self.fluid_density = DENSITY_SALTWATER
        
        self._C = []
        self.D1 = 0
        self.D2 = 0
        self.TEMP = 0
        self.P = 0
        
        self._reset()
        self._load_calibration()
    
    def _reset(self):
        self._bus.write_byte(self._address, MS5837_RESET)
        import time
        time.sleep(0.01)
    
    def _load_calibration(self):
        self._C = [0] * 7
        for i in range(7):
            data = self._bus.read_i2c_block_data(self._address, MS5837_PROM_READ + i * 2, 2)
            self._C[i] = (data[0] << 8) | data[1]
    
    def read(self):
        import time
        
        # Request D1 conversion
        self._bus.write_byte(self._address, MS5837_CONVERT_D1_8192)
        time.sleep(0.02)
        
        data = self._bus.read_i2c_block_data(self._address, MS5837_ADC_READ, 3)
        self.D1 = (data[0] << 16) | (data[1] << 8) | data[2]
        
        # Request D2 conversion
        self._bus.write_byte(self._address, MS5837_CONVERT_D2_8192)
        time.sleep(0.02)
        
        data = self._bus.read_i2c_block_data(self._address, MS5837_ADC_READ, 3)
        self.D2 = (data[0] << 16) | (data[1] << 8) | data[2]
        
        self._calculate()
        return True
    
    def _calculate(self):
        dT = self.D2 - self._C[5] * 256
        SENS = self._C[1] * 32768 + (self._C[3] * dT) // 256
        OFF = self._C[2] * 65536 + (self._C[4] * dT) // 128
        
        self.TEMP = 2000 + dT * self._C[6] // 8388608
        self.P = (self.D1 * SENS // 2097152 - OFF) // 8192
        
        # Second order compensation
        if (self.TEMP // 100) < 20:
            Ti = (3 * dT * dT) // 8589934592
            OFFi = (3 * (self.TEMP - 2000) * (self.TEMP - 2000)) // 2
            SENSi = (5 * (self.TEMP - 2000) * (self.TEMP - 2000)) // 8
            if (self.TEMP // 100) < -15:
                OFFi = OFFi + 7 * (self.TEMP + 1500) * (self.TEMP + 1500)
                SENSi = SENSi + 4 * (self.TEMP + 1500) * (self.TEMP + 1500)
        else:
            Ti = 2 * (dT * dT) // 137438953472
            OFFi = (1 * (self.TEMP - 2000) * (self.TEMP - 2000)) // 16
            SENSi = 0
        
        OFF2 = OFF - OFFi
        SENS2 = SENS - SENSi
        
        self.TEMP = self.TEMP - Ti
        self.P = ((self.D1 * SENS2) // 2097152 - OFF2) // 8192
    
    def pressure(self, conversion=1.0):
        return self.P / 10.0 * conversion
    
    def temperature(self):
        return self.TEMP / 100.0
    
    def depth(self):
        return (self.pressure(100.0) - 101300) / (self.fluid_density * 9.80665)
    
    def altitude(self):
        return (1 - pow((self.pressure() / 1013.25), 0.190284)) * 145366.45 * 0.3048
    
    def close(self):
        self._bus.close()


class GPIOControl:
    """GPIO control using gpiod."""
    
    def __init__(self, chip=GPIO_CHIP, pin=I2C_GPIO_PIN):
        self.chip = gpiod.Chip(chip)
        self.line = self.chip.get_line(pin)
        self.line.request(consumer="depth_sensor", type=gpiod.LINE_REQ_DIR_OUT)
    
    def set_high(self):
        self.line.set_value(1)
    
    def set_low(self):
        self.line.set_value(0)
    
    def close(self):
        self.line.release()


def signal_handler(received_signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


def sensor_initialization(density=DENSITY_DEFAULT, use_gpio=False):
    """Initialize the sensor.

    Args:
        density (string): density in which the pressure sensor is used
        use_gpio (bool): whether to use GPIO for power control

    Returns:
        Tuple of (gpio_control or None, sensor)
    """
    gpio_ctrl = None
    
    if use_gpio:
        try:
            gpio_ctrl = GPIOControl()
            gpio_ctrl.set_high()
            import time
            time.sleep(0.1)  # Wait for sensor to power up
        except Exception as e:
            rospy.logwarn("GPIO control not available: %s" % str(e))
            gpio_ctrl = None
    
    sensor = MS5837(density=density)
    
    return gpio_ctrl, sensor


def sensor_termination(gpio_ctrl, sensor):
    """Terminate the sensor."""
    if sensor:
        sensor.close()
    
    if gpio_ctrl:
        gpio_ctrl.set_low()
        gpio_ctrl.close()


def publish_data_from_depth_sensor():
    """Code that publishes the data from the depth sensor."""
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Instantiating publishers.
    pressure_pub = rospy.Publisher(
        TOPIC_SEPARATOR.join([SENSOR_NAME, PRESSURE_TOPIC_NAME]),
        FluidPressure, queue_size=10)
    temperature_pub = rospy.Publisher(
        TOPIC_SEPARATOR.join([SENSOR_NAME, TEMPERATURE_TOPIC_NAME]),
        Temperature, queue_size=10)
    depth_pub = rospy.Publisher(
        TOPIC_SEPARATOR.join([SENSOR_NAME, DEPTH_TOPIC_NAME]),
        Depth, queue_size=10)

    # Start the node.
    rospy.init_node('bar30_talker', anonymous=True)

    # Get parameters.
    frequency = rospy.get_param('~poll_frequency', POLL_FREQUENCY)
    density = rospy.get_param('~density', DENSITY_DEFAULT)
    use_gpio = rospy.get_param('~use_gpio', False)

    rospy.loginfo("Density: %s" % density)
    rospy.loginfo("Poll frequency: %d Hz" % frequency)

    # Initialize sensor.
    gpio_ctrl, sensor = sensor_initialization(density=density, use_gpio=use_gpio)

    if not sensor:
        rospy.logerr("Sensor not created.")
        sys.exit(-1)

    rate = rospy.Rate(frequency)

    rospy.loginfo("BAR30 sensor initialized. Publishing data...")

    while not rospy.is_shutdown():
        pressure_msg = FluidPressure()
        temperature_msg = Temperature()
        depth_msg = Depth()
        
        try:
            sensor.read()

            time_now = rospy.get_rostime()
            
            pressure_msg.header.stamp = time_now
            pressure_msg.header.frame_id = 'bar30'
            pressure_msg.fluid_pressure = sensor.pressure(100.0)  # Pascals
            pressure_msg.variance = 0
            
            temperature_msg.header.stamp = time_now
            temperature_msg.header.frame_id = 'bar30'
            temperature_msg.temperature = sensor.temperature()  # Celsius
            temperature_msg.variance = 0
            
            depth_msg.header.stamp = time_now
            depth_msg.header.frame_id = 'bar30'
            depth_msg.depth = sensor.depth()
            depth_msg.depth_variance = 0
            depth_msg.altitude = sensor.altitude()
            depth_msg.altitude_variance = 0

            rospy.logdebug('Pressure = {0:0.2f} Pa'.format(sensor.pressure(100.0)))
            rospy.logdebug('Temp = {0:0.2f} *C'.format(sensor.temperature()))
            rospy.logdebug('Depth = {0:0.2f} m'.format(sensor.depth()))
            rospy.logdebug('Altitude = {0:0.2f} m'.format(sensor.altitude()))

            pressure_pub.publish(pressure_msg)
            temperature_pub.publish(temperature_msg)
            depth_pub.publish(depth_msg)

        except Exception as e:
            rospy.logerr("Exception when reading depth data: %s" % str(e))

        rate.sleep()

    sensor_termination(gpio_ctrl, sensor)


if __name__ == '__main__':
    publish_data_from_depth_sensor()