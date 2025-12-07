# Copyright (c) 2016
# Author: Alberto Quattrini Li
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHA THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""
Author: Alberto Quattrini Li
Affiliation: AFRL - University of South Carolina
Date: 06/20/2016

Library for interfacing to the BlueRobotics BAR30 depth sensor.
Code derived from porting the C code https://github.com/bluerobotics/BlueRobotics_MS5837_Library/blob/master/MS5837.cpp.

For more information, the technical sheet at:
http://www.mouser.com/ds/2/418/MS5837-30BA-736494.pdf

@todo More documentation.
@todo Clean the code.
"""
from __future__ import division
import logging
import sys
import time

# DEBUG.
LOG_LEVEL = logging.INFO

# MS5837 default address.
MS5837_I2CADDR           = 0x76
MS5837_WRITE_ADDR        = 0xEC
MS5837_READ_ADDR         = 0xED

# Commands.
MS5837_RESET             = 0x1E
MS5837_ADC_READ          = 0x00 # Read Analog Digital Converter result.
MS5837_PROM_READ         = 0xA0 # Read calibration data (112 bit from technical sheet).
MS5837_CONVERT_D1_8192   = 0x4A
MS5837_CONVERT_D2_8192   = 0x5A

# Constants for MS5837.
MS5837_PROM_ADDRESSES_NUM = 7 # Number of bytes to read from PROM_READ.

# Constants.
little_endian = False
Pa = 100.0
bar = 0.001
mbar = 1.0
sea_density = 1029 # Sea. # @TODO add parameter. TOMODIFY.
fresh_density = 997 # Fresh water.
air_density = 1.225 # Air.

fluid_density = fresh_density

class NullHandler(logging.Handler):
    def emit(self, record):
        pass

h = NullHandler()
logging.getLogger("depth").addHandler(h)

# TODO External library.
def send_command(device, address, command):
    device._idle()
    device._transaction_start()
    device._i2c_start()
    device._i2c_write_bytes([address, command])
    device._i2c_stop()
    response = device._transaction_end()

    return response


class MS5837(object):
    def __init__(self, address=MS5837_I2CADDR, i2c=None, density="sea", **kwargs):
        """
        Constructor.
        """
        # Set the logger
        self._logger = logging.getLogger('depth.MS5837')
        self._logger.setLevel(LOG_LEVEL)

        if LOG_LEVEL == logging.DEBUG:
            ch = logging.StreamHandler(sys.stdout)
            ch.setLevel(LOG_LEVEL)
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s: %(message)s')
            ch.setFormatter(formatter)
            self._logger.addHandler(ch)

        self._logger.debug("Init.") 
        
        # Set the fluid density used in the calculations.
        if density == "sea":      
            self.setFluidDensity(sea_density)
        elif density == "fresh":      
            self.setFluidDensity(fresh_density)
        if density == "air":      
            self.setFluidDensity(air_density)
        else:
            self._logger.error("Density not set. Default is sea.") 
            self.setFluidDensity(sea_density)

        # Create I2C device.
        if i2c is None:
            import Adafruit_GPIO.I2C as I2C
            i2c = I2C
        self._device = i2c.get_i2c_device(address, **kwargs)
        
        # Reset the MS5837, per datasheet.
        self._reset()
        
        # Load calibration values.
        self._C = []
        self._load_calibration()

    def _reset(self):
        self._logger.debug("Reset.") 
        #self._device.writeRaw8(MS5837_RESET)
        self.writeRaw8(MS5837_RESET)
        time.sleep(0.5)

    def _second_reset(self):
        # Send of several SCL.
        self._logger.debug("Second Reset.") 
        for i in range(10):
            self._device._idle()
            self._device._transaction_start()
            self._device._i2c_start()
            self._device._i2c_stop()
            response = self._device._transaction_end()
        for i in range(10):
            self._reset()

    def _load_calibration(self):
        self._logger.debug("Load Calibration.") 
        # Read calibration values and CRC
        for i in range(MS5837_PROM_ADDRESSES_NUM):
            #self._device.writeRaw8(MS5837_PROM_READ+i*2)
            self.writeRaw8(MS5837_PROM_READ+i*2)

            value = self.readRaw16(little_endian)

            if len(self._C) < MS5837_PROM_ADDRESSES_NUM:
                self._C.append(value)
            else:
                self._C[i] = value

            self._logger.debug('C{0:d} = {1:6d}'.format(i, value))
        
        crcRead = self._C[0] >> 12
        crcCalculated = self.crc4(self._C)
        if crcCalculated != crcRead:
            # @todo manage the failure.
            self._logger.warning('Failure, crcRead={0:6d}, crcCalculated={1:6d}'.format(crcRead, crcCalculated)) 

    def setFluidDensity(self, density):
      self.fluid_density = density;

    def read(self):
        try:
            # Request D1 conversion.
            self._logger.debug("D1 Conversion.") 
            #self._device.writeRaw8(MS5837_CONVERT_D1_8192)
            self.writeRaw8(MS5837_CONVERT_D1_8192)

            time.sleep(0.02); # Max conversion time per datasheet
      
            self._device.writeRaw8(MS5837_ADC_READ)

            self.D1 = self.readRaw24(little_endian) 

            self._logger.debug('D1={0:6d}.'.format(self.D1)) 

            # Request D2 conversion.
            self._logger.debug("D2 Conversion.") 
            #self._device.writeRaw8(MS5837_CONVERT_D2_8192)
            self.writeRaw8(MS5837_CONVERT_D2_8192)

            time.sleep(0.02) # Max conversion time per datasheet
      
            #self._device.writeRaw8(MS5837_ADC_READ)
            self.writeRaw8(MS5837_ADC_READ)

            self.D2 = self.readRaw24(little_endian)

            self._logger.debug('D2={0:6d}.'.format(self.D2)) 

            # Calculate the pressure and temperature.
            self.calculate()
        except:
            raise
        

    def readTestCase(self):
        # Test case.
        self._C[0] = 0
        self._C[1] = 34982
        self._C[2] = 36352
        self._C[3] = 20328
        self._C[4] = 22354
        self._C[5] = 26646
        self._C[6] = 26146
        self._C[7] = 0

        self.D1 = 4958179
        self.D2 = 6815414

        self.calculate()

    def calculate(self):
        # Given C1-C6 and D1, D2, calculated self.TEMP and P.
        # Do conversion first and then second order temp compensation.
        # Terms called.
        dT = self.D2-self._C[5]*256
        SENS = self._C[1]*32768+(self._C[3]*dT)//256
        OFF = self._C[2]*65536+(self._C[4]*dT)//128
  
  
        # Temp and P conversion.
        self.TEMP = 2000+dT*self._C[6]//8388608
        self.P = (self.D1*SENS//(2097152)-OFF)//(8192)
  
        # Second order compensation.
        if((self.TEMP//100)<20):
            Ti = (3*dT*dT)//(8589934592)
            OFFi = (3*(self.TEMP-2000)*(self.TEMP-2000))//2
            SENSi = (5*(self.TEMP-2000)*(self.TEMP-2000))//8
            if((self.TEMP//100)<-15):   # Very low temp.
                OFFi = OFFi+7*(self.TEMP+1500)*(self.TEMP+1500)
                SENSi = SENSi+4*(self.TEMP+1500)*(self.TEMP+1500)
        elif((self.TEMP//100)>=20):    #High temp
          Ti = 2*(dT*dT)//(137438953472)
          OFFi = (1*(self.TEMP-2000)*(self.TEMP-2000))//16
          SENSi = 0
  
        OFF2 = OFF-OFFi           # Calculate pressure and temp second order
        SENS2 = SENS-SENSi
  
        self.TEMP = (self.TEMP-Ti)
        self.P = (((self.D1*SENS2)//2097152-OFF2)//8192)

    def pressure(self, conversion=1.0):
      return self.P/10.0*conversion

    def temperature(self):
      return self.TEMP/100.0

    def depth(self):
      return (self.pressure(Pa)-101300)/(self.fluid_density*9.80665)

    def altitude(self):
      return (1-pow((self.pressure()/1013.25),.190284))*145366.45*.3048

    def crc4(self, n_prom):
        """
        Calculate the CRC according to the data.
        """
        n_rem = 0

        n_prom[0] = ((n_prom[0]) & 0x0FFF)
        n_prom.append(0)    

        for i in range(16):
            if i%2 == 1:
                n_rem ^= ((n_prom[i>>1]) & 0x00FF)
            else:
                n_rem ^= (n_prom[i>>1] >> 8)

            for n_bit in range(8, 0, -1):
                if ( n_rem & 0x8000 ):
                    n_rem = (n_rem << 1) ^ 0x3000
                else:
                    n_rem = (n_rem << 1)
  
        n_rem = ((n_rem >> 12) & 0x000F)

        return n_rem ^ 0x00

    def writeRaw8(self, value):
        """Write an 8-bit value on the bus (without register)."""
        is_written = False
        
        while not is_written:
            value = value & 0xFF
            self._device._idle()
            self._device._transaction_start()
            self._device._i2c_start()
            self._device._i2c_write_bytes([self._device._address_byte(False), value])
            self._device._i2c_stop()
            response = self._device._transaction_end()
        
            try:
                self._device._verify_acks(response)
                is_written = True
            except:
                self._logger.warning("Failed to find expected I2C ACK")   
        #self._logger.debug('writeRaw8={0:08b}.'.format(response))
        if LOG_LEVEL == logging.DEBUG:
            print("writeRaw8")
            print("{0:08b}".format(response[-1]))
            print("{0:08b}".format(response[-2]))        

    def readRaw16(self, little_endian=True):
        """
        Read a 16-bit value on the bus (without register).
        For more information, look at https://github.com/adafruit/Adafruit_Python_GPIO/blob/master/Adafruit_GPIO/FT232H.py#L530.
        """
        is_read = False
        
        while not is_read:
            self._device._idle()
            self._device._transaction_start()
            """
            self._device._i2c_start()
            self._device._i2c_write_bytes([self._device._address_byte(False)])
            self._device._i2c_stop()
            self._device._i2c_idle()
            """
            self._device._i2c_start()
            self._device._i2c_write_bytes([self._device._address_byte(True)])
            self._device._i2c_read_bytes(2)
            self._device._i2c_stop()
            response = self._device._transaction_end()
            try:
                self._device._verify_acks(response[:-2])
                is_read = True
            except:
                self._logger.warning("Failed to find expected I2C ACK")   
                raise

        #self._logger.debug('readRaw16={0:b} {0:b} {0:b} {0:b}.'.format(response[-1], response[-2], response[-3], response[-4])) 
        if LOG_LEVEL == logging.DEBUG:
            print("readRaw16")
            print("{0:08b}".format(response[-1]))
            print("{0:08b}".format(response[-2]))
            print("{0:08b}".format(response[-3]))

        if little_endian:
            return (response[-1] << 8) | response[-2]
        else:
            return (response[-2] << 8) | response[-1]
        
    def readRaw24(self, little_endian=True):
        """
        Read a 16-bit value on the bus (without register).
        For more information, look at https://github.com/adafruit/Adafruit_Python_GPIO/blob/master/Adafruit_GPIO/FT232H.py#L530.
        """
        is_read = False
        
        while not is_read:
            self._device._idle()
            self._device._transaction_start()
            """
            self._device._i2c_start()
            self._device._i2c_write_bytes([self._device._address_byte(False)])
            self._device._i2c_stop()
            self._device._i2c_idle()
            """
            self._device._i2c_start()
            self._device._i2c_write_bytes([self._device._address_byte(True)])
            self._device._i2c_read_bytes(3)
            self._device._i2c_stop()
            
            response = self._device._transaction_end()
            try:
                self._device._verify_acks(response[:-3])
                is_read = True
            except:
                self._logger.warning("Failed to find expected I2C ACK")  
                raise

        if LOG_LEVEL == logging.DEBUG:
            print("readRaw24")
            print("{0:08b}".format(response[-1]))
            print("{0:08b}".format(response[-2]))
            print("{0:08b}".format(response[-3]))
            print("{0:08b}".format(response[-4]))
        if little_endian:
            result = response[-1]
            result = (result << 8) | response[-2]
            result = (result << 8) | response[-3]
        else:
            result = response[-3]
            result = (result << 8) | response[-2]
            result = (result << 8) | response[-1]
        return result