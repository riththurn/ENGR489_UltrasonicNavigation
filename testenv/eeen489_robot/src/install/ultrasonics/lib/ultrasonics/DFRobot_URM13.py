# -*- coding: utf-8 -*
'''!
  @file  DFRobot_URM13.py
  @brief  Define the infrastructure of DFRobot_URM13 class.
  @n      get URM13 basic information, measuring distance and internal temperature, select the sensor communication interface, and set sensor parameters
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license  The MIT License (MIT)
  @author  [qsjhyy](yihuan.huang@dfrobot.com)
  @version  V1.0
  @date  2021-09-22
  @url  https://github.com/DFRobot/DFRobot_URM13
'''
import sys
import time

import smbus

import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

import logging
from ctypes import *


logger = logging.getLogger()
#logger.setLevel(logging.INFO)  #Display all print information
logger.setLevel(logging.FATAL)  #If you don’t want to display too many prints, only print errors, please use this option
ph = logging.StreamHandler()
formatter = logging.Formatter("%(asctime)s - [%(filename)s %(funcName)s]:%(lineno)d - %(levelname)s: %(message)s")
ph.setFormatter(formatter) 
logger.addHandler(ph)

## default I2C communication address
URM13_DEFAULT_ADDR_I2C   = 0x12
## default RTU communication address
URM13_DEFAULT_ADDR_RTU   = 0x000D

# URM13 I2C register address
## sensor I2C address register, power off to save the settings, and restart for the settings to take effect, the default value is 0x12
URM13_ADDR_REG_I2C               = 0x00
## sensor PID register, the bit is used for product check[can detect the sensor type], the default value is 0x02
URM13_PID_REG_I2C                = 0x01
## sensor VID register, firmware revision number: default value 0x10 represents V1.0
URM13_VID_REG_I2C                = 0x02
## distance value register high bit, the marker is 1cm
URM13_DISTANCE_MSB_REG_I2C       = 0x03
## distance value register low bit
URM13_DISTANCE_LSB_REG_I2C       = 0x04
## internal temperature register high bit, the marker is 0.1℃, data type is signed
URM13_INTERNAL_TEMP_MSB_REG_I2C  = 0x05
## internal temperature register low bit
URM13_INTERNAL_TEMP_LSB_REG_I2C  = 0x06
## external temperature compensation data register high bit, write ambient temperature data to the register for external temperature compensation, the marker is 0.1℃, data type is signed
URM13_EXTERNAL_TEMP_MSB_REG_I2C  = 0x07
## external temperature compensation data register low bit
URM13_EXTERNAL_TEMP_LSB_REG_I2C  = 0x08
## configure register, power off to save the settings, and it takes effect at once, the default value is 0x04
URM13_CONFIG_REG_I2C             = 0x09
## command register, write 1 to the bit, trigger once ranging, write 0 to the bit and it's ignored
URM13_COMMAND_REG_I2C            = 0x0A
## power supply noise level register, 0x00-0x0A matches noise level of 0-10。 the parameter indicates the influence of power supply and environment on the sensor;
## the smaller the noise level, the more accurate the distance value obtained by the sensor
URM13_NOISE_REG_I2C              = 0x0B
## ranging sensibility setting register, 0x00-0x0A:sensitivity level 0-10。 to set the sensor ranging sensitivity in large range (40-900cm);
## the smaller the value, and the higher the sensitivity, power off to save the settings, and it takes effect at once
URM13_SENSITIVITY_REG_I2C        = 0x0C

# URM13 RTU register address
## module PID memory register, the bit is used for product check[can detect the module type], the default value is 0x0003
URM13_PID_REG_RTU                 = 0x00
## module VID memory register, the bit is used for version check[0x0010 means V0.0.1.0]
URM13_VID_REG_RTU                 = 0x01
## module address register, the default value is 0x000D, module device address(1~247), power off to save the settings, and restart for the settings to take effect
URM13_ADDR_REG_RTU                = 0x02
## module baud rate memory register, the default value is 0x0005, power off to save the settings, and restart for the settings to take effect
URM13_BAUDRATE_REG_RTU            = 0x03
## module check bit and stop bit memory register, the default value is 0x0001, power off to save the settings, and restart for the settings to take effect
URM13_CHECKBIT_STOPBIT_REG_RTU    = 0x04
## distance value register, the marker is 1cm
URM13_DISTANCE_REG_RTU            = 0x05
## internal temperature register, the marker is 0.1℃, data type is signed
URM13_INTERNAL_TEMP_REG_RTU       = 0x06
## external temperature compensation data register, write ambient temperature data to the register for external temperature compensation, the marker is 0.1℃, data type is signed
URM13_EXTERNAL_TEMP_REG_RTU       = 0x07
## configure register, power off to save the settings, and it takes effect at once, the default value is 0x04
URM13_CONFIG_REG_RTU              = 0x08
## power supply noise level register, 0x00-0x0A matches noise level of 0-10。 the parameter indicates the influence of power supply and environment on the sensor;
## the smaller the noise level, the more accurate the distance value obtained by the sensor
URM13_NOISE_REG_RTU               = 0x09
## ranging sensibility setting register, 0x00-0x0A:sensitivity level 0-10。 to set the sensor ranging sensitivity in large range (40-900cm);
## the smaller the value, and the higher the sensitivity, power off to save the settings, and it takes effect at once
URM13_SENSITIVITY_REG_RTU         = 0x0A

class DFRobot_URM13(object):
    '''!
      @brief define DFRobot_URM13 class
      @details to drive URM13
    '''

    E_INTERNAL_TEMP = 0
    E_EXTERNAL_TEMP = 1

    E_TEMP_COMP_MODE_EN = 0
    E_TEMP_COMP_MODE_DIS = 1<<1

    E_AUTO_MEASURE_MODE_EN = 0
    E_AUTO_MEASURE_MODE_DIS = 1<<2

    E_MEASURE_RANGE_MODE_LONG = 0
    E_MEASURE_RANGE_MODE_SHORT = 1<<4

    E_RTU_INTERFACE = 0
    E_I2C_INTERFACE = 1
    # E_TRIG_INTERFACE = 2

    addr_I2C = 0
    PID_I2C = 0
    VID_I2C = 0

    PID_RTU = 0
    VID_RTU = 0
    addr_RTU = 0
    baudrate_RTU = 0
    checkbit_RTU = 0
    stopbit_RTU = 0

    def __init__(self, interface_URM13):
        '''!
          @brief Module init
          @param interface_URM13 the current interface mode: E_RTU_INTERFACE, E_I2C_INTERFACE
        '''
        # initialize configured parameters
        self._device_interface = interface_URM13
        self._measure_range_mode = self.E_MEASURE_RANGE_MODE_SHORT

    def begin(self):
        '''!
          @brief Initialize sensor
          @return  return initialization status
          @retval True indicate initialization succeed
          @retval False indicate initialization failed
        '''
        ret = True

        if self._device_interface == self.E_RTU_INTERFACE:
            id = self._read_reg(URM13_PID_REG_RTU, 1)[0]
        elif self._device_interface == self.E_I2C_INTERFACE:
            id = self._read_reg(URM13_PID_REG_I2C, 1)[0]
        else:
          id = 0

        logger.info(id)
        if id not in  [0x02, 0x0003]:   # 0x02 represents the id read by I2C 8-bit register, 0x0003 represents the id read by modbus 16-bit register
            ret = False

        return ret

    def read_basic_info(self):
        '''!
          @brief read module basic information
          @n retrieve basic information from the sensor and buffer it into a variable that stores information:
          @n I2C interface mode: addr_I2C, PID_I2C, VID_I2C
          @n RTU interface mode: PID_RTU, VID_RTU, addr_RTU, baudrate_RTU, checkbit_RTU, stopbit_RTU
        '''
        if self._device_interface == self.E_RTU_INTERFACE:
            info_buf = self._read_reg(URM13_PID_REG_RTU, 5)   # rtu basic information length is 5 16-bit data
            self.PID_RTU = info_buf[0]
            self.VID_RTU = info_buf[1]
            self.addr_RTU = info_buf[2]
            self.baudrate_RTU = info_buf[3]
            self.checkbit_RTU = (info_buf[4] & 0xFF00) >> 8
            self.stopbit_RTU = info_buf[4] & 0x00FF
        elif self._device_interface == self.E_I2C_INTERFACE:
            info_buf = self._read_reg(URM13_ADDR_REG_I2C, 3)   # I2C basic information length is 3 bytes
            self.addr_I2C = info_buf[0]
            self.PID_I2C = info_buf[1]
            self.VID_I2C = info_buf[2]

    def set_addr(self, addr):
        '''!
          @brief set the module communication address, power off to save the settings, and restart for the settings to take effect
          @param addr device address to be set, I2C address range(1~127 is 0x01~0x7F), RTU address range(1~247 is 0x0001-0x00F7)
        '''
        if self._device_interface == self.E_RTU_INTERFACE:
            if(0x0001 <= addr) and (0x00F7 >= addr):
              self._write_reg(URM13_ADDR_REG_RTU, [addr])

        elif self._device_interface == self.E_I2C_INTERFACE:
            if(0x01 <= addr) and (0x7F >= addr):
              self._write_reg(URM13_ADDR_REG_I2C, [addr])

    def get_distance_cm(self):
        '''!
          @brief read the current distance value, the value of zero indicates it's not measured within the range
          @return the current distance value, unit is cm, resolution is 1cm, large range(40 - 900cm)small range(15-150cm)
        '''
        if self._device_interface == self.E_RTU_INTERFACE:
            distance = self._read_reg(URM13_DISTANCE_REG_RTU, 1)[0]

        elif self._device_interface == self.E_I2C_INTERFACE:
            data = self._read_reg(URM13_DISTANCE_MSB_REG_I2C, 2)
            distance = (data[0] << 8) | data[1]
        else:
            distance = 0

        if self.E_MEASURE_RANGE_MODE_LONG == self._measure_range_mode:
          if (40 > distance) or (900 < distance):
            distance = 0
        elif self.E_MEASURE_RANGE_MODE_SHORT == self._measure_range_mode:
          if (15 > distance) or (150 < distance):
            distance = 0

        return distance

    def get_internal_tempreture_C(self):
        '''!
          @brief read the current internal temperature
          @return the current internal temperature value, unit is ℃, resolution is 0.1℃, signed number
        '''
        if self._device_interface == self.E_RTU_INTERFACE:
            internal_temp = self._read_reg(URM13_INTERNAL_TEMP_REG_RTU, 1)[0]

        elif self._device_interface == self.E_I2C_INTERFACE:
            data = self._read_reg(URM13_INTERNAL_TEMP_MSB_REG_I2C, 2)
            internal_temp = (data[0] << 8) | data[1]
        else:
            internal_temp = 0

        return self._uint16_to_int(internal_temp) / 10.0

    def set_external_tempreture_C(self, temp=0.0):
        '''!
          @brief write ambient temperature data for external temperature compensation, the setting is invalid when out of range
          @param temp written ambient temperature data, unit is ℃, resolution is 0.1℃, signed number, range:-10℃～＋70℃
        '''
        external_temp = int(temp * 10)

        if self._device_interface == self.E_RTU_INTERFACE:
            self._write_reg(URM13_EXTERNAL_TEMP_REG_RTU, [external_temp])
        elif self._device_interface == self.E_I2C_INTERFACE:
            data = [(external_temp & 0xFF00) >> 8, external_temp & 0x00FF]
            self._write_reg(URM13_EXTERNAL_TEMP_MSB_REG_I2C, data)

    def set_measure_mode(self, mode=0):
        '''!
          @brief set measure mode
          @param mode measure mode to be set, the following patterns constitute mode:
          @n       E_INTERNAL_TEMP: use internal temperature compensation function, E_EXTERNAL_TEMP: use external temperature compensation function (users need to write external temperature)
          @n       E_TEMP_COMP_MODE_EN: enable temperature compensation function, E_TEMP_COMP_MODE_DIS: disable temperature compensation function
          @n       E_AUTO_MEASURE_MODE_EN: automatic ranging, E_AUTO_MEASURE_MODE_DIS: passive ranging
          @n       E_MEASURE_RANGE_MODE_LONG: large range measurement(40 - 900cm), E_MEASURE_RANGE_MODE_SHORT: small range measurement(15-150cm)
        '''
        if self._device_interface == self.E_RTU_INTERFACE:
            self._write_reg(URM13_CONFIG_REG_RTU, [mode])
        elif self._device_interface == self.E_I2C_INTERFACE:
            self._write_reg(URM13_CONFIG_REG_I2C, [mode])

        if self._measure_range_mode & self.E_MEASURE_RANGE_MODE_SHORT:
          self._measure_range_mode = self.E_MEASURE_RANGE_MODE_SHORT
        else:
          self._measure_range_mode = self.E_MEASURE_RANGE_MODE_LONG


    def passive_measurement_TRIG(self):
        '''!
          @brief the function to trigger measuring in passive measurement mode
          @n     in passive measurement mode, the function is called once, the ranging command is sent once, and the module measures the distance once and saves the measured value into the distance register
        '''
        if self._device_interface == self.E_RTU_INTERFACE:
            data = self._read_reg(URM13_CONFIG_REG_RTU, 1)
            data[0] |= (1 << 3)
            self._write_reg(URM13_CONFIG_REG_RTU, data)

        elif self._device_interface == self.E_I2C_INTERFACE:
            data = [0x01]
            self._write_reg(URM13_COMMAND_REG_I2C, data)

        time.sleep(0.1)

    def get_noise_level(self):
        '''!
          @brief get noise level of power supply, 0x00-0x0A matches noise level of 0-10
          @n     the parameter indicates the influence of power supply and environment on the sensor. The smaller the noise level, the more accurate the distance value obtained by the sensor
        '''
        if self._device_interface == self.E_RTU_INTERFACE:
            noise_level = self._read_reg(URM13_NOISE_REG_RTU, 1)[0]

        elif self._device_interface == self.E_I2C_INTERFACE:
            noise_level = self._read_reg(URM13_NOISE_REG_I2C, 1)[0]
        else:
            noise_level = 0

        return noise_level

    def set_measure_sensitivity(self, measure_sensitivity):
        '''!
          @brief ranging sensitivity setting, 0x00-0x0A:sensitivity level 0-10
          @param measure_sensitivity to set the sensor ranging sensitivity in large range (40-900cm), the smaller the value, and the higher the sensitivity, power off to save the settings, and it takes effect at once
        '''
        if self._device_interface == self.E_RTU_INTERFACE:
            self._write_reg(URM13_SENSITIVITY_REG_RTU, [measure_sensitivity])
        elif self._device_interface == self.E_I2C_INTERFACE:
            self._write_reg(URM13_SENSITIVITY_REG_I2C, [measure_sensitivity])

    def _uint16_to_int(self, num):
        '''!
          @brief Convert the incoming uint16 type data to int type
          @param num unsigned 16-bit data that need to be converted to signed integer(uint16_t)
          @return data converted to int type
        '''
        if(num > 32767):
            num = num - 65536
        return num

    def _write_reg(self, reg, data):
        '''!
          @brief writes data to a register
          @param reg register address
          @param data written data
        '''
        # Low level register writing, not implemented in base class
        raise NotImplementedError()

    def _read_reg(self, reg, length):
        '''!
          @brief read the data from the register
          @param reg register address
          @param length read data length
        '''
        # Low level register writing, not implemented in base class
        raise NotImplementedError()


class DFRobot_URM13_I2C(DFRobot_URM13):
    '''!
      @brief define DFRobot_URM13_I2C class
      @details to use I2C drive URM13 
    '''

    def __init__(self, i2c_addr=0x12, bus=1, interface_URM13=DFRobot_URM13.E_I2C_INTERFACE):
        '''!
          @brief Module I2C communication init
          @param i2c_addr I2C communication address
          @param bus I2C communication bus
          @param interface_URM13 the current interface mode: E_RTU_INTERFACE, E_I2C_INTERFACE
        '''
        self._i2c_addr = i2c_addr
        self._i2c = smbus.SMBus(bus)
        super(DFRobot_URM13_I2C, self).__init__(interface_URM13)

    def _write_reg(self, reg, data):
        '''!
          @brief writes data to a register
          @param reg register address
          @param data written data
        '''
        if isinstance(data, int):
            data = [data]
            #logger.info(data)
        self._i2c.write_i2c_block_data(self._i2c_addr, reg, data)

    def _read_reg(self, reg, length):
        '''!
          @brief read the data from the register
          @param reg register address
          @param length read data length
        '''
        return self._i2c.read_i2c_block_data(self._i2c_addr, reg, length)


class DFRobot_URM13_RTU(DFRobot_URM13):
    '''!
      @brief define DFRobot_URM13_RTU class
      @details to use RTU drive URM13
    '''

    E_BAUDRATE_2400 = 0x0001
    E_BAUDRATE_4800 = 0x0002
    E_BAUDRATE_9600 = 0x0003
    E_BAUDRATE_14400 = 0x0004
    E_BAUDRATE_19200 = 0x0005
    E_BAUDRATE_38400 = 0x0006
    E_BAUDRATE_57600 = 0x0007
    E_BAUDRATE_115200 = 0x0008

    E_CHECKBIT_NONE = 0x0000<<8
    E_CHECKBIT_EVEN = 0x0001<<8
    E_CHECKBIT_ODD = 0x0002<<8

    E_STOPBIT_0P5 = 0x0000
    E_STOPBIT_1 = 0x0001
    E_STOPBIT_1P5 = 0x0002
    E_STOPBIT_2 = 0x0003

    def __init__(self, addr=0x000D, port="/dev/ttyAMA0", baud = 19200, bytesize = 8, parity = 'N', stopbit = 1, xonxoff=0, interface_URM13=DFRobot_URM13.E_RTU_INTERFACE):
        '''!
          @brief Module RTU communication init
          @param addr modbus communication address
          @param port modbus communication serial port
          @param baud modbus communication baud rate
          @param bytesize modbus communication data bits
          @param parity modbus communication check bit
          @param stopbit modbus communication stop bit
          @param xonxoff modbus communication synchronous and asynchronous setting
          @param interface_URM13 the current interface mode: E_RTU_INTERFACE, E_I2C_INTERFACE
        '''
        self._modbus_addr = addr

        self._DFRobot_RTU = modbus_rtu.RtuMaster(
            serial.Serial(port, baud, bytesize, parity, stopbit, xonxoff)
        )
        self._DFRobot_RTU.set_timeout(0.5)
        self._DFRobot_RTU.set_verbose(False)

        super(DFRobot_URM13_RTU, self).__init__(interface_URM13)

    def set_baudrate_mode(self, baudrate_mode=E_BAUDRATE_19200):
        '''!
          @brief UART interface mode, set the module baud rate, power off to save the settings, and restart for the settings to take effect
          @param baudrate_mode the baud rate to be set:
          @n     E_BAUDRATE_2400---2400, E_BAUDRATE_4800---4800, E_BAUDRATE_9600---9600, 
          @n     E_BAUDRATE_14400---14400, E_BAUDRATE_19200---19200, E_BAUDRATE_38400---38400, 
          @n     E_BAUDRATE_57600---57600, E_BAUDRATE_115200---115200
        '''
        self._write_reg(URM13_BAUDRATE_REG_RTU, [baudrate_mode])

    def set_checkbit_stopbit(self, checkbit_stopbit = E_CHECKBIT_NONE | E_STOPBIT_1):
        '''!
          @brief UART interface mode, set check bit and stop bit of the module
          @param checkbit_stopbit the mode to be set, the following patterns constitute checkbit_stopbit:
          @n     check bit:
          @n          E_CHECKBIT_NONE
          @n          E_CHECKBIT_EVEN
          @n          E_CHECKBIT_ODD
          @n     atop bit:
          @n          E_STOPBIT_0P5
          @n          E_STOPBIT_1
          @n          E_STOPBIT_1P5
          @n          E_STOPBIT_2
        '''
        self._write_reg(URM13_CHECKBIT_STOPBIT_REG_RTU, [checkbit_stopbit])

    def _write_reg(self, reg, data):
        '''!
          @brief writes data to a register
          @param reg register address
          @param data written data
          @return Write register address, and write length
        '''
        if isinstance(data, int):
            data = [data]
            #logger.info(data)
        ret = self._DFRobot_RTU.execute(self._modbus_addr, cst.WRITE_MULTIPLE_REGISTERS, reg, output_value=data)
        # logger.info(ret)
        return ret

    def _read_reg(self, reg, length):
        '''!
          @brief read the data from the register
          @param reg register address
          @param length read data length
          @return list: The value list of the holding register.
        '''
        return list(self._DFRobot_RTU.execute(self._modbus_addr, cst.READ_HOLDING_REGISTERS, reg, length))
