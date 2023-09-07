# DFRobot_URM13
* [中文版](./README_CN.md)

DFRobot URM13 is an ultrasonic ranging sensor with open single probe. This sensor supports TRIG Pulse-triggered ranging(SR04 compatible), UART and I2C, which brings more possibilities for actual using scenarios. With a small and compact body, the sensor works well with 3.3V or 5V mainboards like Arduino, Raspberry Pi, easy to use and integrate into various applications. Besides, the UART mode employs standard Modbus-RTU protocol and integrates receive/send control output to allow users to easily expand RS485 interface using external RS485 transceiver. URM13 is much more smaller and lighter while still excellent in sensitivity, which makes it perform better than similar sensors when detecting some targets with low acoustic reflectivity. At the same time, URM13 sensor is able to automatically detect the environment and electrical noise, and complete the dynamic adjustment and calibration of sensor parameters in real-time, which ensures that it can keep stable performance in various complex application scenarios.
URM13 provides users with two built-in measuring ranges for meeting different application requirements:
1. 15-150cm small range with up to 50Hz detecting frequency, suitbale for indoor robot obstacle avoidance, etc.
2. 40-900cm large range with 10Hz frequency and high sensitivity, applicable for open field scenarios or projects that requires high accuracy and long ranging distance.
The two detecting ranges can be triggered repeatedly in actual use to realize the measurement of whole range.

![产品实物图](../../resources/images/URM13.jpg)


## Product Link (https://www.dfrobot.com/product-2161.html)
    SKU:SEN0352


## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)


## Summary

* The sensor supports UART(modbus-rtu), I2C and TRIG for data output, which brings more possibilities for actual using scenarios.<br>
* Can obtain the sensor basic information, current distance value and current temperature value.<br>
* Can configure the sensor communication address, parameters, etc.<br>
* URM13 provides users with two built-in measuring ranges for meeting different application requirements: 15-150cm small range；40-900cm large range.<br>


## Installation

The library has used modbus_tk, detect whether modbus_tk has been imported into Raspberry Pi before use, if not, run the following command to install modbus_tk library.
python2: pip install modbus_tk
python3: pip3 install modbus_tk

Download the library file before use, paste them into the specified directory, then open the Examples folder and run the demo in the folder.


## Methods

```python

    '''!
      @brief Initialize sensor
      @return Return True indicate initialization succeed, return False indicate failed
    '''
    def begin(self):

    '''!
      @brief read module basic information
      @n retrieve basic information from the sensor and buffer it into a variable that stores information:
      @n I2C interface mode: addr_I2C, PID_I2C, VID_I2C
      @n RTU interface mode: PID_RTU, VID_RTU, addr_RTU, baudrate_RTU, checkbit_RTU, stopbit_RTU
    '''
    def read_basic_info(self):

    '''!
      @brief set the module communication address, power off to save the settings, and restart for the settings to take effect
      @param addr device address to be set, I2C address range(1~127 is 0x01~0x7F), RTU address range(1~247 is 0x0001-0x00F7)
    '''
    def set_addr(self, addr):

    '''!
      @brief read the current distance value, the value of zero indicates it's not measured within the range
      @return the current distance value, unit is cm, resolution is 1cm, large range(40 - 900cm)small range(15-150cm)
    '''
    def get_distance_cm(self):

    '''!
      @brief read the current internal temperature
      @return the current internal temperature, unit is ℃, resolution is 0.1℃, signed number
    '''
    def get_internal_tempreture_C(self):

    '''!
      @brief write ambient temperature data for external temperature compensation
      @param temp written ambient temperature data, unit is ℃, resolution is 0.1℃, signed number
    '''
    def set_external_tempreture_C(self, temp=0.0):

    '''!
      @brief set measure mode
      @param mode measure mode to be set, the following patterns constitute mode:
      @n       E_INTERNAL_TEMP: use internal temperature compensation function, E_EXTERNAL_TEMP: use external temperature compensation function (users need to write external temperature)
      @n       E_TEMP_COMP_MODE_EN: enable temperature compensation function, E_TEMP_COMP_MODE_DIS: disable temperature compensation function
      @n       E_AUTO_MEASURE_MODE_EN: automatic ranging, E_AUTO_MEASURE_MODE_DIS: passive ranging
      @n       E_MEASURE_RANGE_MODE_LONG: large range measurement(40 - 900cm), E_MEASURE_RANGE_MODE_SHORT: small range measurement(15-150cm)
    '''
    def set_measure_mode(self, mode=0):

    '''!
      @brief the function to trigger measuring in passive measurement mode
      @n in passive measurement mode, the function is called once, the ranging command is sent once, and the module measures the distance once and saves the measured value into the distance register
    '''
    def passive_measurement_TRIG(self):

    '''!
      @brief get noise level of power supply, 0x00-0x0A matches noise level of 0-10
      @n The parameter indicates the influence of power supply and environment on the sensor. The smaller the noise level, the more accurate the distance value obtained by the sensor.
    '''
    def get_noise_level(self):

    '''!
      @brief ranging sensitivity setting, 0x00-0x0A:sensitivity level 0-10
      @param measure_sensitivity to set the sensor ranging sensitivity in large range (40-900cm), the smaller the value, and the higher the sensitivity, power off to save the settings, and it takes effect at once
    '''
    def set_measure_sensitivity(self, measure_sensitivity):

    '''!
      @brief UART interface mode, set the module baud rate, power off to save the settings, and restart for the settings to take effect
      @param baudrate_mode the baud rate to be set:
             E_BAUDRATE_2400---2400, E_BAUDRATE_4800---4800, E_BAUDRATE_9600---9600, 
             E_BAUDRATE_14400---14400, E_BAUDRATE_19200---19200, E_BAUDRATE_38400---38400, 
             E_BAUDRATE_57600---57600, E_BAUDRATE_115200---115200
    '''
    def set_baudrate_mode(self, baudrate_mode=E_BAUDRATE_19200):

    '''!
      @brief UART interface mode, set check bit and stop bit of the module
      @param checkbit_stopbit the mode to be set, the following patterns constitute checkbit_stopbit:
      @n      check bit:
      @n           E_CHECKBIT_NONE
      @n           E_CHECKBIT_EVEN
      @n           E_CHECKBIT_ODD
      @n      stop bit:
      @n           E_STOPBIT_0P5
      @n           E_STOPBIT_1
      @n           E_STOPBIT_1P5
      @n           E_STOPBIT_2
    '''
    def set_checkbit_stopbit(self, checkbit_stopbit = E_CHECKBIT_NONE | E_STOPBIT_1):

```


## Compatibility

* RaspberryPi Version

| Board        | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :-------: | :--------: | :------: | ------- |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |           |            |    √     |         |
| RaspberryPi4 |     √     |            |          |         |

* Python Version

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |     √     |            |          |         |
| Python3 |     √     |            |          |         |


## History

- 2021/09/26 - Version 1.0.1 released.


## Credits

Written by qsjhyy(yihuan.huang@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))

