# DFRobot_URM13
* [中文版](./README_CN.md)

DFRobot URM13 is an ultrasonic ranging sensor with open single probe. This sensor supports TRIG Pulse-triggered ranging(SR04 compatible), UART and I2C, which brings more possibilities for actual using scenarios. With a small and compact body, the sensor works well with 3.3V or 5V mainboards like Arduino, Raspberry Pi, easy to use and integrate into various applications. Besides, the UART mode employs standard Modbus-RTU protocol and integrates receive/send control output to allow users to easily expand RS485 interface using external RS485 transceiver. URM13 is much more smaller and lighter while still excellent in sensitivity, which makes it perform better than similar sensors when detecting some targets with low acoustic reflectivity. At the same time, URM13 sensor is able to automatically detect the environment and electrical noise, and complete the dynamic adjustment and calibration of sensor parameters in real-time, which ensures that it can keep stable performance in various complex application scenarios.
URM13 provides users with two built-in measuring ranges for meeting different application requirements:
1. 15-150cm small range with up to 50Hz detecting frequency, suitbale for indoor robot obstacle avoidance, etc.
2. 40-900cm large range with 10Hz frequency and high sensitivity, applicable for open field scenarios or projects that requires high accuracy and long ranging distance.
The two detecting ranges can be triggered repeatedly in actual use to realize the measurement of whole range.

![产品实物图](./resources/images/URM13.jpg)


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
* URM13 provides users with two built-in measuring ranges for meeting different application requirements, 15-150cm small range and 40-900cm large range.<br>


## Installation

Download the library file (https://github.com/DFRobot/DFRobot_URM13) and its dependencies (https://github.com/DFRobot/DFRobot_RTU) before use, paste them into the \Arduino\libraries directory, then open the sample folder and run the demo in the folder.


## Methods

```C++

  /**
   * @fn begin
   * @brief initialization function
   * @return int type, means returning initialization status
   * @retval 0 NO_ERROR
   * @retval -1 ERR_DATA_BUS
   * @retval -2 ERR_IC_VERSION
   */
  virtual int begin(void);

  /**
   * @fn refreshBasicInfo
   * @brief retrieve the basic information from the sensor and buffer it into the structure that stores information:
   * @n       I2C interface mode: addr_I2C, PID_I2C, VID_I2C
   * @n       RTU interface mode: PID_RTU, VID_RTU, addr_RTU, baudrate_RTU, checkbit_RTU, stopbit_RTU
   * @return None
   */
  void refreshBasicInfo(void);

  /**
   * @fn setADDR
   * @brief set the module communication address, power off to save the settings, and restart for the settings to take effect
   * @param addr device address to be set, I2C address range(1~127 is 0x01~0x7F), RTU address range(1~247 is 0x0001-0x00F7)
   * @return None
   */
  void setADDR(uint8_t addr);

  /**
   * @fn getDistanceCm
   * @brief read the current distance value, the value of zero indicates it’s not measured within the range
   * @return the current distance value, unit is cm, large range(40 - 900cm)small range(15-150cm)
   */
  uint16_t getDistanceCm(void);

  /**
   * @fn getInternalTempretureC
   * @brief read the current internal temperature
   * @return the current internal temperature value, unit is ℃, resolution is 0.1℃, signed number
   */
  float getInternalTempretureC(void);

  /**
   * @fn setExternalTempretureC
   * @brief write ambient temperature data for external temperature compensation, the setting is invalid when out of range
   * @param temp written ambient temperature data, unit is ℃, resolution is 0.1℃, signed number, range:-10℃～＋70℃
   * @return None
   */
  void setExternalTempretureC(float temp);

  /**
   * @fn setMeasureMode
   * @brief set measure mode
   * @param mode measure mode to be set, the following patterns constitute mode:
   * @n       eInternalTemp: use internal temperature compensation function, eExternalTemp: use external temperature compensation function (users need to write external temperature)
   * @n       eTempCompModeEn: enable temperature compensation function, eTempCompModeDis: disable temperature compensation function
   * @n       eAutoMeasureModeEn: automatic ranging, eAutoMeasureModeDis: passive ranging
   * @n       eMeasureRangeModeLong: large range measurement(40 - 900cm), eMeasureRangeModeShort: small range measurement(15-150cm)
   * @return None
   */
  void setMeasureMode(uint8_t mode);

  /**
   * @fn passiveMeasurementTRIG
   * @brief the function to trigger measuring in passive measurement mode
   * @n in passive measurement mode, the function is called once, the ranging command is sent once, and the module measures the distance once and saves the measured value into the distance register
   * @return None
   */
  void passiveMeasurementTRIG(void);

  /**
   * @fn getNoiseLevel
   * @brief get noise level of power supply, the smaller the noise level, the more accurate the distance value obtained by the sensor
   * @return the parameter indicates the influence of power supply and environment on the sensor. 0x00-0x0A matches noise level of 0-10.
   */
  uint8_t getNoiseLevel(void);

  /**
   * @fn setMeasureSensitivity
   * @brief ranging sensitivity setting, 0x00-0x0A:sensitivity level is 0-10
   * @param mode to set the sensor ranging sensitivity in large range (40-900cm), the smaller the value, and the higher the sensitivity, power off to save the settings, and it takes effect at once
   * @return None
   */
  void setMeasureSensitivity(uint8_t mode);

  /**
   * @fn setBaudrateMode
   * @brief UART interface mode, set the module baud rate, power off to save the settings, and restart for the settings to take effect
   * @param mode the baud rate to be set:
   * @n     eBaudrate2400---2400, eBaudrate4800---4800, eBaudrate9600---9600, 
   * @n     eBaudrate14400---14400, eBaudrate19200---19200, eBaudrate38400---38400, 
   * @n     eBaudrate57600---57600, eBaudrate115200---115200
   * @return None
   */
  void setBaudrateMode(eBaudrateMode_t mode);

  /**
   * @fn setCheckbitStopbit
   * @brief UART interface mode, set check bit and stop bit of the module
   * @param mode the mode to be set, the following patterns constitute mode:
   * @n     check bit:
   * @n           eCheckBitNone
   * @n           eCheckBitEven
   * @n           eCheckBitOdd
   * @n     stop bit:
   * @n           eStopBit0P5
   * @n           eStopBit1
   * @n           eStopBit1P5
   * @n           eStopBit2
   * @return None
   */
  void setCheckbitStopbit(uint16_t mode);

```


## Compatibility

MCU                | SoftwareSerial | HardwareSerial |
------------------ | :------------: | :------------: |
Arduino Uno        |       √        |       X        |
Mega2560           |       √        |       √        |
Leonardo           |       √        |       √        |
ESP32              |       X        |       √        |
ESP8266            |       √        |       X        |
micro:bit          |       X        |       X        |
FireBeetle M0      |       X        |       √        |
raspberry          |       X        |       √        |

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | :---:
Arduino Uno        |      √       |              |             |
Arduino MEGA2560   |      √       |              |             |
Arduino Leonardo   |      √       |              |             |
FireBeetle-ESP8266 |      √       |              |             |
FireBeetle-ESP32   |      √       |              |             |
FireBeetle-M0      |      √       |              |             |
Micro:bit          |      √       |              |             |


## History

- 2021/09/22 - Version 1.0.1 released.


## Credits

Written by qsjhyy(yihuan.huang@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))

