/*!
 * @file  DFRobot_URM13.h
 * @brief  DFRobot_URM13.h detailed description for DFRobot_URM13.cpp
 * @n      DFRobot_URM13.h has defined read and write functions of device information register and device function register, declared the sensor function API
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2021-09-16
 * @url  https://github.com/DFRobot/DFRobot_URM13
 */
#ifndef __DFROBOT_URM13_H__
#define __DFROBOT_URM13_H__

#include <Arduino.h>
#include <Wire.h>
#include <Stream.h>
#include <DFRobot_RTU.h>

// #define ENABLE_DBG   //!< open the macro and you can see the details of the program
#ifdef ENABLE_DBG
  #define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
  #define DBG(...)
#endif

#define URM13_DEFAULT_ADDR_I2C   uint8_t(0x12)        ///< default I2C communication address
#define URM13_DEFAULT_ADDR_RTU   uint16_t(0x000D)     ///< default RTU communication address

/* URM13 I2C register address */
#define URM13_ADDR_REG_I2C               uint8_t(0x00)   ///< sensor I2C address register, power off to save the settings, and restart for the settings to take effect, the default value is 0x12
#define URM13_PID_REG_I2C                uint8_t(0x01)   ///< sensor PID register, the bit is used for product check[can detect the sensor type], the default value is 0x02
#define URM13_VID_REG_I2C                uint8_t(0x02)   ///< sensor VID register, firmware revision number: the default value 0x10 represents V1.0
#define URM13_DISTANCE_MSB_REG_I2C       uint8_t(0x03)   ///< distance value register high bit, the marker is 1cm
#define URM13_DISTANCE_LSB_REG_I2C       uint8_t(0x04)   ///< distance value register low bit
#define URM13_INTERNAL_TEMP_MSB_REG_I2C  uint8_t(0x05)   ///< internal temperature register high bit, the marker is 0.1℃, data type is signed
#define URM13_INTERNAL_TEMP_LSB_REG_I2C  uint8_t(0x06)   ///< internal temperature register low bit
#define URM13_EXTERNAL_TEMP_MSB_REG_I2C  uint8_t(0x07)   ///< external temperature compensation data register high bit, write ambient temperature data to the register for external temperature compensation, the marker is 0.1℃, data type is signed
#define URM13_EXTERNAL_TEMP_LSB_REG_I2C  uint8_t(0x08)   ///< external temperature compensation data register low bit
#define URM13_CONFIG_REG_I2C             uint8_t(0x09)   ///< configure register, power off to save the settings, and it takes effect at once, the default value is 0x04
#define URM13_COMMAND_REG_I2C            uint8_t(0x0A)   ///< command register, write 1 to the bit, trigger once ranging, write 0 to the bit and it's ignored
#define URM13_NOISE_REG_I2C              uint8_t(0x0B)   ///< power supply noise level register, 0x00-0x0A matches noise level of 0-10。 the parameter indicates the influence of power supply and environment on the sensor;
                                                         ///< the smaller the noise level, the more accurate the distance value obtained by the sensor
#define URM13_SENSITIVITY_REG_I2C        uint8_t(0x0C)   ///< ranging sensibility setting register, 0x00-0x0A:sensitivity level 0-10。 to set the sensor ranging sensitivity in large range (40-900cm);
                                                         ///< the smaller the value, and the higher the sensitivity, power off to save the settings, and it takes effect at once

/* URM13 RTU register address */
#define URM13_PID_REG_RTU                 uint16_t(0x00)   ///< module PID memory register, the bit is used for product check[can detect the module type], the default value is 0x0003
#define URM13_VID_REG_RTU                 uint16_t(0x01)   ///< module VID memory register, the bit is used for version check[0x0010 means V0.0.1.0]
#define URM13_ADDR_REG_RTU                uint16_t(0x02)   ///< module address register, the default value is 0x000D, module device address(1~247), power off to save the settings, and restart for the settings to take effect
#define URM13_BAUDRATE_REG_RTU            uint16_t(0x03)   ///< module baud rate memory register, the default value is 0x0005, power off to save the settings, and restart for the settings to take effect
#define URM13_CHECKBIT_STOPBIT_REG_RTU    uint16_t(0x04)   ///< module check bit and stop bit memory register, the default value is 0x0001, power off to save the settings, and restart for the settings to take effect
#define URM13_DISTANCE_REG_RTU            uint16_t(0x05)   ///< distance value register, the marker is 1cm
#define URM13_INTERNAL_TEMP_REG_RTU       uint16_t(0x06)   ///< internal temperature register, the marker is 0.1℃, data type is signed
#define URM13_EXTERNAL_TEMP_REG_RTU       uint16_t(0x07)   ///< external temperature compensation data register, write ambient temperature data to the register for external temperature compensation, the marker is 0.1℃, data type is signed
#define URM13_CONFIG_REG_RTU              uint16_t(0x08)   ///< configure register, power off to save the settings, and it takes effect at once, the default value is 0x04
#define URM13_NOISE_REG_RTU               uint16_t(0x09)   ///< power supply noise level register, 0x00-0x0A matches noise level of 0-10. the parameter indicates the influence of power supply and environment on the sensor;
                                                           ///< the smaller the noise level, the more accurate the distance value obtained by the sensor
#define URM13_SENSITIVITY_REG_RTU         uint16_t(0x0A)   ///< ranging sensibility setting register, 0x00-0x0A:sensitivity level 0-10. to set the sensor ranging sensitivity in large range (40-900cm);
                                                           ///< the smaller the value, and the higher the sensitivity, power off to save the settings, and it takes effect at once

#define    VELOCITY_TEMP(temp)       ( ( 331.5 + 0.6 * (float)( temp ) ) * 100 / 1000000.0 )   ///< The ultrasonic velocity (cm/us) compensated by temperature


class DFRobot_URM13
{
public:
  #define NO_ERR             0     ///< No error
  #define ERR_DATA_BUS      (-1)   ///< data bus error
  #define ERR_IC_VERSION    (-2)   ///< the chip version isn't suitable

  /**
   * @struct sPWRCTRL_t
   * @brief control register
   * @note register structure:
   * @n ----------------------------------------------------------------------------------------------------
   * @n | b7 | b6 | b5 |        b4        |       b3       |        b2       |      b1      |      b0      |
   * @n ----------------------------------------------------------------------------------------------------
   * @n |   reserved   | measureRangeMode | measureTrigger | autoMeasureMode | tempCompMode | tempCompPick |
   * @n ----------------------------------------------------------------------------------------------------
  */
  typedef struct
  {
    uint8_t   tempCompPick: 1; /*!< power is 0, 0:use internal temperature compensation function, 1:use external temperature compensation function (users need to write external temperature) */
    uint8_t   tempCompMode: 1; /*!< power is 0, 0:enable temperature compensation function, 1:disable temperature compensation function */
    uint8_t   autoMeasureMode: 1; /*!< power is 0, 0:automatic ranging, 1:passive ranging */
    uint8_t   measureTrigger: 1; /*!< power is 0, in passive mode, write 1 to the bit and the sensor will complete a distance measurement, bits after the position 1 will automatically be reset */
    uint8_t   measureRangeMode: 1; /*!<  power is 0, 0:large range measurement(40-900cm), 1:small range measurement(15-150cm), power off to save the settings, and it takes effect at once */
    uint8_t   reserved: 3; /*!< the reserved bit */
  } __attribute__ ((packed)) sPWRCTRL_t;

  /**
   * @struct sBasicInfoI2C_t
   * @brief device information structure in I2C mode
   */
  typedef struct
  {
    uint8_t addr;   /**< I2C communication address, the default value is 0x12, module device address(1~127) */
    uint8_t PID;    /**< I2C mode module PID, default value 0x02 the bit is used for product check[can detect the sensor type] */
    uint8_t VID;    /**< I2C mode module VID, firmware revision number:0x10 represents V1.0 */
  }sBasicInfoI2C_t;

  /**
   * @struct sBasicInfoRTU_t
   * @brief device information structure in modbus mode
   */
  typedef struct
  {
    uint16_t PID;        /**< modbus mode module PID, the default value is 0x03 the bit is used for product check[can detect the sensor type] */
    uint16_t VID;        /**< modbus mode module VID, firmware revision number:0x10 represents V1.0 */
    uint16_t addr;       /**< modbus communication address, the default value is 0x0D, module device address(1~247) */
    uint16_t baudrate;   /**< modbus mode module baud rate，the default value is 0x0005 */
    uint8_t checkbit;    /**< modbus mode module check bit */
    uint8_t stopbit;     /**< modbus mode module stop bit */
  }sBasicInfoRTU_t;

  /**
   * @enum  eTempCompPick_t
   * @brief  source selection of temperature for compensation
   */
  typedef enum
  {
    eInternalTemp,       /**< use internal temperature compensation function */
    eExternalTemp = 1,   /**< use external temperature compensation function (users need to write external temperature) */
  }eTempCompPick_t;

  /**
   * @enum  eTempCompMode_t
   * @brief  temperature compensation enabling
   */
  typedef enum
  {
    eTempCompModeEn,           /**< enable temperature compensation function */
    eTempCompModeDis = 1<<1,   /**< disable temperature compensation function */
  }eTempCompMode_t;

  /**
   * @enum  eAutoMeasureMode_t
   * @brief  measure mode selection
   */
  typedef enum
  {
    eAutoMeasureModeEn,           /**< automatic ranging */
    eAutoMeasureModeDis = 1<<2,   /**< passive ranging */
  }eAutoMeasureMode_t;

  /**
   * @enum  eMeasureRangeMode_t
   * @brief  measure range selection
   */
  typedef enum
  {
    eMeasureRangeModeLong,           /**< large range measurement(40 - 900cm) */
    eMeasureRangeModeShort = 1<<4,   /**< small range measurement(15-150cm) */
  }eMeasureRangeMode_t;

  /**
   * @enum  eInterfaceMode_t
   * @brief  interface mode selection
   */
  typedef enum
  {
    eRtuInterface = 0,   /**< modbusRTU communication mode */
    eI2cInterface,       /**< I2C communication mode */
    // eTrigInterface,      /**< TRIG communication mode */
  }eInterfaceMode_t;

public:
  /**
   * @fn DFRobot_URM13
   * @brief constructor
   * @param interfaceURM13 the current interface mode: eRtuInterface, eI2cInterface
   * @return None
   */
  DFRobot_URM13(eInterfaceMode_t interfaceURM13);

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
   * @brief read the current distance value, the value of zero indicates it's not measured within the range
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
   * @brief ranging sensitivity setting, 0x00-0x0A:sensitivity level 0-10
   * @param mode to set the sensor ranging sensitivity in large range (40-900cm), the smaller the value, and the higher the sensitivity, power off to save the settings, and it takes effect at once
   * @return None
   */
  void setMeasureSensitivity(uint8_t mode);


protected:
/***************** read and write interface of register ******************************/

  /**
   * @fn writeReg
   * @brief write register function, design it as a pure virtual function, implement the function body through a derived class
   * @param reg  register address 8bits
   * @param pBuf to write data storage and buffer
   * @param size to write data length
   * @return None
   */
  virtual void writeReg(uint8_t reg, const void* pBuf, size_t size)=0;

  /**
   * @fn readReg
   * @brief read register function, design it as a pure virtual function, implement the function body through a derived class
   * @param reg  register address 8bits
   * @param pBuf to read data storage and buffer
   * @param size to read data length
   * @return return read length, returning 0 means read failure
   */
  virtual size_t readReg(uint8_t reg, void* pBuf, size_t size)=0;

public:
  /* the structure storing basic sensor information */
  sBasicInfoI2C_t basicInfoI2C;
  sBasicInfoRTU_t basicInfoRTU;

private:
  /* define private variables */
  uint8_t _deviceInterface;
  uint8_t _measureRangeMode;
};


/***************** initialization, read and write of I2C interface ******************************/

class DFRobot_URM13_I2C:public DFRobot_URM13
{
public:
  /**
   * @fn DFRobot_URM13_I2C
   * @brief constructor, configure sensor I2C communication address according to module DIP switch status
   * @param i2cAddr RotaryEncoder I2C communication address
   * @param pWire Wire object is defined in Wire.h, so just use &Wire and the methods in Wire can be pointed to and used
   * @param interfaceURM13 the current interface mode: eRtuInterface, eI2cInterface
   * @return None
   */
  DFRobot_URM13_I2C(uint8_t i2cAddr=URM13_DEFAULT_ADDR_I2C, TwoWire *pWire = &Wire, eInterfaceMode_t interfaceURM13=eI2cInterface);

  /**
   * @fn begin
   * @brief subclass initialization function
   * @return int type, means returning initialization status
   * @retval 0 NO_ERROR
   * @retval -1 ERR_DATA_BUS
   * @retval -2 ERR_IC_VERSION
   */
  virtual int begin(void);


protected:
  /**
   * @fn writeReg
   * @brief write register values through I2C bus
   * @param reg  register address 8bits
   * @param pBuf to write data storage and buffer
   * @param size to write data length
   * @return None
   */
  virtual void writeReg(uint8_t reg, const void* pBuf, size_t size);

  /**
   * @fn readReg
   * @brief read register values through I2C bus
   * @param reg  register address 8bits
   * @param pBuf to read data storage and buffer
   * @param size to read data length
   * @return return read length, returning 0 means read failure
   */
  virtual size_t readReg(uint8_t reg, void* pBuf, size_t size);

private:
  TwoWire *_pWire;   // pointer to I2C communication method
  uint8_t _deviceAddr;   // I2C communication device address
};


/***************** initialization, read and write of Modbus-RTU interface ******************************/

class DFRobot_URM13_RTU:public DFRobot_URM13
{
public:
  /**
   * @enum  eBaudrateMode_t
   * @brief  the module can set baud rate value
   */
  typedef enum
  {
    eBaudrate2400 = 0x0001,
    eBaudrate4800 = 0x0002,
    eBaudrate9600 = 0x0003,
    eBaudrate14400 = 0x0004,
    eBaudrate19200 = 0x0005,
    eBaudrate38400 = 0x0006,
    eBaudrate57600 = 0x0007,
    eBaudrate115200 = 0x0008,
  }eBaudrateMode_t;

  /**
   * @enum  eCheckBitMode_t
   * @brief the module can set check bit mode
   */
  typedef enum
  {
    eCheckBitNone = 0x0000<<8,
    eCheckBitEven = 0x0001<<8,
    eCheckBitOdd = 0x0002<<8,
  }eCheckBitMode_t;

  /**
   * @enum  eStopBitMode_t
   * @brief the module can set stop bit mode
   */
  typedef enum
  {
    eStopBit0P5 = 0x0000,
    eStopBit1 = 0x0001,
    eStopBit1P5 = 0x0002,
    eStopBit2 = 0x0003,
  }eStopBitMode_t;

  /**
   * @fn DFRobot_URM13_RTU
   * @brief constructor
   * @param _serial serial ports for communication, supporting hard and soft serial ports
   * @param RTU_addr RS485 communication device address
   * @param interfaceURM13 the current interface mode: eRtuInterface, eI2cInterface
   * @return None
   */
  DFRobot_URM13_RTU(Stream *_serial, uint16_t RTU_addr=URM13_DEFAULT_ADDR_RTU, eInterfaceMode_t interfaceURM13=eRtuInterface);

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

protected:
  /**
   * @fn writeReg
   * @brief register function for keep writing (when writing several register data)
   * @param reg  register address 16bits
   * @param pBuf to write data storage and buffer
   * @param size to write data length
   * @return None
   */
  virtual void writeReg(uint8_t reg, const void * pBuf, size_t size);

  /**
   * @fn readReg
   * @brief register function for keep reading
   * @param reg  register address 16bits
   * @param pBuf to read data storage and buffer
   * @param size to read data length
   * @return return read length, returning 0 means read failure
   */
  virtual size_t readReg(uint8_t reg, void * pBuf, size_t size);

private:
  DFRobot_RTU _DFRobot_RTU;   // object of RS485 communication method
  uint16_t _deviceAddr;   // device address of RTU communication
};

#endif
