/*!
 * @file  URM13WorkInModbus.ino
 * @brief  This demo shows how URM13 works in Modbus-RTU interface mode.
 * @n      can obtain and change the sensor basic information, configure parameters and get the current distance value and current temperature value
 * @n      Note: because the called DFRobot_RTU library does't support microbit, this demo does't support microbit either
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0.1
 * @date  2021-09-18
 * @url  https://github.com/DFRobot/DFRobot_URM13
 */
#include <DFRobot_URM13.h>
#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
#include <SoftwareSerial.h>
#endif

/*!
 * UART(Modbus-RTU) and I2C/TRIG mode switch
 * URM13sensor default setting is in UART mode. the sensor can switch between I2C and UART modes simply by short-circuiting different pins before power-on：

 * I2C/TRIG: Short-circuit TRIG and ECHO pins before the sensor is powered on. After the sensor is powered on, that the LED flashes twice indicates the sensor has switched to I2C mode.
 * UART(Modbus-RTU): Short-circuit TRIG and BUSY pins before the sensor is powered on. After the sensor is powered on, that the LED flashes once indicates the sensor has switched to UART(Modbus-RTU) mode.

 * After the mode switch succeeds, users can disconnect the corresponding pin short-circuiting, and the switched mode will be recorded by the sensor and take effect permanently.
 */

#define DEFAULT_DEVICE_ADDRESS 0x000D
/**
 * @brief DFRobot_URM13_RTU constructor
 * @param addr: modbus slave address（range1~247）or broadcast address（0x00），if it's set to a broadcast address, send a broadcast packet, and all slaves on bus will process it but not respond
 * @param s   : a serial port pointer to Stream
 */
#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
  SoftwareSerial mySerial(/*rx =*/4, /*tx =*/5);
  DFRobot_URM13_RTU sensor(/*s =*/&mySerial, /*addr =*/DEFAULT_DEVICE_ADDRESS);
#else
  DFRobot_URM13_RTU sensor(/*s =*/&Serial1, /*addr =*/DEFAULT_DEVICE_ADDRESS);
#endif


void setup()
{
  Serial.begin(115200);

#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
  mySerial.begin(19200);   // excessive baud rate of UNO soft serial port will makes communication unstable. 9600 is recommended.
#elif defined(ESP32)
  Serial1.begin(19200, SERIAL_8N1, /*rx =*/D3, /*tx =*/D2);
#else
  Serial1.begin(19200);
#endif

  // initialize the sensor
  while( NO_ERR != sensor.begin() ){
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("Begin ok!");

  /**
   * retrieve basic information from the sensor and buffer it into basicInfoRTU, the structure that stores information
   */
  sensor.refreshBasicInfo();

  /* module PID, default value is 0x03 the bit is used for product check[can detect the sensor type] */
  Serial.print("PID: 0x0");
  Serial.println(sensor.basicInfoRTU.PID, HEX);

  /* module VID, firmware revision number：0x10 represents V1.0 */
  Serial.print("VID: 0x");
  Serial.println(sensor.basicInfoRTU.VID, HEX);

  /* module Modbus-RTU slave address, default value is 0x0D, module device address(1~247) */
  Serial.print("mailing address: 0x0");
  Serial.println(sensor.basicInfoRTU.addr, HEX);

  /* module baud rate，default value is 0x0005:
   * 0x0001---2400  0x0002---4800  0x0003---9600  0x0004---14400  0x0005---19200
   * 0x0006---38400  0x0007---57600  0x0008---115200 */
  Serial.print("baudrate: 0x0");
  Serial.println(sensor.basicInfoRTU.baudrate, HEX);

  /* check bit and stop bit of the module，default value is 0x0001
   * check bit: 0 represents none; 1 represents even; 2 represents odd
   * stop bit: 0.5bit; 1bit; 1.5bit; 2bit */
  Serial.print("check bit: ");
  Serial.println(sensor.basicInfoRTU.checkbit);
  Serial.print("stop bit: ");
  Serial.println((sensor.basicInfoRTU.stopbit + 1.0) / 2.0, 1);

  /**
   * set the module communication address, power off to save the settings, and restart for the settings to take effect
   * addr device address to be set, Modbus-RTU address range(1~247 is 0x01~0xF7)
   */
  sensor.setADDR(0x0D);

  /**
   * UART interface mode, set module baud rate, the setting takes effect after power fail and restart, the default is 19200
   * addr the baud rate to be set：
   * eBaudrate2400---2400, eBaudrate4800---4800, eBaudrate9600---9600, 
   * eBaudrate14400---14400, eBaudrate19200---19200, eBaudrate38400---38400, 
   * eBaudrate57600---57600, eBaudrate115200---115200
   */
  sensor.setBaudrateMode(sensor.eBaudrate19200);

  /**
   * UART interface mode, set check bit and stop bit of the module
   * mode the mode to be set, the following patterns constitute mode：
   * check bit：
   *       eCheckBitNone
   *       eCheckBitEven
   *       eCheckBitOdd
   * stop bit：
   *       eStopBit0P5
   *       eStopBit1
   *       eStopBit1P5
   *       eStopBit2
   */
  sensor.setCheckbitStopbit(sensor.eCheckBitNone | sensor.eStopBit1);

  /**
   * set measure mode
   * mode measure mode to be set, the following patterns constitute mode:
   *   eInternalTemp: use internal temperature compensation function, eExternalTemp: use external temperature compensation function (users need to write external temperature)
   *   eTempCompModeEn: enable temperature compensation function, eTempCompModeDis: disable temperature compensation function
   *   eAutoMeasureModeEn: automatic ranging, eAutoMeasureModeDis: passive ranging
   *   eMeasureRangeModeLong: large range measurement(40 - 900cm), eMeasureRangeModeShort: small range measurement(15-150cm)
   */
  sensor.setMeasureMode(sensor.eInternalTemp | \
                        sensor.eTempCompModeEn | \
                        sensor.eAutoMeasureModeDis | \
                        sensor.eMeasureRangeModeLong);

  /**
   * note: the api makes external temperature compensation function meaningful only when setting measure mode
   * write ambient temperature data for external temperature compensation, the setting is invalid when out of range
   * temp written ambient temperature data, unit is ℃, resolution is 0.1℃, signed number, range:-10℃～＋70℃
   */
  sensor.setExternalTempretureC(30.0);

  /**
   * ranging sensitivity setting, 0x00-0x0A:sensitivity level 0-10
   * mode to set the sensor ranging sensitivity in large range (40-900cm), the smaller the value, and the higher the sensitivity, power off to save the settings, and it takes effect at once
   */
  sensor.setMeasureSensitivity(0x00);

  Serial.println();
  delay(1000);
}

void loop()
{
  /**
   * the function to trigger measuring in passive measurement mode
   * in passive measurement mode, the function is called once, the ranging command is sent once, and the module measures the distance once and saves the measured value into the distance register
   */
  sensor.passiveMeasurementTRIG();

  /**
   * get noise level of power supply, the smaller the noise level, the more accurate the distance value obtained by the sensor
   * the parameter indicates the influence of power supply and environment on the sensor. 0x00-0x0A matches noise level of 0-10.
   */
  Serial.print("Current ambient noise level: 0x0");
  Serial.println(sensor.getNoiseLevel(), HEX);

  /**
   * read the current internal temperature
   * the current internal temperature value, unit is ℃, resolution is 0.1℃, signed number
   */
  Serial.print("The onboard temperature: ");
  Serial.print(sensor.getInternalTempretureC());
  Serial.println(" C");

  /**
   * read the current distance value, the value of zero indicates it's not measured within the range
   * note: when the object is not in the sensor ranging range, the read measured data will be meaningless
   * the current distance value, unit is cm, large range(40 - 900cm)small range(15-150cm)
   */
  Serial.print("Current distance measurement: ");
  Serial.print(sensor.getDistanceCm());
  Serial.println(" cm");

  Serial.println();
  delay(1000);
}
