/*!
 * @file  URM13WorkInI2C.ino
 * @brief  This demo shows how URM13 works in I2C interface mode.
 * @n      can obtain and change the sensor basic information, configure parameters and get the current distance value and current temperature value
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0.1
 * @date  2021-09-18
 * @url  https://github.com/DFRobot/DFRobot_URM13
 */
#include <DFRobot_URM13.h>

/*!
 * UART(Modbus-RTU) and I2C/TRIG mode switch
 * URM13 sensor default setting is in UART mode. the sensor can switch between I2C and UART modes simply by short-circuiting different pins before power-on：

 * I2C/TRIG: Short-circuit TRIG and ECHO pins before the sensor is powered on. After the sensor is powered on, that the LED flashes twice indicates the sensor has switched to I2C mode.
 * UART(Modbus-RTU): Short-circuit TRIG and BUSY pins before the sensor is powered on. After the sensor is powered on, that the LED flashes once indicates the sensor has switched to UART(Modbus-RTU) mode.

 * After the mode switch succeeds, users can disconnect the corresponding pin short-circuiting, and the switched mode will be recorded by the sensor and take effect permanently.
 */

/*
 * instantiate an object to drive the sensor;
 * need to set the sensor I2C address and the I2C bus used by it, keep the default value if it’s not changed.
 */
DFRobot_URM13_I2C sensor(/*i2cAddr = */0x12, /*i2cBus = */&Wire);

// open the macro if you want to use data measuring completion signal from busyPin as a measure, note: use pin numbers based on the main controller settings using
// #define BUSYPIN_SIGNAL
#ifdef BUSYPIN_SIGNAL
  int16_t    busyPin = 4;   // "Data Measuring Ready Signal" pin, you can set the pins can be used as required (depending on the controller differences)
  #define    isSensorBusy()           (digitalRead(busyPin))
#endif

void setup()
{
  Serial.begin(115200);

#ifdef BUSYPIN_SIGNAL
  pinMode(busyPin, INPUT);   // initialize "Data Measuring Ready Signal" pin to input
#endif

  // initialize the sensor
  while( NO_ERR != sensor.begin() ){
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("Begin ok!");

  /**
   * retrieve basic information from the sensor and buffer it into basicInfoI2C, the structure that stores information
   */
  sensor.refreshBasicInfo();

  /* I2C slave address of the module, default value is 0x12, module device address(1~127) */
  Serial.print("mailing address: 0x");
  Serial.println(sensor.basicInfoI2C.addr, HEX);

  /* module PID, default value is 0x02 the bit is used for product check[can detect the sensor type] */
  Serial.print("PID: 0x0");
  Serial.println(sensor.basicInfoI2C.PID, HEX);

  /* module VID, firmware revision number：0x10 represents V1.0 */
  Serial.print("VID: 0x");
  Serial.println(sensor.basicInfoI2C.VID, HEX);

  /**
   * set the module communication address, power off to save the settings, and restart for the settings to take effect
   * addr device address to be set, I2C address range(1~127 is 0x01~0x7F)
   */
  sensor.setADDR(0x12);

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

#ifdef BUSYPIN_SIGNAL
  // You can replace the delay with these two lines of code
  while(isSensorBusy()== HIGH);   // Wait for the sensor to start ranging
  while(isSensorBusy()== LOW);   // Wait for sensor ranging to complete
#else
  delay(100);   // delay 100ms
#endif

  /**
   * get noise level of power supply, the smaller the noise level, the more accurate the distance value obtained by the sensor
   * the parameter indicates the influence of power supply and environment on the sensor. 0x00-0x0A matches noise level of 0-10。
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
   * read the current distance value, the value of zero indicates it’s not measured within the range
   * note: when the object is not in the sensor ranging range, the read measured data will be meaningless
   * the current distance value, unit is cm, large range(40 - 900cm)small range(15-150cm)
   */
  Serial.print("Current distance measurement: ");
  Serial.print(sensor.getDistanceCm());
  Serial.println(" cm");

  Serial.println();
  delay(1000);
}
