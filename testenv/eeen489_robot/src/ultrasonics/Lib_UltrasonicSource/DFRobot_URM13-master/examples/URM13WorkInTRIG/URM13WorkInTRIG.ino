/*!
 * @file  URM13WorkInTRIG.ino
 * @brief  This demo shows how URM13 works in TRIG interface mode.
 * @n      can obtain the sensor current distance value and change its parameters through I2C
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
 * URM13sensor default setting is in UART mode. the sensor can switch between I2C and UART modes simply by short-circuiting different pins before power-on：

 * I2C/TRIG: Short-circuit TRIG and ECHO pins before the sensor is powered on. After the sensor is powered on, that the LED flashes twice indicates the sensor has switched to I2C mode.
 * UART(Modbus-RTU): Short-circuit TRIG and BUSY pins before the sensor is powered on. After the sensor is powered on, that the LED flashes once indicates the sensor has switched to UART(Modbus-RTU) mode.
 * After the mode switch succeeds, users can disconnect the corresponding pin short-circuiting, and the switched mode will be recorded by the sensor and take effect permanently.
 */

/* if you want to use I2C to change sensor parameters and mode, open the macro and configure the following api */
// #define I2C_CONFIG
#ifdef I2C_CONFIG
  /* instantiate an object with I2C communication to drive the sensor */
  DFRobot_URM13_I2C sensor(/*i2cAddr = */0x12, /*i2cBus = */&Wire);
#endif

/*
 * trigPin   in I2C interface mode, to measure distance and trigger pin as external port, rising edge trigger
 * echoPin   in I2C interface mode, when the sensor ranging function is triggered, the pin outputs a high pulse-width, which represents the ultrasonic transmission time (unit：us)
 */
#if defined(ESP32) || defined(ESP8266)
  uint8_t trigPin = D2;
  uint8_t echoPin = D3;
#elif defined(ARDUINO_SAM_ZERO)
  uint8_t trigPin= 6;
  uint8_t echoPin= 7;
#else
  uint8_t trigPin= 8;
  uint8_t echoPin= 9;
#endif

// The ultrasonic velocity (cm/us) compensated by temperature
#define   VELOCITY_TEMP(temp)   ( ( 331.5 + 0.6 * (float)( temp ) ) * 100 / 1000000.0 )

void setup()
{
  Serial.begin(115200);

  pinMode(trigPin,OUTPUT);   // initialize trigPin and two IO interfaces of echoPin
  digitalWrite(trigPin,LOW);
  pinMode(echoPin,INPUT);

  Serial.println("TRIG pin begin ok!");

#ifdef I2C_CONFIG
  // initialize the sensor
  while( NO_ERR != sensor.begin() ){
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("I2C begin ok!");

  /**
   * set measure mode
   * mode measure mode to be set, the following patterns constitute mode mode:
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
#endif

  delay(1000);
}

void loop()
{
  uint16_t  pulseWidthUs, distanceCm;
  digitalWrite(trigPin,HIGH);   // Set the tirgPin High
  delayMicroseconds(50);         // Delay of 50 microseconds
  digitalWrite(trigPin,LOW);    // Set the tirgPin Low

  /**
   * Measure echo high level time, the output high level time represents the ultrasonic flight time (unit: us)
   * The distance can be calculated according to the flight time of ultrasonic wave, 
   * and the ultrasonic sound speed can be compensated according to the actual ambient temperature
   */
  pulseWidthUs = pulseIn(echoPin,HIGH);
  delayMicroseconds(50);   // Delay of 50 microseconds
  distanceCm = (uint16_t)(pulseWidthUs * VELOCITY_TEMP(30.0) / 2.0);

  Serial.print("This distance measurement value: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  Serial.println();
  delay(1000);
}
