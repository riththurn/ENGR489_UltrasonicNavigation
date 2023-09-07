# -*- coding: utf-8 -*
'''!
  @file  URM13_work_in_TRIG.py
  @brief This demo shows how URM13 works in TRIG interface mode, note: due to python2 timekeeping, this demo can only work well in python3
  @n      can obtain and change the sensor basic information, configure parameters and get the current distance value and current temperature value
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license  The MIT License (MIT)
  @author  [qsjhyy](yihuan.huang@dfrobot.com)
  @version  V1.0.1
  @date  2021-09-24
  @url  https://github.com/DFRobot/DFRobot_URM13
'''
from __future__ import print_function
import sys
import RPi.GPIO as GPIO
sys.path.append('../')
from python.raspberrypi.example.DFRobot_URM13 import *


'''
  # UART(Modbus-RTU) and I2C/TRIG mode switch
  # URM13 sensor default setting is in UART mode. the sensor can switch between I2C and UART modes simply by short-circuiting different pins before power-on：

  # I2C/TRIG: Short-circuit TRIG and ECHO pins before the sensor is powered on. After the sensor is powered on, that the LED flashes twice indicates the sensor has switched to I2C mode.
  # UART(Modbus-RTU): Short-circuit TRIG and BUSY pins before the sensor is powered on. After the sensor is powered on, that the LED flashes once indicates the sensor has switched to UART(Modbus-RTU) mode.

  # After the mode switch succeeds, users can disconnect the corresponding pin short-circuiting, and the switched mode will be recorded by the sensor and take effect permanently.

  # the sensor parameters can be configured by instantiating an object with I2C communication
'''
# sensor = DFRobot_URM13_I2C(i2c_addr = 0x12, bus = 1)


'''
  # trig_pin   in I2C interface mode, to measure distance and trigger pin as external port, rising edge trigger
  # echo_pin   when the sensor ranging function is triggered, the pin outputs a high pulse-width, which represents the ultrasonic transmission time (unit：us)
'''
global flag, echo_pin, echo_pin_high_start_ticks, echo_pin_high_end_ticks, speed_of_sound
flag = 0
echo_pin_high_start_ticks = 0   # start time of high level corresponding to the sensor measured data pulse signal
echo_pin_high_end_ticks = 0   # end time of high level corresponding to the sensor measured data pulse signal
speed_of_sound = 0   # 音速
trig_pin = 20   # trig_pin
echo_pin = 21   # echo_pin


def int_callback(channel):
  global flag, echo_pin, echo_pin_high_start_ticks, echo_pin_high_end_ticks   # declare global variables
  if 1 == GPIO.input(echo_pin) and 0 == flag:
    echo_pin_high_start_ticks = time.time()
    flag = 1
  if 0 == GPIO.input(echo_pin) and 1 == flag:
    echo_pin_high_end_ticks = time.time()
    flag = 2


def delay_microsecond(microsecond):   # microsecond delay function
    start, end = 0, 0   # declare variables
    start = time.time()       # record start time
    microsecond = (microsecond - 3) / 1000000   # convert the unit of t to second, and -3 is time compensation
    while end-start < microsecond:  # keep cycling until the time difference is more than or equal to the set value
        end = time.time()   # record end time



def setup():
  global echo_pin, trig_pin, speed_of_sound   # declare global variables
  GPIO.setwarnings(False)   # turn off warnings like pin setting
  GPIO.setmode(GPIO.BCM)   # set pin encoding mode to BCM
  GPIO.setup(trig_pin, GPIO.OUT, initial=0)   # set measure trigger pin to output mode, initialize output low level
  GPIO.setup(echo_pin, GPIO.IN)   # set measure data pin to input mode
  GPIO.add_event_detect(echo_pin, GPIO.BOTH, callback=int_callback)   # Use GPIO port to monitor sensor interrupt

  environment_temperature = 30.0   # the current ambient temperature
  speed_of_sound = (331.5 + 0.6 * environment_temperature ) * 100  # the speed of sound calculated from a given current ambient temperature

  # while (sensor.begin() == False):
  #   print ('Please check that the device is properly connected')
  #   time.sleep(3)
  # print("sensor begin successfully!!!\n")

  '''
    # set measure mode
    # mode measure mode to be set, the following patterns constitute mode:
    #   E_INTERNAL_TEMP: use internal temperature compensation function, E_EXTERNAL_TEMP: use external temperature compensation function (users need to write external temperature)
    #   E_TEMP_COMP_MODE_EN: enable temperature compensation function, E_TEMP_COMP_MODE_DIS: disable temperature compensation function
    #   E_AUTO_MEASURE_MODE_EN: automatic ranging, E_AUTO_MEASURE_MODE_DIS: passive ranging
    #   E_MEASURE_RANGE_MODE_LONG: large range measurement(40 - 900cm), E_MEASURE_RANGE_MODE_SHORT: small range measurement(15-150cm)
  '''
  # sensor.set_measure_mode(sensor.E_INTERNAL_TEMP | 
  #                         sensor.E_TEMP_COMP_MODE_EN | 
  #                         sensor.E_AUTO_MEASURE_MODE_DIS | 
  #                         sensor.E_MEASURE_RANGE_MODE_LONG)

  '''
    # write ambient temperature data for external temperature compensation, the setting is invalid when out of range
    # temp written ambient temperature data, unit is ℃, resolution is 0.1℃, signed number, range:-10℃～＋70℃
  '''
  # sensor.set_external_tempreture_C(30.0)

  '''
    # ranging sensitivity setting, 0x00-0x0A:sensitivity level0-10
    # measure_sensitivity to set the sensor ranging sensitivity in large range (40-900cm), the smaller the value, and the higher the sensitivity, power off to save the settings, and it takes effect at once
  '''
  # sensor.set_measure_sensitivity(0x00)

  print()
  time.sleep(1.5)


def loop():
  global flag, echo_pin, trig_pin, speed_of_sound, echo_pin_high_start_ticks, echo_pin_high_end_ticks   # declare global variables
  GPIO.output(trig_pin, GPIO.HIGH)   # Set the trig_pin High
  delay_microsecond(50)   # Delay of 50 microseconds
  GPIO.output(trig_pin, GPIO.LOW)   # Set the trig_pin Low

  for i in range(1000):
    if flag == 2:
        break

  if flag == 2:
    # Measure echo high level time, the output high level time represents the ultrasonic flight time (unit: us)
    measuring_time = echo_pin_high_end_ticks - echo_pin_high_start_ticks
    flag = 0
    # print(measuring_time)
    # print(speed_of_sound)

    '''
      # calculate the current distance value
      # note：when the object is not in the sensor ranging range, the read measured data will be meaningless
      # the current distance value, unit is cm, large range(40 - 900cm)small range(15-150cm)
      # The distance can be calculated according to the flight time of ultrasonic wave,
      # and the ultrasonic sound speed can be compensated according to the actual ambient temperature
    '''
    measuring_distance = speed_of_sound * measuring_time / 2.0
    print("Current distance measurement: %d cm" %measuring_distance)
    print()

  time.sleep(1)


if __name__ == "__main__":
  setup()
  while True:
    loop()
