# DFRobot_URM13
* [English Version](./README.md)

URM13是一款开放式单探头超声波测距传感器, 支持TRIG脉冲触发测距（兼容SR04）、UART和I2C, 传感器可以在三种接口模式间无缝切换。该传感器尺寸紧凑、并且兼容如Arduino、树莓派等各种3.3V或5V主控板, 非常方便用户集成和应用, UART模式使用标准Modbus-RTU协议并集成了收发控制输出, 可简单通过外接RS485收发器扩展RS485接口。该传感器在保持同类传感器尺寸及重量优势的同时还具有非常出色的测距灵敏度, 使得他对于一些低声波反射率的探测目标同样具备超越同类传感器的探测性能。URM13传感器会在每次测距时自动检测环境及电源噪声并以此来动态调节测量参数, 确保它能在各种复杂应用场景之下依旧能够稳定工作。
为了满足不同的用户需求, URM13内置两段测距量程: 
1、小量程15-150cm: 可以实现高达50HZ的高频率探测, 适用于室内机器人避障等场景。
2、大量程40-900cm: 具有卓越的测量灵敏度, 测量频率为常规10HZ, 适用于空旷场景或需要高灵敏度、高量程距离探测的场景。
实际使用时, 可通过重复触发2段量程的测距, 以实现整个量程的检测。

![产品实物图](./resources/images/URM13.jpg)


## 产品链接 (https://www.dfrobot.com.cn/goods-2965.html)
    SKU: SEN0352


## 目录

* [概述](#概述)
* [库安装](#库安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [创作者](#创作者)


## 概述

* 传感器数据可以由UART(modbus-rtu)、I2C和TRIG三种接口输出, 满足多种接口环境。<br>
* 可以获取传感器基本信息、当前距离测量值和当前温度测量值。<br>
* 可以配置传感器通信地址, 测量参数等。<br>
* 为了满足不同的用户需求, URM13内置两段测距量程: 小量程15-150cm; 大量程40-900cm。<br>


## 库安装

要使用这个库, 首先下载库文件( https://github.com/DFRobot/DFRobot_URM13 )和依赖文件( https://github.com/DFRobot/DFRobot_RTU ), 将其粘贴到\Arduino\libraries目录中, 然后打开示例文件夹并在文件夹中运行演示。


## 方法

```C++

  /**
   * @fn begin
   * @brief 初始化函数
   * @return int类型, 表示返回初始化的状态
   * @retval 0 NO_ERROR
   * @retval -1 ERR_DATA_BUS
   * @retval -2 ERR_IC_VERSION
   */
  virtual int begin(void);

  /**
   * @fn refreshBasicInfo
   * @brief 重新从传感器获取其基本信息, 并缓存到存储信息的结构体里面:
   * @n       I2C接口模式: addr_I2C, PID_I2C, VID_I2C
   * @n       RTU接口模式: PID_RTU, VID_RTU, addr_RTU, baudrate_RTU, checkbit_RTU, stopbit_RTU
   * @return None
   */
  void refreshBasicInfo(void);

  /**
   * @fn setADDR
   * @brief 设置模块的通信地址, 断电保存, 重启后生效
   * @param addr 要设置的设备地址, I2C地址范围(1~127即0x01~0x7F), RTU地址范围(1~247即0x0001-0x00F7)
   * @return None
   */
  void setADDR(uint8_t addr);

  /**
   * @fn getDistanceCm
   * @brief 读取当前距离测量值, 测量值为零表示未测量到量程范围内的值
   * @return 当前距离测量值, 单位cm, 大量程测距范围(40 - 900cm)小量程测距范围(15-150cm)
   */
  uint16_t getDistanceCm(void);

  /**
   * @fn getInternalTempretureC
   * @brief 读取当前板载温度
   * @return 当前板载温度值, 单位℃, 分辨率0.1℃,有符号数
   */
  float getInternalTempretureC(void);

  /**
   * @fn setExternalTempretureC
   * @brief 写入环境温度数据用于外部温度补偿, 超出范围, 设置无效
   * @param temp 写入的环境温度数据, 单位℃, 分辨率0.1℃,有符号数, 范围:-10℃～＋70℃
   * @return None
   */
  void setExternalTempretureC(float temp);

  /**
   * @fn setMeasureMode
   * @brief 设置测量相关模式
   * @param mode 需要设置的测量相关模式, 下列模式经过或运算后得到mode:
   * @n       eInternalTemp: 使用板载温度补偿功能, eExternalTemp: 使用外部温度补偿功能(需用户写入外部温度)
   * @n       eTempCompModeEn: 开启温度补偿功能, eTempCompModeDis: 关闭温度补偿功能
   * @n       eAutoMeasureModeEn: 自动测距, eAutoMeasureModeDis: 被动测距
   * @n       eMeasureRangeModeLong: 大量程测距(40 - 900cm), eMeasureRangeModeShort: 小量程测距(15-150cm)
   * @return None
   */
  void setMeasureMode(uint8_t mode);

  /**
   * @fn passiveMeasurementTRIG
   * @brief 被动测量模式下的触发测量函数
   * @n 在被动测量模式下, 调用一次此函数, 发送一次测距命令, 模块测量一次距离并将测量的距离值存入距离寄存器
   * @return None
   */
  void passiveMeasurementTRIG(void);

  /**
   * @fn getNoiseLevel
   * @brief 获取电源噪声等级, 噪声等级越小, 传感器得到的距离值将更精准。
   * @return 该参数能够反映供电电源以及环境对传感器的影响程度。 0x00-0x0A对应噪声等级0-10。
   */
  uint8_t getNoiseLevel(void);

  /**
   * @fn setMeasureSensitivity
   * @brief 测距灵敏度设置, 0x00-0x0A:灵敏度等级0-10
   * @param mode 用于设置传感器大量程段(40-900cm)的测距灵敏度, 该值越小, 灵敏度越高, 断电保存, 立即生效
   * @return None
   */
  void setMeasureSensitivity(uint8_t mode);

  /**
   * @fn setBaudrateMode
   * @brief UART接口模式下, 设置模块的波特率, 断电保存, 重启后生效
   * @param mode 要设置的波特率:
   * @n     eBaudrate2400---2400, eBaudrate4800---4800, eBaudrate9600---9600, 
   * @n     eBaudrate14400---14400, eBaudrate19200---19200, eBaudrate38400---38400, 
   * @n     eBaudrate57600---57600, eBaudrate115200---115200
   * @return None
   */
  void setBaudrateMode(eBaudrateMode_t mode);

  /**
   * @fn setCheckbitStopbit
   * @brief UART接口模式下, 设置模块的校验位和停止位
   * @param mode 要设置的模式, 下列模式经过或运算后得到mode:
   * @n     校验位:
   * @n           eCheckBitNone
   * @n           eCheckBitEven
   * @n           eCheckBitOdd
   * @n     停止位:
   * @n           eStopBit0P5
   * @n           eStopBit1
   * @n           eStopBit1P5
   * @n           eStopBit2
   * @return None
   */
  void setCheckbitStopbit(uint16_t mode);

```


## 兼容性

主控               |     软串口     |     硬串口     |
------------------ | :------------: | :------------: |
Arduino Uno        |       √        |       X        |
Mega2560           |       √        |       √        |
Leonardo           |       √        |       √        |
ESP32              |       X        |       √        |
ESP8266            |       √        |       X        |
micro:bit          |       X        |       X        |
FireBeetle M0      |       X        |       √        |
raspberry          |       X        |       √        |

主控               |  正常运行    |   运行失败    |   未测试    | 备注
------------------ | :----------: | :----------: | :---------: | :---:
Arduino Uno        |      √       |              |             |
Arduino MEGA2560   |      √       |              |             |
Arduino Leonardo   |      √       |              |             |
FireBeetle-ESP8266 |      √       |              |             |
FireBeetle-ESP32   |      √       |              |             |
FireBeetle-M0      |      √       |              |             |
Micro:bit          |      √       |              |             |


## 历史

- 2021/09/22 - 1.0.1 版本


## 创作者

Written by qsjhyy(yihuan.huang@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))

