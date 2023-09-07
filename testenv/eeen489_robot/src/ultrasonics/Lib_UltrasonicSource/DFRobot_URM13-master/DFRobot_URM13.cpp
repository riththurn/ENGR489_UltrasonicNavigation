/*!
 * @file  DFRobot_URM13.cpp
 * @brief  DFRobot_URM13.cpp Initialize the I2C,
 * @n      obtain URM13 basic information, measure distance and internal temperature, select the sensor communication interface and set the sensor parameters
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2021-09-15
 * @url  https://github.com/DFRobot/DFRobot_URM13
 */
#include "DFRobot_URM13.h"

DFRobot_URM13::DFRobot_URM13(eInterfaceMode_t interfaceURM13)
{
  _deviceInterface = interfaceURM13;
  _measureRangeMode = eMeasureRangeModeLong;
}


int DFRobot_URM13::begin(void)
{
  if (_deviceInterface == eRtuInterface){
    uint8_t idBuf[2];
    if(0 == readReg(URM13_PID_REG_RTU, idBuf, sizeof(idBuf))){   // Judge whether the data bus is successful
      DBG("ERR_DATA_BUS");
      return ERR_DATA_BUS;
    }
    DBG("real sensor id ="); DBG( (uint16_t)idBuf[0] << 8 | (uint16_t)idBuf[1], HEX );
    if( 0x0003 != ( (uint16_t)idBuf[0] << 8 | (uint16_t)idBuf[1] ) ){   // Judge whether the chip version matches
      DBG("ERR_IC_VERSION");
      return ERR_IC_VERSION;
    }

  }else if (_deviceInterface == eI2cInterface){
    uint8_t id;
    if(0 == readReg(URM13_PID_REG_I2C, &id, sizeof(id))){   // Judge whether the data bus is successful
      DBG("ERR_DATA_BUS");
      return ERR_DATA_BUS;
    }
    DBG("real sensor id="); DBG(id, HEX );
    if( 0x02 != id ){   // Judge whether the chip version matches
      DBG("ERR_IC_VERSION");
      return ERR_IC_VERSION;
    }
  }

  delay(200);
  DBG("begin ok!");

  return NO_ERR;
}

void DFRobot_URM13::refreshBasicInfo(void)
{
  if (_deviceInterface == eRtuInterface){
    uint8_t buf[10] = {0};
    readReg(URM13_PID_REG_RTU, buf, 10);   // rtu basic information length is 10 bytes
    basicInfoRTU.PID = (buf[0] << 8) | buf[1];
    basicInfoRTU.VID = (buf[2] << 8) | buf[3];
    basicInfoRTU.addr = (buf[4] << 8) | buf[5];
    basicInfoRTU.baudrate = (buf[6] << 8) | buf[7];
    basicInfoRTU.checkbit = buf[8];
    basicInfoRTU.stopbit = buf[9];
  }else if (_deviceInterface == eI2cInterface){
    readReg(URM13_ADDR_REG_I2C, &basicInfoI2C, 3);   // I2C basic information length is 3 bytes
  }
}

void DFRobot_URM13::setADDR(uint8_t addr)
{
  if (_deviceInterface == eRtuInterface){
    if((0x01 <= addr) && (0xF7 >= addr)){
      uint8_t buf[2] = {0};
      buf[1] = addr;
      writeReg(URM13_ADDR_REG_RTU, buf, sizeof(buf));
    }

  }else if (_deviceInterface == eI2cInterface){
    if((0x01 <= addr) && (0x7F >= addr)){
      writeReg(URM13_ADDR_REG_I2C, &addr, sizeof(addr));
    }
  }
}

uint16_t DFRobot_URM13::getDistanceCm(void)
{
  uint8_t buf[2] = {0};
  uint16_t distanceCm;

  if (_deviceInterface == eRtuInterface){
    readReg(URM13_DISTANCE_REG_RTU, buf, sizeof(buf));
  }else if (_deviceInterface == eI2cInterface){
    readReg(URM13_DISTANCE_MSB_REG_I2C, buf, sizeof(buf));
  }

  distanceCm = (buf[0] << 8) | buf[1];
  // if(eMeasureRangeModeLong == _measureRangeMode){
  //   if((40 > distanceCm) || (900 < distanceCm)){
  //     distanceCm = 0;
  //   }

  // }else if(eMeasureRangeModeLong == eMeasureRangeModeShort){
  //   if((15 > distanceCm) || (150 < distanceCm)){
  //     distanceCm = 0;
  //   }
  // }

  return distanceCm;
}

float DFRobot_URM13::getInternalTempretureC(void)
{
  uint8_t buf[2] = {0};
  if (_deviceInterface == eRtuInterface){
    readReg(URM13_INTERNAL_TEMP_REG_RTU, buf, sizeof(buf));
  }else if (_deviceInterface == eI2cInterface){
    readReg(URM13_INTERNAL_TEMP_MSB_REG_I2C, buf, sizeof(buf));
  }

  return (float)(int16_t)((buf[0] << 8) | buf[1]) / 10;
}

void DFRobot_URM13::setExternalTempretureC(float temp)
{
  if((-10 < temp) && (70 > temp)){
    int16_t temperature = (int16_t)(temp * 10);   // written ambient temperature data, unit is ℃, resolution is 0.1℃, signed number
    uint8_t buf[2] = {0};
    buf[0] = (uint8_t)((temperature & 0xFF00) >> 8);
    buf[1] = (uint8_t)(temperature & 0x00FF);

    if (_deviceInterface == eRtuInterface){
      writeReg(URM13_EXTERNAL_TEMP_REG_RTU, buf, sizeof(buf));
    }else if (_deviceInterface == eI2cInterface){
      writeReg(URM13_EXTERNAL_TEMP_MSB_REG_I2C, buf, sizeof(buf));
    }
  }
}

void DFRobot_URM13::setMeasureMode(uint8_t mode)
{
  if (_deviceInterface == eRtuInterface){
    uint8_t buf[2] = {0};
    // readReg(URM13_CONFIG_REG_RTU, buf, sizeof(buf));
    // DBG(buf[1]);
    buf[1] = mode;
    writeReg(URM13_CONFIG_REG_RTU, buf, sizeof(buf));

  }else if (_deviceInterface == eI2cInterface){
    // uint8_t data = 0;
    // readReg(URM13_CONFIG_REG_I2C, &data, sizeof(data));
    // DBG(data);
    writeReg(URM13_CONFIG_REG_I2C, &mode, sizeof(mode));
  }

  if(mode & eMeasureRangeModeShort){
    _measureRangeMode = eMeasureRangeModeShort;
  }else{
    _measureRangeMode = eMeasureRangeModeLong;
  }
}

void DFRobot_URM13::passiveMeasurementTRIG(void)
{
  if (_deviceInterface == eRtuInterface){
    uint8_t buf[2] = {0};
    readReg(URM13_CONFIG_REG_RTU, buf, sizeof(buf));

    // in passive mode, write 1 to the bit and the sensor will complete a distance measurement. The distance value can be read from the distance register after the measurement is completed (about 100ms), in automatic ranging mode, the bit is reserved, and bits after the position 1 will automatically be reset
    buf[1] |= (1 << 3);
    writeReg(URM13_CONFIG_REG_RTU, buf, sizeof(buf));
    delay(300);

  }else if (_deviceInterface == eI2cInterface){
    uint8_t mode = 0x01;
    writeReg(URM13_COMMAND_REG_I2C, &mode, sizeof(mode));
  }
}

uint8_t DFRobot_URM13::getNoiseLevel(void)
{
  uint8_t mode = 0;
  if (_deviceInterface == eRtuInterface){
    uint8_t buf[2] = {0};
    readReg(URM13_NOISE_REG_RTU, buf, sizeof(buf));
    mode = buf[1];

  }else if (_deviceInterface == eI2cInterface){
    readReg(URM13_NOISE_REG_I2C, &mode, sizeof(mode));
  }

  return mode;
}

void DFRobot_URM13::setMeasureSensitivity(uint8_t mode)
{
  if (_deviceInterface == eRtuInterface){
    uint8_t buf[2] = {0};
    buf[1] = mode;
    writeReg(URM13_SENSITIVITY_REG_RTU, buf, sizeof(buf));

  }else if (_deviceInterface == eI2cInterface){
    writeReg(URM13_SENSITIVITY_REG_I2C, &mode, sizeof(mode));
  }
}


/************ Initialization of I2C interfaces reading and writing ***********/

DFRobot_URM13_I2C::DFRobot_URM13_I2C(uint8_t i2cAddr, TwoWire *pWire, eInterfaceMode_t interfaceURM13)
  :DFRobot_URM13(interfaceURM13)
{
  _deviceAddr = i2cAddr;
  _pWire = pWire;
}

int DFRobot_URM13_I2C::begin(void)
{
  _pWire->begin();   // Wire.h（I2C）library function initialize wire library
  delay(50);

  return DFRobot_URM13::begin();   // Use the initialization function of the parent class
}

void DFRobot_URM13_I2C::writeReg(uint8_t reg, const void* pBuf, size_t size)
{
  if(pBuf == NULL){
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;

  _pWire->beginTransmission(_deviceAddr);
  _pWire->write(reg);

  for(size_t i = 0; i < size; i++){
    _pWire->write(_pBuf[i]);
  }

  _pWire->endTransmission();
}

size_t DFRobot_URM13_I2C::readReg(uint8_t reg, void* pBuf, size_t size)
{
  size_t count = 0;
  if(NULL == pBuf){
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t*)pBuf;

  _pWire->beginTransmission(_deviceAddr);
  _pWire -> write(reg);
  if(0 != _pWire->endTransmission()){
    // Used Wire.endTransmission() to end a slave transmission started by beginTransmission() and arranged by write().
    DBG("endTransmission ERROR!!");
  }else{
    // Master device requests size bytes from slave device, which can be accepted by master device with read() or available()
    _pWire->requestFrom( _deviceAddr, (uint8_t)size );

    while (_pWire->available()){
      _pBuf[count++] = _pWire->read();   // Use read() to receive and put into buf
      // DBG(_pBuf[count-1], HEX);
    }
  }

  return count;
}


/************ Initialization of modbus-RTU interfaces reading and writing ***********/

DFRobot_URM13_RTU::DFRobot_URM13_RTU(Stream *_serial, uint16_t RTU_addr, eInterfaceMode_t interfaceURM13)
  : DFRobot_URM13(interfaceURM13), _DFRobot_RTU(_serial)
{
  _deviceAddr = RTU_addr;
  _DFRobot_RTU.setTimeoutTimeMs(500);
}

void DFRobot_URM13_RTU::setBaudrateMode(eBaudrateMode_t mode)
{
  uint8_t buf[2] = {0};
  buf[0] = (uint8_t)((mode & 0xFF00) >> 8);
  buf[1] = (uint8_t)(mode & 0x00FF);

  writeReg(URM13_BAUDRATE_REG_RTU, buf, sizeof(buf));
  delay(200);
}

void DFRobot_URM13_RTU::setCheckbitStopbit(uint16_t mode)
{
  uint8_t buf[2] = {0};
  buf[0] = (uint8_t)((mode & 0xFF00) >> 8);
  buf[1] = (uint8_t)(mode & 0x00FF);
  writeReg(URM13_CHECKBIT_STOPBIT_REG_RTU, buf, sizeof(buf));
}

void DFRobot_URM13_RTU::writeReg(uint8_t reg, const void * pBuf, size_t size)
{
  if(NULL == pBuf)
  {
    DBG("pBuf ERROR!! : null pointer");
  }

  uint8_t ret = _DFRobot_RTU.writeHoldingRegister(_deviceAddr, (uint16_t)reg, (uint8_t *)pBuf, (uint16_t)size);
  if (ret != 0){
    DBG(ret);
  }
}

size_t DFRobot_URM13_RTU::readReg(uint8_t reg, void * pBuf, size_t size)
{
  if(NULL == pBuf)
  {
    DBG("pBuf ERROR!! : null pointer");
  }

  uint8_t ret = _DFRobot_RTU.readHoldingRegister(_deviceAddr, (uint16_t)reg, pBuf, (uint16_t)size);
  if (ret != 0){
    DBG(ret);
    return 0;
  }
  return size;
}
