/*
 * I2CDev.h

 *
 *  Created on: 5 Apr 2018
 *      Author: Adam
 */
#ifndef _I2C_DEV_H_
#define _I2C_DEV_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define I2CTIMEOUT 0x100

static int8_t readBit(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout);
static int8_t readBits(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout);
static int8_t readByte(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout);
static int8_t readWord(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout);
static int8_t readBytes(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout);
//static int8_t readWords(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout);
static int8_t read32Bits(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint32_t *data, uint16_t timeout);

static bool writeBit(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
static bool writeBits(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
static bool writeByte(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t data);
static bool writeWord(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint16_t data);
static bool writeBytes(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
static bool write32Bits(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint32_t data);
//static bool writeWords(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

#endif //_I2C_DEV_H_

int8_t readBit(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout) {
    uint8_t b;
    uint8_t count = readByte(handler, devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

int8_t readBits(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    yyy   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count,b;
    if ((count = readByte(handler, devAddr, regAddr, &b, timeout)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

int8_t readByte(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout) {
    return readBytes(handler, devAddr, regAddr, 1, data, timeout);
}

int8_t readBytes(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout){
	int8_t count = length;
	HAL_I2C_Mem_Read(handler, devAddr<<1, (uint16_t)regAddr, 1, data, (uint16_t)length, (uint32_t)timeout);
	return count;
}

int8_t readWord(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout) {
	int8_t count = 2;
	uint8_t temp[4];
	readBytes(handler, devAddr, regAddr, 2, (uint8_t*)temp, timeout);
	*data = (((uint16_t)temp[0])<<8) | (temp[1]);
	return count;
}

int8_t read32Bits(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint32_t *data, uint16_t timeout){
	int8_t count = 4;
	uint8_t temp[8];
	readBytes(handler,devAddr,regAddr,4,(uint8_t*)temp,timeout);
	*data =  ((uint32_t)temp[0])<<24;
	*data |= ((uint32_t)temp[1])<<16;
	*data |= ((uint32_t)temp[2])<<8;
	*data |= ((uint32_t)temp[3]);
	return count;
}

bool writeBit(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(handler, devAddr, regAddr, &b,100);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(handler, devAddr, regAddr, b);
}

bool writeBits(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    yyy   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(handler, devAddr, regAddr, &b,100) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(handler, devAddr, regAddr, b);
    } else {
        return false;
    }
}

bool writeByte(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return writeBytes(handler, devAddr, regAddr, 1, &data);
}

bool writeBytes(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data){
	uint8_t status = 0;
	HAL_I2C_Mem_Write(handler, devAddr<<1, (uint16_t)regAddr,1,data, (uint16_t)length, (uint32_t)100);
	return status == 0;
}
bool writeWord(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint16_t data){
	uint8_t towrite[2];
	towrite[0]=(uint8_t)data>>8 & 0xFF; //MSB
	towrite[1]=(uint8_t)data & 0xFF; //LSB
	writeBytes(handler, devAddr, regAddr, 2,(uint8_t*)towrite);
	return true;
}
bool write32Bits(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint32_t data){
	uint8_t towrite[4];
	towrite[0]=(uint8_t)data>>24 & 0xFF; //MSB
	towrite[1]=(uint8_t)data>>16 & 0xFF;
	towrite[2]=(uint8_t)data>>8 & 0xFF;
	towrite[3]=(uint8_t)data & 0xFF;
	writeBytes(handler, devAddr, regAddr, 4,(uint8_t*)towrite);
	return true;
}
/*bool writeWords(I2C_HandleTypeDef *handler, uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data) {
	uint8_t status = 0;
	uint8_t towrite[sizeof(*data)]
	HAL_I2C_Mem_Write(handler, devAddr<<1, (uint16_t)regAddr,1,towrite,1);
	return status == 0;
}*/