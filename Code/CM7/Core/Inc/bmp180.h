/*
 * bmp180.h
 *
 *  Created on: Nov 21, 2023
 *      Author: scath
 */
#include "main.h"

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

// Register Addresses
#define BMP180_ADDR 0x77
#define BMP180_REG_CTRL 0xF4
#define BMP180_REG_RESULT 0xF6
#define BMP180_COMMAND_TEMP 0x2E
#define BMP180_COMMAND_PRESSURE0 0x34
#define BMP180_COMMAND_PRESSURE1 0x74
#define BMP180_COMMAND_PRESSURE2 0xB4
#define BMP180_COMMAND_PRESSURE3 0xF4

// EEPROM Registers
#define AC1_ADDR 0xAA
#define AC2_ADDR 0xAC
#define AC3_ADDR 0xAE
#define AC4_ADDR 0xB0
#define AC5_ADDR 0xB2
#define AC6_ADDR 0xB4
#define  B1_ADDR 0xB6
#define  B2_ADDR 0xB8
#define  MB_ADDR 0xBA
#define  MC_ADDR 0xBC
#define  MD_ADDR 0xBE

// Pressure Oversampling Timer Delays
#define OSS_0_TIME 5
#define OSS_1_TIME 8
#define OSS_2_TIME 14
#define OSS_3_TIME 26

// Conversion Factors
#define atmPress 101325
#define BMP_PRESS_FACTOR 19029495718


void bmp180_uTemp(I2C_HandleTypeDef *hi2c);

void bmp180_calc_temp(I2C_HandleTypeDef *hi2c);

/* Function: Read uncompensated pressure value
 * @params: None
 * @return: None
 * */

void bmp180_get_upressure(I2C_HandleTypeDef *hi2c, int oss);

void bmp180_calc_pressure(I2C_HandleTypeDef *hi2c, int oss);

/* Function: Calculate true temperature
 * @params: None
 * @return: None
 * */

void bmp180_altitude(I2C_HandleTypeDef *hi2c, int oss);

void read_calibration(I2C_HandleTypeDef *hi2c, char addr);

#endif /* INC_BMP180_H_ */
