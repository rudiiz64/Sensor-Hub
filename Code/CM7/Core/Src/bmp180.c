/*
 * bmp180.c
 *
 *  Created on: Nov 21, 2023
 *      Author: scath
 */

#include "bmp180.h"

/* Data structure to retain calibration parameters from BMP180 */
struct cal_params {
	short AC1, AC2, AC3, B1, B2, MB, MC, MD;
	unsigned short AC4, AC5, AC6;
	long x1, x2, x3, B3, B4, B5, B6, B7, t, p, UT, UP;
	float atmP;
} params;

void bmp180_uTemp(I2C_HandleTypeDef *hi2c){
	uint8_t dataToWrite = BMP180_COMMAND_TEMP;
	uint8_t tempRaw[2] = {0};
	HAL_I2C_Mem_Write(hi2c, BMP180_ADDR, BMP180_REG_CTRL, 1, &dataToWrite, 1, 1000);
	HAL_Delay(5);
	HAL_I2C_Mem_Read(hi2c, BMP180_ADDR, BMP180_REG_RESULT, 1, tempRaw, 2, 1000);
	params.UT = ((tempRaw[0] << 8) + tempRaw[1]);
}

void bmp180_calc_temp(I2C_HandleTypeDef *hi2c){
	bmp180_uTemp(hi2c);
	params.x1 = (params.UT - params.AC6) * (params.AC5/pow(2, 15));
	params.x2 = (params.MC * pow(2, 11)) / (params.x1 + params.MD);
	params.B5 = params.x1 + params.x2;
	params.t = (params.B5 + 8) / pow(2, 4);
}

void bmp180_get_upressure(I2C_HandleTypeDef *hi2c, int oss){
	uint8_t dataToWrite = 0x34 + (oss << 6);
	uint8_t pressRaw[3] = {0};
	HAL_I2C_Mem_Write(hi2c, BMP180_ADDR, BMP180_REG_CTRL, 1, &dataToWrite, 1, 1000);
	switch (oss) {
	case (0):
			HAL_Delay(OSS_0_TIME);
			break;

	case (1):
			HAL_Delay(OSS_1_TIME);
			break;

	case (2):
			HAL_Delay(OSS_2_TIME);
			break;

	case (3):
			HAL_Delay(OSS_3_TIME);
			break;
	}
	HAL_I2C_Mem_Read(hi2c, BMP180_ADDR, BMP180_REG_RESULT, 1, pressRaw, 3, 1000);
	params.UP = ((pressRaw[0] << 16) + (pressRaw[1] << 8) + pressRaw[2]) >> (8 - oss);
}

void bmp180_calc_pressure(I2C_HandleTypeDef *hi2c, int oss){
	bmp180_get_upressure(hi2c, oss);
	params.B6 = params.B5 - 4000;
	params.x1 = (params.B2 * (params.B6 * (params.B6/pow(2, 12)))) / pow(2, 11);
	params.x2 = params.AC2 * (params.B6 / pow(2, 11));
	params.x3 = params.x1 + params.x2;
	params.B3 = ((((params.AC1 * 4) + params.x3) << oss) + 2) / 4;
	params.x1 = params.AC3 * (params.B6 / pow(2, 13));
	params.x2 = (params.B1);
	params.x3 = ((params.x1 + params.x2) + 2) / pow(2, 2);
	params.B4 = params.AC4 * (unsigned long)(params.x3 + 32768) / pow(2, 15);
	params.B7 = ((unsigned long) params.UP - params.B3) * (50000 >> oss);
	if (params.B7 < 0x80000000){
		params.p = (params.B7 * 2) / params.B4;
	}
	else {
		params.p = (params.B7 / params.B4) * 2;
	}
	params.x1 = (params.p / pow(2, 8)) * (params.p / pow(2, 8));
	params.x1 = (params.x1 * 3038) / pow(2, 16);
	params.x2 = (-7357 * params.p) / pow(2, 16);
	params.p = params. p + (params.x1 + params.x2 + 3791) / pow(2, 4);
}

void bmp180_altitude(I2C_HandleTypeDef *hi2c, int oss){
	bmp180_calc_pressure(hi2c, oss);
	params.atmP = 44330 * (1 - pow(((float) params.p / (float) atmPress), BMP_PRESS_FACTOR));
}

void read_calibration(I2C_HandleTypeDef *hi2c, char addr){
	uint8_t data[22] = {0};
	uint16_t mem_addr = addr;
	HAL_I2C_Mem_Read(hi2c, BMP180_ADDR, mem_addr, 1, data, 22, HAL_MAX_DELAY);

	params.AC1 = ((data[0]  << 8) | data[1]);
	params.AC2 = ((data[2]  << 8) | data[3]);
	params.AC3 = ((data[4]  << 8) | data[5]);
	params.AC4 = ((data[6]  << 8) | data[7]);
	params.AC5 = ((data[8]  << 8) | data[9]);
	params.AC6 = ((data[10] << 8) | data[11]);
	params.B1  = ((data[12] << 8) | data[13]);
	params.B2  = ((data[14] << 8) | data[15]);
	params.MB  = ((data[16] << 8) | data[17]);
	params.MC  = ((data[18] << 8) | data[19]);
	params.MD  = ((data[20] << 8) | data[21]);
}
