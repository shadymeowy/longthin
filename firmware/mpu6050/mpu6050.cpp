#include "mpu6050.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>


int mpu6050_init(struct mpu6050 *mpu, uint16_t addr)
{
	WIRE_INST.begin();
	WIRE_INST.beginTransmission(addr);
	WIRE_INST.write(0x6B); // PWR_MGMT_1 register
	WIRE_INST.write(0); // set to zero (wakes up the MPU-6050)
	WIRE_INST.endTransmission(true);
	mpu->address = addr;

	mpu6050_samplediv_set(mpu, 1);
	mpu6050_config_set(mpu, 0);
	mpu6050_gyro_conf_set(mpu, 0);
	mpu6050_accel_conf_set(mpu, 0);
	return 0;
}

int mpu6050_read(struct mpu6050 *mpu)
{
	WIRE_INST.beginTransmission(mpu->address);
	WIRE_INST.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
	WIRE_INST.endTransmission(false);
	// request a total of 14 registers
	WIRE_INST.requestFrom(mpu->address, 14, true);
	// 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
	mpu->raw_data[0] = WIRE_INST.read() << 8 | WIRE_INST.read();
	// 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	mpu->raw_data[1] = WIRE_INST.read() << 8 | WIRE_INST.read();
	// 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	mpu->raw_data[2] = WIRE_INST.read() << 8 | WIRE_INST.read();
	// 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	WIRE_INST.read();
	WIRE_INST.read();
	// 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	mpu->raw_data[3] = WIRE_INST.read() << 8 | WIRE_INST.read();
	// 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	mpu->raw_data[4] = WIRE_INST.read() << 8 | WIRE_INST.read();
	// 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
	mpu->raw_data[5] = WIRE_INST.read() << 8 | WIRE_INST.read();
	for (int i = 0; i < 3; i++) {
		mpu->accel[i] = mpu->raw_data[i] / 16384.0;
		mpu->accel[i] = mpu->accel[i] * 9.81;
		mpu->gyro[i] = mpu->raw_data[i + 3] / 131.0;
		mpu->gyro[i] = mpu->gyro[i] * (M_PI / 180.0) - mpu->bias_gyro[i];
	}
	return 0;
}

int mpu6050_calibrate(struct mpu6050 *mpu, uint16_t samples)
{
	float gyro[3] = { 0 };
	for (int i = 0; i < samples; i++) {
		mpu6050_read(mpu);
		for (int j = 0; j < 3; j++) {
			gyro[j] += mpu->gyro[j];
		}
		delay(1);
	}
	for (int i = 0; i < 3; i++) {
		mpu->bias_gyro[i] = gyro[i] / samples;
	}
	return 0;
}

int mpu6050_samplediv_set(struct mpu6050 *mpu, uint8_t div)
{
	WIRE_INST.beginTransmission(mpu->address);
	WIRE_INST.write(0x19); // SMPLRT_DIV register
	WIRE_INST.write(div);
	WIRE_INST.endTransmission(true);
	return 0;
}

int mpu6050_samplediv_get(struct mpu6050 *mpu, uint8_t *div)
{
	WIRE_INST.beginTransmission(mpu->address);
	WIRE_INST.write(0x19); // SMPLRT_DIV register
	WIRE_INST.endTransmission(false);
	WIRE_INST.requestFrom(mpu->address, 1, true);
	*div = WIRE_INST.read();
	return 0;
}

int mpu6050_config_set(struct mpu6050 *mpu, uint8_t dlfp)
{
	WIRE_INST.beginTransmission(mpu->address);
	WIRE_INST.write(0x1A); // CONFIG register
	WIRE_INST.write(dlfp & 0x07);
	WIRE_INST.endTransmission(true);
	return 0;
}

int mpu6050_config_get(struct mpu6050 *mpu, uint8_t *dlfp)
{
	WIRE_INST.beginTransmission(mpu->address);
	WIRE_INST.write(0x1A); // CONFIG register
	WIRE_INST.endTransmission(false);
	WIRE_INST.requestFrom(mpu->address, 1, true);
	*dlfp = WIRE_INST.read();
	return 0;
}

int mpu6050_gyro_conf_set(struct mpu6050 *mpu, uint8_t conf)
{
	WIRE_INST.beginTransmission(mpu->address);
	WIRE_INST.write(0x1B); // GYRO_CONFIG register
	WIRE_INST.write(conf & 0x18);
	WIRE_INST.endTransmission(true);
	return 0;
}

int mpu6050_gyro_conf_get(struct mpu6050 *mpu, uint8_t *conf)
{
	WIRE_INST.beginTransmission(mpu->address);
	WIRE_INST.write(0x1B); // GYRO_CONFIG register
	WIRE_INST.endTransmission(false);
	WIRE_INST.requestFrom(mpu->address, 1, true);
	*conf = WIRE_INST.read();
	return 0;
}

int mpu6050_accel_conf_set(struct mpu6050 *mpu, uint8_t conf)
{
	WIRE_INST.beginTransmission(mpu->address);
	WIRE_INST.write(0x1C); // ACCEL_CONFIG register
	WIRE_INST.write(conf & 0x18);
	WIRE_INST.endTransmission(true);
	return 0;
}

int mpu6050_accel_conf_get(struct mpu6050 *mpu, uint8_t *conf)
{
	WIRE_INST.beginTransmission(mpu->address);
	WIRE_INST.write(0x1C); // ACCEL_CONFIG register
	WIRE_INST.endTransmission(false);
	WIRE_INST.requestFrom(mpu->address, 1, true);
	*conf = WIRE_INST.read();
	return 0;
}