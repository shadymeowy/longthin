#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

#define WIRE_INST Wire1

struct mpu6050 {
	uint16_t address;
	int16_t raw_data[6];
	float accel[3];
	float gyro[3];
	float bias_gyro[3];
};

int mpu6050_init(struct mpu6050 *mpu, uint16_t addr);
int mpu6050_read(struct mpu6050 *mpu);
int mpu6050_calibrate(struct mpu6050 *mpu, uint16_t samples);
int mpu6050_samplediv_set(struct mpu6050 *mpu, uint8_t div);
int mpu6050_samplediv_get(struct mpu6050 *mpu, uint8_t *div);
int mpu6050_config_set(struct mpu6050 *mpu, uint8_t dlfp);
int mpu6050_config_get(struct mpu6050 *mpu, uint8_t *dlfp);
int mpu6050_gyro_conf_set(struct mpu6050 *mpu, uint8_t conf);
int mpu6050_gyro_conf_get(struct mpu6050 *mpu, uint8_t *conf);
int mpu6050_accel_conf_set(struct mpu6050 *mpu, uint8_t conf);
int mpu6050_accel_conf_get(struct mpu6050 *mpu, uint8_t *conf);

#endif