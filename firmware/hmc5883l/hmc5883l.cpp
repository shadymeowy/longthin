#include "hmc5883l.h"

#include <Arduino.h>
#include <Wire.h>

int hmc5883l_init(struct hmc5883l *hmc, uint16_t addr)
{
	hmc->address = addr;
	hmc5883l_mode_set(hmc, 0x00);
	hmc5883l_config1_set(hmc, 0x00, 0x04, 0x00);
	hmc5883l_config2_set(hmc, 0x01);
	return 0;
}

int hmc5883l_read(struct hmc5883l *hmc)
{
	WIRE_INST.beginTransmission(hmc->address);
	WIRE_INST.write(0x03);
	WIRE_INST.endTransmission(false);
	WIRE_INST.requestFrom(hmc->address, 6, true);
	int16_t raw_data[3];
	raw_data[0] = WIRE_INST.read() << 8 | WIRE_INST.read();
	raw_data[2] = WIRE_INST.read() << 8 | WIRE_INST.read();
	raw_data[1] = WIRE_INST.read() << 8 | WIRE_INST.read();
	hmc->mag_raw[0] = raw_data[0]; // / 1090.0;
	hmc->mag_raw[1] = raw_data[1]; // / 1090.0;
	hmc->mag_raw[2] = raw_data[2]; // / 1090.0;

	float m[3] = { 0 };
	m[0] = hmc->mag_raw[0] - hmc->bias[0];
	m[1] = hmc->mag_raw[1] - hmc->bias[1];
	m[2] = hmc->mag_raw[2] - hmc->bias[2];
	hmc->mag[0] = m[0] * hmc->mtx[0] + m[1] * hmc->mtx[1] + m[2] * hmc->mtx[2];
	hmc->mag[1] = m[0] * hmc->mtx[3] + m[1] * hmc->mtx[4] + m[2] * hmc->mtx[5];
	hmc->mag[2] = m[0] * hmc->mtx[6] + m[1] * hmc->mtx[7] + m[2] * hmc->mtx[8];
	return 0;
}

int hmc5883l_mode_set(struct hmc5883l *hmc, uint8_t mode)
{
	WIRE_INST.beginTransmission(hmc->address);
	WIRE_INST.write(0x02);
	WIRE_INST.write(mode);
	WIRE_INST.endTransmission(true);
	return 0;
}

int hmc5883l_mode_get(struct hmc5883l *hmc, uint8_t *mode)
{
	WIRE_INST.beginTransmission(hmc->address);
	WIRE_INST.write(0x02);
	WIRE_INST.endTransmission(false);
	WIRE_INST.requestFrom(hmc->address, 1, true);
	*mode = WIRE_INST.read();
	return 0;
}

int hmc5883l_config1_set(struct hmc5883l *hmc, uint8_t ms, uint8_t do_, uint8_t ma)
{
	WIRE_INST.beginTransmission(hmc->address);
	WIRE_INST.write(0x00);
	WIRE_INST.write((ms & 0x3) | (do_ & 0x7) << 2 | (ma & 0x7) << 5);
	WIRE_INST.endTransmission(true);
	return 0;
}

int hmc5883l_config1_get(struct hmc5883l *hmc, uint8_t *ms, uint8_t *do_, uint8_t *ma)
{
	WIRE_INST.beginTransmission(hmc->address);
	WIRE_INST.write(0x00);
	WIRE_INST.endTransmission(false);
	WIRE_INST.requestFrom(hmc->address, 1, true);
	uint8_t reg = WIRE_INST.read();
	*ms = reg & 0x3;
	*do_ = (reg >> 2) & 0x7;
	*ma = (reg >> 5) & 0x7;
	return 0;
}

int hmc5883l_config2_set(struct hmc5883l *hmc, uint8_t gain)
{
	WIRE_INST.beginTransmission(hmc->address);
	WIRE_INST.write(0x01);
	WIRE_INST.write(gain << 5);
	WIRE_INST.endTransmission(true);
	return 0;
}

int hmc5883l_config2_get(struct hmc5883l *hmc, uint8_t *cr)
{
	WIRE_INST.beginTransmission(hmc->address);
	WIRE_INST.write(0x01);
	WIRE_INST.endTransmission(false);
	WIRE_INST.requestFrom(hmc->address, 1, true);
	*cr = WIRE_INST.read() >> 5;
	return 0;
}