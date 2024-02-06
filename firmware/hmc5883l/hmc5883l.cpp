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
	hmc->raw_data[0] = WIRE_INST.read() << 8 | WIRE_INST.read();
	hmc->raw_data[2] = WIRE_INST.read() << 8 | WIRE_INST.read();
	hmc->raw_data[1] = WIRE_INST.read() << 8 | WIRE_INST.read();

	//for (int i = 0; i < 3; i++) {
	//	hmc->mag[i] = hmc->raw_data[i] / 1090.0;
	//}
	float mag_offset[3] = { 0 };
	mag_offset[0] = hmc->raw_data[0] - -0.01409842;
	mag_offset[1] = hmc->raw_data[1] - -0.01651878;
	mag_offset[2] = hmc->raw_data[2] - -0.15467174;
	hmc->mag[0] = mag_offset[0] * 2248.0437349117365 + mag_offset[1] * -9.820466585653632 + mag_offset[2] * 83.5181286985458;
	hmc->mag[1] = mag_offset[0] * -9.820466585653618 + mag_offset[1] * 2323.575656987126 + mag_offset[2] * -10.094511360510472;
	hmc->mag[2] = mag_offset[0] * 83.51812869854605 + mag_offset[1] * -10.0945113605105 + mag_offset[2] * 2721.9289350440895;
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