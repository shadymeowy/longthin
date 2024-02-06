#ifndef HMC5883L_H
#define HMC5883L_H

#include <stdint.h>

#define WIRE_INST Wire1

struct hmc5883l {
    uint16_t address;
    int16_t raw_data[3];
    float mag[3];
};

int hmc5883l_init(struct hmc5883l *hmc, uint16_t addr);
int hmc5883l_read(struct hmc5883l *hmc);
int hmc5883l_mode_set(struct hmc5883l *hmc, uint8_t mode);
int hmc5883l_mode_get(struct hmc5883l *hmc, uint8_t *mode);
int hmc5883l_config1_set(struct hmc5883l *hmc, uint8_t ms, uint8_t do_,
			 uint8_t ma);
int hmc5883l_config1_get(struct hmc5883l *hmc, uint8_t *ms, uint8_t *do_,
			 uint8_t *ma);
int hmc5883l_config2_set(struct hmc5883l *hmc, uint8_t cr);
int hmc5883l_config2_get(struct hmc5883l *hmc, uint8_t *cr);

#endif