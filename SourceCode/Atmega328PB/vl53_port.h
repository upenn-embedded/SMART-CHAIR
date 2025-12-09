#ifndef VL53_PORT_H
#define VL53_PORT_H

#include <stdint.h>
#include <avr/io.h>

#define VL53_ADDR_DEF 0x29

#define VL53_ADDR_L   0x2A
#define VL53_ADDR_M   0x2B
#define VL53_ADDR_R   0x2C

#define XSHUT_L PD5
#define XSHUT_M PD6
#define XSHUT_R PD7

void tof_gpio_init(void);
void tof_all_shutdown(void);
void tof_release_one(uint8_t xshut_bit);
uint8_t vl53_change_address(uint8_t new7);
uint8_t  vl53_init_and_start(uint8_t addr7);
uint16_t vl53_read_mm(uint8_t addr7);
void vl53_bring_to_known_state(uint16_t* dM, uint16_t* dR);
#endif
