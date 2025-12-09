
#ifndef RTC_DS1307_H
#define RTC_DS1307_H

#include <stdint.h>

#define DS1307_I2C_ADDRESS  0x68

typedef struct {
  uint8_t sec,min,hour,day,date,mon,year; 
} rtc_time_t;

void DS1307_I2C_init(void);
void time_i2c_write_single(uint8_t device_address, uint8_t register_address, uint8_t *data_byte);
void time_i2c_write_multi (uint8_t device_address, uint8_t start_register_address, uint8_t *data_array, uint8_t data_length);
void time_i2c_read_single (uint8_t device_address, uint8_t register_address, uint8_t *data_byte);
void time_i2c_read_multi  (uint8_t device_address, uint8_t start_register_address, uint8_t *data_array, uint8_t data_length);
rtc_time_t DS1307_GetValidTime(void);

void DS1307_Init(void);
void DS1307_GetTime(rtc_time_t *t);
void DS1307_SetTime(const rtc_time_t *t);

#endif
