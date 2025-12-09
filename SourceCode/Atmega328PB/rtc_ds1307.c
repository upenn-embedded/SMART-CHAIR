#include <stdbool.h>
#include "rtc_ds1307.h"

static uint8_t bcd2d(uint8_t b){ return (b>>4)*10 + (b & 0x0F); }
static uint8_t d2bcd(uint8_t d){ return ((d/10)<<4) | (d%10);  }

void DS1307_Init(void){
  DS1307_I2C_init();

  uint8_t sec;
  time_i2c_read_single(DS1307_I2C_ADDRESS, 0x00, &sec);
  if (sec & 0x80){ sec &= 0x7F; time_i2c_write_single(DS1307_I2C_ADDRESS, 0x00, &sec); }
}

void DS1307_GetTime(rtc_time_t *t){
  uint8_t buf[7];
  time_i2c_read_multi(DS1307_I2C_ADDRESS, 0x00, buf, 7);
  t->sec  = bcd2d(buf[0] & 0x7F);
  t->min  = bcd2d(buf[1]);
  t->hour = bcd2d(buf[2] & 0x3F);
  t->day  = bcd2d(buf[3]);
  t->date = bcd2d(buf[4]);
  t->mon  = bcd2d(buf[5]);
  t->year = bcd2d(buf[6]); 
}

void DS1307_SetTime(const rtc_time_t *t){
  uint8_t buf[7];
  buf[0]=d2bcd(t->sec);
  buf[1]=d2bcd(t->min);
  buf[2]=d2bcd(t->hour);
  buf[3]=d2bcd(t->day);
  buf[4]=d2bcd(t->date);
  buf[5]=d2bcd(t->mon);
  buf[6]=d2bcd(t->year);
  time_i2c_write_multi(DS1307_I2C_ADDRESS, 0x00, buf, 7);
}

rtc_time_t DS1307_GetValidTime(void)
{
    rtc_time_t t;
    rtc_time_t t1;
        while (1)
        {
            DS1307_GetTime(&t1);
            DS1307_GetTime(&t);

            if (t.sec > 59 || t.min > 59 || t.hour > 23) {
                // clearly garbage, retry
                continue;
            }
            bool ok = false;
            if (t.min == t1.min) {
                // same minute, OK if seconds didn't jump backwards or by a crazy amount
                if (t.sec == t1.sec || t.sec == (uint8_t)(t1.sec + 1)) {
                    ok = true;
                }
            } else if (t.min == (uint8_t)(t1.min + 1)) {
                // minute advanced by exactly 1
                if (t1.sec == 59 && t.sec <= 1) {
                    ok = true;
                }
            }

            if (ok) {
                return t;
            }
            // otherwise: loop and read again
        }
}
