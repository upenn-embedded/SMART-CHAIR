#include <avr/io.h>
#include <util/delay.h>
#include "vl53_port.h"
#include "twi.h"
#include "MotorControl.h"
#include "UART.h"

#define REG_SYSRANGE_START          0x00
#define REG_SYSTEM_INTERRUPT_CLEAR  0x0B
#define REG_RESULT_INTERRUPT_STATUS 0x13
#define REG_RESULT_RANGE_STATUS     0x14
#define REG_RESULT_RANGE_MM_HI      0x1E
#define REG_RESULT_RANGE_MM_LO      0x1F
#define REG_IDENTIFICATION_MODEL_ID 0xC0 


void tof_gpio_init(void)
{

    DDRD  |= (1<<XSHUT_L) | (1<<XSHUT_M) | (1<<XSHUT_R);
    PORTD &= ~((1<<XSHUT_L) | (1<<XSHUT_M) | (1<<XSHUT_R));
}

void tof_all_shutdown(void)
{
    DDRD  |= (1<<XSHUT_L) | (1<<XSHUT_M) | (1<<XSHUT_R);
    PORTD &= ~((1<<XSHUT_L) | (1<<XSHUT_M) | (1<<XSHUT_R));
}

void tof_release_one(uint8_t xshut_bit)
{
    DDRD  |= xshut_bit;  
    PORTD |= xshut_bit;  
    _delay_ms(5); 
}


static uint8_t vl53_write_u8(uint8_t addr7, uint8_t reg, uint8_t value)
{
    if (twi_start((addr7 << 1) | 0)) 
        return 1;

    if (twi_write(reg)) {    
        twi_stop();
        return 2;
    }
    if (twi_write(value)) {  
        twi_stop();
        return 3;
    }

    twi_stop();
    return 0;
}

static uint8_t vl53_read_u8(uint8_t addr7, uint8_t reg, uint8_t *value)
{
    if (twi_start((addr7 << 1) | 0))
        return 1;

    if (twi_write(reg)) {
        twi_stop();
        return 2;
    }

    if (twi_start((addr7 << 1) | 1)) {  
        twi_stop();
        return 3;
    }

    *value = twi_read_nack();
    twi_stop();
    return 0;
}

static uint8_t vl53_read_u16(uint8_t addr7, uint8_t reg_hi, uint16_t *value)
{
    uint8_t hi, lo;

    if (twi_start((addr7 << 1) | 0))
        return 1;
    if (twi_write(reg_hi)) {
        twi_stop();
        return 2;
    }

    if (twi_start((addr7 << 1) | 1)) {
        twi_stop();
        return 3;
    }

    hi = twi_read_ack();
    lo = twi_read_nack();
    twi_stop();

    *value = ((uint16_t)hi << 8) | lo;
    return 0;
}

uint8_t vl53_change_address(uint8_t new7)
{
    if (twi_start((VL53_ADDR_DEF << 1) | 0)) return 1;
    if (twi_write(0x8A))            { twi_stop(); return 2; } 
    if (twi_write(new7))            { twi_stop(); return 3; } 
    twi_stop();
    return 0;
}

static void ToF_ReinitBoth(void)
{
    UART0_SendString("Reinitializing both ToF sensors...\r\n");

    tof_all_shutdown();
    _delay_ms(10);

    tof_release_one(1 << XSHUT_M);
    _delay_ms(10);

    if (vl53_change_address(VL53_ADDR_M) != 0) {
        UART0_SendString("ToF M addr change FAIL in reinit\r\n");
    } else if (vl53_init_and_start(VL53_ADDR_M) != 0) {
        UART0_SendString("ToF M init FAIL in reinit\r\n");
    } else {
        UART0_SendString("ToF M reinit OK\r\n");
    }

    tof_release_one(1 << XSHUT_R);
    _delay_ms(10);

    if (vl53_change_address(VL53_ADDR_R) != 0) {
        UART0_SendString("ToF R addr change FAIL in reinit\r\n");
    } else if (vl53_init_and_start(VL53_ADDR_R) != 0) {
        UART0_SendString("ToF R init FAIL in reinit\r\n");
    } else {
        UART0_SendString("ToF R reinit OK\r\n");
    }
}


uint8_t vl53_init_and_start(uint8_t addr7)
{
    uint8_t id;

    if (vl53_read_u8(addr7, REG_IDENTIFICATION_MODEL_ID, &id) != 0) {
        return 1; 
    }

    vl53_write_u8(addr7, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    return 0;
}


uint16_t vl53_read_mm(uint8_t addr7)
{
    uint16_t dist;

    if (vl53_write_u8(addr7, REG_SYSRANGE_START, 0x01) != 0) {
        return 0;
    }

    _delay_ms(40);

    if (vl53_read_u16(addr7, REG_RESULT_RANGE_MM_HI, &dist) != 0) {
        return 0;
    }

    vl53_write_u8(addr7, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    return dist;
}

void vl53_bring_to_known_state(uint16_t* dM, uint16_t* dR)
{
    *dM = vl53_read_mm(VL53_ADDR_M);
    *dR = vl53_read_mm(VL53_ADDR_R);
    // if both sensors look dead, try reinit
    static uint8_t both_zero_count = 0;

 //   if ((*dM == 0 || *dM == 0xFFFF || *dM >= 8190) &&
 //       (*dR == 0 || *dR == 0xFFFF || *dR >= 8190))
    if(*dM == 0 || *dR == 0)
    {
        if (both_zero_count < 50) both_zero_count++;  // about 10 seconds at 200 ms
    } 
    else 
    {
        both_zero_count = 0;
    }

    if (both_zero_count >= 3)  // ~5 seconds of both dead
    {
        Motor_SetSpeedPercent(0);   // be safe, stop motor while we reset sensors
        ToF_ReinitBoth();
        both_zero_count = 0;
    }

}
