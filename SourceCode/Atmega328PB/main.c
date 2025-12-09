#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "UART.h"
#include "twi.h"
#include "rtc_ds1307.h"
#include "vl53_port.h"
#include "MotorControl.h" 
#include "pressureSensor.h"
#include "Timer.h"

#define DIST_MIN_MM       30   
#define DIST_MAX_MM       900 
#define DIFF_THR_MM        80

// Sample period is ~200 ms -> 10 samples ? 2 seconds
#define BAD_COUNT_THRESHOLD  10

static void print_u8(const char *k, uint8_t v)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "%s=%u ", k, v);
    UART0_SendString(buf);
}

static uint16_t absdiff_u16(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static bool is_valid_distance(uint16_t d)
{
    if (d == 0 || d == 0xFFFF || d >= 8190) return false;  // sensor invalid
    if (d < DIST_MIN_MM)  return false;
    if (d > DIST_MAX_MM)  return false;
    return true;
}

static bool is_posture_ok(uint16_t dM, uint16_t dR)
{
    if (!is_valid_distance(dM)) return false;
    if (!is_valid_distance(dR)) return false;

    uint16_t diff = absdiff_u16(dM, dR);
    if (diff > DIFF_THR_MM) return false;

    return true;
}

int main(void)
{
    UART0_Init();
    UART1_Init();          // for ESP32 (USART1)
    UART0_SendString("\r\n SYSTEM STARTS \r\n");
    PressureSensor_Init();
    MotorPWM_Init();
    Motor_SetSpeedPercent(0);  // ensure motor off
    // ---------- I2C ----------
    twi_init(TW_FREQ_100K, 0);   // 100 kHz on TWI
    // ---------- RTC ----------
    DS1307_Init();
    
    rtc_time_t newTime;

    // Set initial time once
    newTime.hour = 12;   // 24-hour format
    newTime.min  = 0;
    newTime.sec  = 0;
    newTime.day  = 1;    // day of month (1?31)
    newTime.mon  = 1;    // 1=Jan, 2=Feb, ...
    newTime.year = 25;   // last two digits (2025 -> 25)

    DS1307_SetTime(&newTime);
    UART0_SendString("RTC init OK\r\n");

    tof_gpio_init();        // configure XSHUT pins as outputs
    tof_all_shutdown();     // pull all XSHUT low -> all sensors in reset
    _delay_ms(10);

    // Sensor M: bring up first, move from 0x29 -> 0x2A
    tof_release_one(1 << XSHUT_M);    // only M comes out of reset
    _delay_ms(10);                  

    if (vl53_change_address(VL53_ADDR_M) != 0)
    {
        UART0_SendString("ToF M address change FAILED\r\n");
        while (1) { }
    }

    if (vl53_init_and_start(VL53_ADDR_M) != 0)
    {
        UART0_SendString("ToF M init FAILED\r\n");
        while (1) { }
    }
    UART0_SendString("ToF M ready at 0x2A\r\n");

    //Sensor R: bring up second, move from 0x29 -> 0x2B
    tof_release_one(1 << XSHUT_R);
    _delay_ms(10);

    if (vl53_change_address(VL53_ADDR_R) != 0)
    {
        UART0_SendString("ToF R address change FAILED\r\n");
        while (1) { }
    }

    if (vl53_init_and_start(VL53_ADDR_R) != 0)
    {
        UART0_SendString("ToF R init FAILED\r\n");
        while (1) { }
    }
    UART0_SendString("ToF R ready at 0x2B\r\n");

    UART0_SendString("Both ToF sensors init OK, starting measurements...\r\n");

    char buf[128];
    uint8_t bad_count = 0;
    bool motor_on = false;
    static bool isRTCValueInvalid = false;
    static rtc_time_t t; // Keep the last valid time in memory
  //  Timer1_Init_200ms();
    sei();
    while (1)
    {
        bool isUserPresent = PressureSensor_IsUserPresent();
        rtc_time_t raw_t = DS1307_GetValidTime();
        if (raw_t.hour == 0 && raw_t.min == 0 && raw_t.sec == 0)
        {
            UART0_SendString("[I2C Glitch Ignored]\r\n");
        }
        else
        {
            // Read was good, update
            t = raw_t;
        }
        do
        {
            if(!isUserPresent)
            {
                _delay_ms(300);
                UART0_SendString("User is not sitting...\r\n"); 
                if(motor_on)
                {
                    motor_on = false;
                    Motor_SetSpeedPercent(0);   // turn off
                }
                // stop music
                UART1_SendChar('b');
                // mark RTC value as invalid
                isRTCValueInvalid = true;
            }
            isUserPresent = PressureSensor_IsUserPresent();
        }while(!isUserPresent);
        
        if(isRTCValueInvalid)
        {
            UART0_SendString("Clock reset\r\n"); 
            // reset the clock
            rtc_time_t newTime = t; 
            newTime.min = 0; 
            newTime.sec = 0;
            rtc_time_t raw_t1;
            do
            {
                _delay_ms(50);
                DS1307_SetTime(&newTime);
                _delay_ms(50);
                raw_t1 = DS1307_GetValidTime();
            }while(raw_t1.sec != 0);
            isRTCValueInvalid = false;
            continue;
        }
        
        if(t.min >= 1 && isUserPresent && !isRTCValueInvalid)
        {
            UART0_SendString("Starting Music for sitting too long\r\n"); 
            // start music 
            UART1_SendChar('a');
            do
            {
                UART0_SendString("Please get up..\r\n");
                isUserPresent = PressureSensor_IsUserPresent();
            }while(isUserPresent);
            UART0_SendString("User got up..\r\n"); 
            UART1_SendChar('b');
            // reset when user gets up, for now we are just testing for 10s 
            // Set new desired time 
            UART0_SendString("Clock reset\r\n"); 
            newTime = t; 
            newTime.min = 0;
            newTime.sec = 0; 
            rtc_time_t raw_t1;
            do
            {
                _delay_ms(50);
                DS1307_SetTime(&newTime);
                _delay_ms(50);
                raw_t1 = DS1307_GetValidTime();
            }while(raw_t1.min != 0);
        }
        
        print_u8("H", t.hour);
        print_u8("M", t.min);
        print_u8("S", t.sec);


        uint16_t dM;
        uint16_t dR;
        vl53_bring_to_known_state(&dM, &dR);
        bool validM = is_valid_distance(dM);
        bool validR = is_valid_distance(dR);
        bool posture_ok = is_posture_ok(dM, dR);
        uint16_t diff = absdiff_u16(dM, dR);
        snprintf(buf, sizeof(buf),
                 " M=%u%s  R=%u%s  diff=%u  posture=%s\r\n",
                 dM, validM ? "" : " (INV)",
                 dR, validR ? "" : " (INV)",
                 diff,
                 posture_ok ? "OK" : "BAD");
        UART0_SendString(buf);
        if (posture_ok) {
            bad_count = 0;
            if (motor_on) {
                UART1_SendChar('b');
                Motor_SetSpeedPercent(0);
                motor_on = false;
                UART0_SendString("Posture OK -> motor OFF\r\n");
            }
        } else {
            if (bad_count < 255) bad_count++;
            if (!motor_on && bad_count >= BAD_COUNT_THRESHOLD) {
                Motor_SetSpeedPercent(100);   // vibrate
                motor_on = true;
                UART0_SendString("Posture BAD for a while -> motor ON\r\n");
                UART1_SendChar('a');
            }
        }
        _delay_ms(200);
    //    Timer1_WaitFor200ms();   // ~5 samples per second
    }
}
