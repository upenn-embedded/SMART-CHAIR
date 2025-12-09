/*
 * File:   MotorControl.c
 * Author: noman
 *
 * Created on November 16, 2025, 6:52 PM
 */

#include <avr/io.h>
#include <stdint.h>

#include "MotorControl.h"
void MotorPWM_Init(void)
{
    // Set PB1 (OC1A) as output
    DDRB |= (1 << PB1);

    // Timer1 Fast PWM 8-bit:
    // WGM10 = 1, WGM11 = 0, WGM12 = 1, WGM13 = 0  -> Fast PWM, 8-bit
    // Non-inverting mode on OC1A: COM1A1 = 1, COM1A0 = 0
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12)  | (1 << CS11);  // Prescaler = 8

    // Start with 0% duty (motor off)
    OCR1A = 0;
}

// duty: 0..255  (0 = off, 255 = full speed)
void MotorPWM_SetDuty(uint8_t duty)
{
    OCR1A = duty;
}

// 0?100% speed
void Motor_SetSpeedPercent(uint8_t percent)
{
    if (percent > 100) percent = 100;
    uint16_t duty = (uint16_t)percent * 255 / 100;
    MotorPWM_SetDuty((uint8_t)duty);
}
