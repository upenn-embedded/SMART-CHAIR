/*
 * File:   PressureSensor.c
 * Author: noman
 *
 * Created on December 3, 2025, 4:26 PM
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "UART.h"

#include "pressureSensor.h"
volatile uint16_t adc_value = 0;
volatile uint8_t adc_ready = 0;

float voltage = 0.0;
float resistance = 0.0;
uint8_t pressure_level = 0;

bool PressureSensor_IsUserPresent(void)
{
    ADC_start_conversion();

    while(!adc_ready);
    adc_ready = 0;
    voltage = (adc_value * ADC_VREF) / ADC_RESOLUTION;
    resistance = calculate_resistance(adc_value);
    pressure_level = get_pressure_level(adc_value);
    bool isUserPresent;
    isUserPresent = pressure_level == 0 ? false : true;
    return isUserPresent;
}

void PressureSensor_Init(void)
{
    // Ensure PC0 is input and no pull-up
    DDRC &= ~(1 << PC0);
    PORTC &= ~(1 << PC0);
    // Select AVCC as reference voltage
    ADMUX = (1 << REFS0);  // REFS0=1 ? AVCC reference

    // Select ADC0
    ADMUX &= 0xE0;         // Clear MUX[3:0]
    ADMUX |= ADC_CHANNEL;  // 0 for PC0

    // ATmega328PB: ADCSRB does NOT contain MUX5.
    ADCSRB = 0x00;         // Free-running off, MUX5 does NOT exist here

    // Disable digital input on ADC0 (improves accuracy)
    DIDR0 |= (1 << ADC0D);

    // ADC enable, interrupt enable, prescaler = 128
    ADCSRA = (1 << ADEN) |
             (1 << ADIE) |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void ADC_start_conversion(void)
{
    ADCSRA |= (1 << ADSC);
}

float calculate_resistance(uint16_t adc_val)
{
    if(adc_val < PRESSURE_NONE)
        return 10000000.0;

    if(adc_val >= 1023)
        return 100.0;

    return PULL_DOWN_RESISTOR * ((ADC_RESOLUTION - 1) - adc_val) / adc_val;
}

uint8_t get_pressure_level(uint16_t adc_val)
{
    if(adc_val < PRESSURE_NONE)        return 0;
    else if(adc_val < PRESSURE_LOW)    return 1;
    else if(adc_val < PRESSURE_MEDIUM) return 2;
    else if(adc_val < PRESSURE_HIGH)   return 3;
    else                               return 4;
}

void print_reading(void)
{
    // adc_value
    //resistance
}
ISR(ADC_vect)
{
    adc_value = ADCL;
    adc_value |= (ADCH << 8);
    adc_ready = 1;
}
