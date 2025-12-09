/*
 * File:   Timer.c
 * Author: noman
 *
 * Created on December 5, 2025, 9:24 AM
 */

#include "Timer.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdbool.h>
static volatile bool _tick_200ms = false;

// 16 MHz / 64 = 250 kHz -> 0.2 s * 250000 = 50000 counts ? OCR1A = 49999
void Timer1_Init_200ms(void)
{
    TCCR1A = 0;              // normal port operation
    TCCR1B = 0;
    TCNT1  = 0;

    OCR1A = 49999;           // compare value for 200 ms

    TCCR1B |= (1 << WGM12);  // CTC mode (Clear Timer on Compare)
    TCCR1B |= (1 << CS11) | (1 << CS10); // prescaler 64

    TIMSK1 |= (1 << OCIE1A); // enable compare A match interrupt
}

void Timer1_WaitFor200ms(void)
{
    while(!_tick_200ms);
    _tick_200ms = false;
}

ISR(TIMER1_COMPA_vect)
{
    _tick_200ms = true;
}
