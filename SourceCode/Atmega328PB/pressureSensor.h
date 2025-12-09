
#ifndef PRESSURE_SENSOR_H
#define	PRESSURE_SENSOR_H

#include <stdbool.h>

// ==================== CONFIGURATION ====================

#define ADC_CHANNEL 0          // ADC0 = PC0

#define ADC_VREF 5.0
#define ADC_RESOLUTION 1024.0
#define PULL_DOWN_RESISTOR 10000.0

#define PRESSURE_NONE      20
#define PRESSURE_LOW       200
#define PRESSURE_MEDIUM    500
#define PRESSURE_HIGH      800

#define FILTER_SIZE 8

extern void ADC_start_conversion(void);
extern float calculate_resistance(uint16_t adc_val);
extern uint8_t get_pressure_level(uint16_t adc_val);
extern void PressureSensor_Init(void);
extern bool PressureSensor_IsUserPresent(void);
extern void PressureSensor_Init(void);
extern void ADC_start_conversion(void);
extern float calculate_resistance(uint16_t adc_val);
extern uint8_t get_pressure_level(uint16_t adc_val);

#endif	/* XC_HEADER_TEMPLATE_H */

