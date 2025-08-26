/*
 * ADC_Readings.h
 *
 *  Created on: Aug 26, 2025
 *      Author: vinothkannan
 */


#include "main.h"


#ifndef INC_ADC_READINGS_H_
#define INC_ADC_READINGS_H_

#define ADC_Select hadc1
extern ADC_HandleTypeDef ADC_Select;


uint32_t ADC_Auto_Unhitch_Read(uint32_t  adc_read[] );

uint8_t ADC_Current_Sensor_1_Read(uint32_t  adc_read[]) ;


#endif /* INC_ADC_READINGS_H_ */
