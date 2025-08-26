/*
 * ADC_Readings.c
 *
 *  Created on: Aug 26, 2025
 *      Author: vinothkannan
 */

#include "ADC_Readings.h"

void ADC_Select_CH0 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_0;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&ADC_Select, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH1 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&ADC_Select, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH2 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&ADC_Select, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH3 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_3;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&ADC_Select, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}





uint32_t ADC_Auto_Unhitch_Read(uint32_t  adc_read[] )  {


	ADC_Select_CH0();
	HAL_ADC_Start(&ADC_Select);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	adc_read[0] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&ADC_Select);

	return 0;
}

uint8_t ADC_Current_Sensor_1_Read(uint32_t  adc_read[])  {

	uint8_t Current_Sensor1 = 0;
	float A_SEN1 =0;
	ADC_Select_CH1();
	HAL_ADC_Start(&ADC_Select);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	adc_read[1] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&ADC_Select);
//	A_SEN1 = ( (float)ADC_VAL[2] + 89.42 ) /266.3;
//	A_SEN1 = A_SEN1 * 10;
//	Current_Sensor1 = (uint8_t)A_SEN1;


	return 0;
}

uint8_t ADC_Current_Sensor_2_Read(uint32_t  adc_read[])  {

	uint8_t Current_Sensor2 = 0;
	float A_SEN2 =0;
	ADC_Select_CH2();
	HAL_ADC_Start(&ADC_Select);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	adc_read[2] = HAL_ADC_GetValue(&ADC_Select);
	HAL_ADC_Stop(&ADC_Select);
//	A_SEN2 = ( (float)ADC_VAL[3] + 89.42 ) /266.3;
//	A_SEN2 = A_SEN2 * 10;
//	Current_Sensor2 = (uint8_t)A_SEN2;

	return 0;
}

uint32_t ADC_24V_Read(uint32_t  adc_read[])  {



	ADC_Select_CH3();
	HAL_ADC_Start(&ADC_Select);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	adc_read[3] = HAL_ADC_GetValue(&ADC_Select);
	HAL_ADC_Stop(&ADC_Select);

	return 0;
}
