/*
 * Measure.h
 *
 *  Created on: Apr 16, 2025
 *      Author: josel
 */

#ifndef INC_MEASURE_H_
#define INC_MEASURE_H_

#include "adc.h"

#define ADC_BUFFERM_LENGTH 1 // in 32 bits units
extern uint16_t g_ADCBufferM[];

extern uint32_t statsBuffer[4096];



#ifdef __cplusplus
extern "C"
{
#endif
	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle);
#ifdef __cplusplus
 }
#endif

#endif /* INC_MEASURE_H_ */
