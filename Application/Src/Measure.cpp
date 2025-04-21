/*
 * Measure.cpp
 *
 *  Created on: Apr 16, 2025
 *      Author: josel
 */

extern "C"
{
#include "main.h"
#include "gpio.h"
}

#include "Measure.h"

uint16_t g_ADCBufferM[ADC_BUFFERM_LENGTH*4];
static volatile uint16_t * pM_VIN = &g_ADCBufferM[0];
static volatile uint16_t * pM_VIN2 = &g_ADCBufferM[1];

uint32_t statsBuffer[4096];

bool bErrorDMA;
int  bErrorADC;

static int countEOC;

static volatile unsigned long long measureCount;
static volatile bool doneADC;

unsigned long long getMeasureCount()
{
	return measureCount;
}


void   HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) // data overrun
{
	if (hadc->ErrorCode == HAL_ADC_ERROR_DMA) {
		bErrorDMA = true;
	} else if (hadc->ErrorCode == HAL_ADC_ERROR_OVR) {
		resetADCOverrun(hadc);
		bErrorADC = 1;
	} else {
		bErrorADC = 2;
	}
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle)
{// end of DMA
	measureCount++;
	if (isADC_EOC(adcHandle)){
		//doPsenseToggle();
		//Error_Handler();// we should not be there
		countEOC++;
		doPinFast(SYNC_GPIO_Port, SYNC_Pin);
		return;
	}
	if (isADC_EOS(adcHandle))	{
		if (adcHandle == &hadc3) {
			statsBuffer[g_ADCBufferM[0]]++;
			doPinFast(SYNC_GPIO_Port, SYNC_Pin);
			doneADC = true;
		}	else {
			// end of adc1 processing
			Error_Handler();
		}
	}	else {
		Error_Handler();
	}
}
