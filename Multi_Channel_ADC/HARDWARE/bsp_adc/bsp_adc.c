#include "bsp_adc\bsp_adc.h"


uint16_t ADC_X_Get(void)
{
	HAL_ADC_Start(&hadc1);   //����ADC
	HAL_ADC_PollForConversion(&hadc1,50);//ADC�ȴ� ����MAX = 50ms��
	
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC))   //ADC��־�ж�
	{
		return (HAL_ADC_GetValue(&hadc1));
	}	
	return 0;
		
}

