#include "bsp_exti.h"
#include "main.h"
#include "stdbool.h"

volatile bool mpu_ready = false;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_5)
	{
		mpu_ready = true;
		
	}
}