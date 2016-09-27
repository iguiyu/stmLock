/*
 *
 */
#ifndef MISC
#define MISC

#include "stm32f0xx_hal.h"

static __inline void beep(uint32_t millisec)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_Delay(millisec);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
}

static __inline void __lock_up(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

static __inline void __lock_down(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}

static __inline void lock_off(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}

static __inline int lock_is_running(void)
{
	return GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) ||
		GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
}

static __inline int lock_up_ok(void)
{
	return GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
}

static __inline int lock_down_ok(void)
{
	return GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
}

static __inline void ir_off(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
}

static __inline void ir_on(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}

static __inline int shell_is_open(void)
{
	return GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
}


#endif
