/*
 *
 */
#ifndef GEOMAG
#define GEOMAG

#include "stm32f0xx_hal.h"
#include "stdint.h"

__packed struct geomag_state_struct {
	uint8_t  head;
	uint8_t  len;
	uint8_t  code;
	uint32_t id;
	uint8_t  state;
	uint8_t  v;
	uint16_t m;
	uint8_t  cs;
};

static __inline int has_car(void)
{
	return GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
}

static __inline int no_car(void)
{
	return GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
}

static __inline int geomag_is_ready(void)
{
	return GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
}

static __inline int geomag_not_ready(void)
{
	return !geomag_is_ready();
}

static __inline void geomag_corr(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_Delay(100);
}

static __inline void geomag_reset(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
}

static __inline void geomag_clear_reset(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(100);
}

static __inline void geomag_sleep(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_Delay(100);
}

static __inline int geomag_is_sleep(void)
{
	return GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
}

static __inline void geomag_wakeup(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_Delay(100);
}

static __inline int geomag_is_wakeup(void)
{
	return GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
}

extern void geomag_init(void);
extern void get_geomag_state(void);
extern struct geomag_state_struct *geomag_state;
extern void put_usart2_byte(void);

#endif
