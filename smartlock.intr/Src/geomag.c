/*
 *
 */
#include <stdlib.h>
#include <string.h>
#include "cmsis_os.h"

#include "geomag.h"
#include "misc.h"

struct usart2_buf {
	int idx;
	uint8_t data[64];
} usart2_buf;

extern UART_HandleTypeDef huart2;
extern void Error_Handler(void);

static uint8_t usart2_rx_byte;
struct geomag_state_struct *geomag_state = (struct geomag_state_struct *)usart2_buf.data;
// 请求地磁状态命令
static uint8_t STATE_REQUEST[4] = { 0xAA, 0x00, 0x04, 0x04 };
// 关闭周期性状态报告命令
static uint8_t CLOSE_STATE_REPORT[] = {
	0xAA, 0x14, 0x02, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xE9};
//volatile int usart2_intr = 0;

void put_usart2_byte(void)
{
	__HAL_UART_FLUSH_DRREGISTER(&huart2); // Clear the buffer to prevent overrun
	if (sizeof(usart2_buf.data) == usart2_buf.idx + 1)
		return;
	usart2_buf.data[usart2_buf.idx] = usart2_rx_byte;
	usart2_buf.idx++;
}

static __inline void usart2_buf_init(void)
{
	usart2_buf.idx = 0;
	memset(geomag_state, 0, sizeof(*geomag_state));
}

void geomag_init(void)
{
	usart2_buf_init();
	__HAL_UART_FLUSH_DRREGISTER(&huart2);
	HAL_UART_Receive_DMA(&huart2, &usart2_rx_byte, 1);
	// 清除RESET标志
	geomag_clear_reset();
	HAL_Delay(1000);
	// 强制模块立即执行一次校正
	geomag_corr();
	HAL_Delay(2000);
	// 关闭周期性状态报告
	if (HAL_OK != HAL_UART_Transmit(&huart2, CLOSE_STATE_REPORT, 24, 2000))
		Error_Handler();
	HAL_Delay(2000);
	// 检测地磁是否完成学习

	while (1) {
		if (geomag_is_ready())
			break;
		HAL_Delay(1000);
	}

	// 此时，模块已经完成校正，并进入检测状态
	// 地磁睡眠
	geomag_sleep();
}
// 获取地磁状态
void get_geomag_state(void)
{
	geomag_wakeup();
	osDelay(500);
	usart2_buf_init();
	if (HAL_OK != HAL_UART_Transmit(&huart2, STATE_REQUEST, 4, 1000))
		return;
	HAL_Delay(500);
	//usart_intr = 0;
	geomag_sleep();
}
