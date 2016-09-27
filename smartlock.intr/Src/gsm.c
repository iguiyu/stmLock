/*
 *
 */
#include <string.h>

#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "gsm.h"
#include "misc.h"

extern UART_HandleTypeDef huart1;
extern void Error_Handler(void);

#ifdef INIT_GSM
static uint8_t cmd_set_baud_rate[] = "AT+IPR=2400\r\n";	//
static uint8_t gsm_cmd_cmgf[] = "AT+CMGF=1\r\n";	// select SMS message format: text mode
static uint8_t gsm_cmd_cscs[] = "AT+CSCS=\"GSM\"\r\n";
static uint8_t gsm_cmd_cpms[] = "AT+CPMS=\"MT\",\"MT\",\"MT\"\r\n";
static uint8_t gsm_cmd_cmgd[] = "AT+CMGD=1,4\r\n";	// delete sms
static uint8_t gsm_cmd_cnmi[] = "AT+CNMI=2,2,0,0\r\n";	//
static uint8_t gsm_cmd_savegsm[] = "AT&W\r\n";		// save gsm
#endif

static uint8_t gsm_cmd_cpin[] = "AT+CPIN?\r\n";
int cpin_ok = 0;
static uint8_t gsm_cmd_creg[] = "AT+CREG?\r\n";
int creg_ok = 0;

// phone number
// 手机号码：
// [0]: 主号码
// [1]: 副号码
// [2]: 管理号码
uint8_t gsm_cmd_cmgs[3][28];
//volatile int usart1_intr = 0;
// 这是flash中的只读数据，开始时烧录进去
// 当前只有3个号码
static struct flash_ro_data *flash_ro_data = (struct flash_ro_data *)0x0800F800;
// 用于保存未发送的短信
// 当一页(1024)flash都用完时，檫处数据
struct flash_sms *flash_sms = (struct flash_sms *)0x0800FC00;
// 内存中的用于发送短信的数据记录
__align(4) struct flash_sms_mem sms_mem;

// GSM初始化函数
void gsm_init(void)
{
	int i;
	// 清除RESET标志
	gsm_clear_reset();
	// 清除SLEEP标志
	gsm_wakeup();
	// 给GSM上电
	gsm_switch();
	HAL_Delay(1000);
	
	// make phone number
	// 从flash中读取保存的手机号码。
	// 读取主号码
	memcpy(&gsm_cmd_cmgs[0], "AT+CMGS=\"", 9);
	memcpy(&gsm_cmd_cmgs[0][9], flash_ro_data->phone_no[0], 13);
	gsm_cmd_cmgs[0][22] = '"';
	gsm_cmd_cmgs[0][23] = '\r';
	gsm_cmd_cmgs[0][24] = '\n';
	gsm_cmd_cmgs[0][25] = '\0';
	// 读取副号码
	memcpy(&gsm_cmd_cmgs[1], "AT+CMGS=\"", 9);
	memcpy(&gsm_cmd_cmgs[1][9], flash_ro_data->phone_no[1], 13);
	gsm_cmd_cmgs[1][22] = '"';
	gsm_cmd_cmgs[1][23] = '\r';
	gsm_cmd_cmgs[1][24] = '\n';
	gsm_cmd_cmgs[1][25] = '\0';
	// 读取管理号码
	memcpy(&gsm_cmd_cmgs[2], "AT+CMGS=\"", 9);
	memcpy(&gsm_cmd_cmgs[2][9], flash_ro_data->phone_no[2], 13);
	gsm_cmd_cmgs[2][22] = '"';
	gsm_cmd_cmgs[2][23] = '\r';
	gsm_cmd_cmgs[2][24] = '\n';
	gsm_cmd_cmgs[2][25] = '\0';
	
	// check flash sms
	// 检查flash中是否有保存的未发送的短信
	memset(&sms_mem, 0, sizeof(sms_mem));
	for (i = 0; i < NR_FLASH_SMS; ++i) {
		if (1 == flash_sms[i].valid) {
			sms_mem.index = i;
			memcpy(&sms_mem.sms, &flash_sms[i].msg, sizeof(struct flash_sms));
			break;
		}
	}
	// 检查SIM卡工作是否正常
	while (cpin_ok == 0) {
		HAL_UART_Transmit(&huart1, gsm_cmd_cpin, strlen((char *)gsm_cmd_cpin), 1000);
		osDelay(2000);
	}
	// 检查GSM网络注册情况
	while (creg_ok == 0) {
		HAL_UART_Transmit(&huart1, gsm_cmd_creg, strlen((char *)gsm_cmd_creg), 1000);
		osDelay(2000);
	}
	
	// 让GSM模块睡眠，可以接收短信
	gsm_sleep();
}

#if 0
// 保存短信到flash
void save_sms(void)
{
	int i;
	uint32_t *data = (uint32_t *)&sms_mem.sms;
	
	sms_mem.index++;
	// erase flash page
	// 整页的flash都已用完，先擦除整页内容，再重新开始存储当前短信
	if (NR_FLASH_SMS == sms_mem.index) {
		FLASH_EraseInitTypeDef EraseInit;
		uint32_t PageError;
	
		EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
		EraseInit.PageAddress = (uint32_t)flash_sms;
		EraseInit.NbPages = 1;
	
		HAL_FLASH_Unlock();
		HAL_FLASHEx_Erase(&EraseInit, &PageError);
		HAL_FLASH_Lock();
		
		sms_mem.index = 0;
	}
	sms_mem.sms.valid = 1;
	
	HAL_FLASH_Unlock();
	for (i = 0; i < sizeof(*flash_sms) / sizeof(*data); ++i) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 
		(uint32_t)&flash_sms[sms_mem.index], data[i]);
	}
	HAL_FLASH_Lock();
}
// 收到短信回执后，标识flash中的对应短信为已发送
void confirm_sms(void)
{
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 
		(uint32_t)&flash_sms[sms_mem.index].valid, 0);
	HAL_FLASH_Lock();
	phone_no = gsm_cmd_cmgs;
}
#endif

void gsm_send_msg(void)
{
	// 唤醒GSM模块，准备发送短信
	osDelay(3000);
	gsm_wakeup();
	HAL_UART_Transmit(&huart1, sms_mem.phone_no, strlen((char *)sms_mem.phone_no), 1000);
	osDelay(2000);
	HAL_UART_Transmit(&huart1, (uint8_t *)&sms_mem.sms.msg, sizeof(struct send_msg_struct), 1000);
	osDelay(2000);
	gsm_sleep();
	sms_mem.try--;
}
