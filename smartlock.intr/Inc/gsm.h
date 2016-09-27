/*
 *
 */
#ifndef GSM_H
#define GSM_H

#include "stm32f0xx_hal.h"

static __inline void gsm_reset(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

static __inline void gsm_clear_reset(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_Delay(100);
}

static __inline void gsm_sleep(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_Delay(100);
}

static __inline void gsm_wakeup(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(1000);
}

static __inline void gsm_switch(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
}

#define GSM_SEND_MSG_TYPE_AU	1
#define GSM_SEND_MSG_TYPE_AD	2
#define GSM_SEND_MSG_TYPE_MH	3
#define GSM_SEND_MSG_TYPE_ML	4
#define GSM_SEND_MSG_TYPE_HO	5
#define GSM_SEND_MSG_TYPE_RP	6
#define GSM_SEND_MSG_TYPE_FD	7
#define GSM_SEND_MSG_TYPE_FU	8
#define GSM_SEND_MSG_TYPE_SU	9
#define GSM_SEND_MSG_TYPE_AK	10
#define GSM_SEND_MSG_TYPE(t)	((t) & (~0x80000000))
#define ENABLE_AK(t)		((t) | 0x80000000)
#define AK_ENABLED(t)		((t) & 0x80000000)

__packed struct send_msg_struct {
	uint8_t head[2];
	uint8_t cmd[2];
	uint8_t ts[10];
	uint8_t sm1;
	uint8_t sm2;
	uint8_t confirm;
	uint8_t voltage[3];
	uint8_t data[10];
	uint8_t reserve[20];
	uint8_t cs[3];
	uint8_t end;
};	// 54

__packed struct recv_msg_struct {
	uint8_t head[2];
	uint8_t cmd[2];
	uint8_t confirm;
	uint8_t ts[10];
	uint8_t cs[3];
};
// flash�е�ֻ������
// ��ǰֻ�к���
__packed struct flash_ro_data {
	uint8_t phone_no[3][16];	// 0x0800F800
};

#define NR_FLASH_SMS	(1024 / sizeof(struct flash_sms))
// align(4)
// size must be a multiple of 4
// flash�еĶ�������
__packed struct flash_sms {// 0x0800FC00
	uint16_t valid;	// ������¼�Ƿ���Ч����Ч��ʾ����δ�ܷ��ͳɹ�����Ч��ʾ�����ѷ���
	struct send_msg_struct msg;	// ʵ�ʵĶ�������
};
// align(4)
// �ڴ��еĶ�������
struct flash_sms_mem {	
	int index;	// ����������flash�е�����
	int try;	// ���Ͷ��ŵĳ��Դ�����Ϊ0ʱ�������뷢�ͻ���ŷ�����ȥ
	int is_sent;
	uint8_t *phone_no;
	struct flash_sms sms;
};

extern struct flash_sms_mem sms_mem;
extern uint8_t gsm_cmd_cmgs[3][28];
// �ж�flash���Ƿ���Ҫ���͵Ķ���
static __inline int has_flash_sms(void)
{
	extern struct flash_sms *flash_sms;
	return 1 == flash_sms[sms_mem.index].valid;
}

extern void gsm_init(void);
extern void gsm_send_msg(void);
extern void gsm_recv_msg(struct recv_msg_struct *msg);
extern void confirm_sms(void);
extern void save_sms(void);
extern void put_usart1_byte(void);

#endif
