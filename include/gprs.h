/**
  *The header file for gprs.c
  *
  *@file gprs.h
  *@author huangya
  *@data 2014-03-01
  *
  */

#ifndef __GPRS_H
#define __GPRS_H

#include <stdio.h>
#include <gprs_queue.h>
#include <gprs_rx_parse.h>
#include <driver.h>
#include <osel_arch.h>
#include <hal_timer.h>
#include <gprs_str.h>

#define GPRS_CMD_SIZE_MAX        128u

//������Ϣ���Ͷ���
#define GPRS_TYPE_CB_MAX         6u     
#define GS_SEND_OK               0X00
#define GS_SEND_FAIL             0X01
#define GS_ERROR                 0X02
#define GS_NO_NETWORK            0X03
#define GS_NO_DATA_SEND          0X04
#define GS_NO_DATA_RECV          0X05

#define GPRS_RESPONSE_TIME_500MS 500u
#define GPRS_RESPONSE_TIME_1S    1000u
#define GPRS_RESPONSE_TIME_2S    2000u 
#define GPRS_RESPONSE_TIME_4S    4000u
#define GPRS_RESPONSE_TIME_5S    5000u  

#define PEEK_QUEUE_FIRST_DATA (queue_peek(&gprs_queue, &n_item) \
== TRUE)

#define GET_DATA_FROM_QUEUE   (queue_receive(&gprs_queue, &n_item) \
== TRUE)

#define GPRS_QUEUE_HAVE_DATA  (is_queue_empty(&gprs_queue) == FALSE)

#define GET_QUEUE_LEN         (queue_count(&gprs_queue))

//����ģʽ����
typedef enum
{
	SINGLE_MODE,
    CONTINE_MODE,
}gprs_mode_t;

//gprs�����������Ͷ���
typedef struct{
    uint8_t *gprs_data;
    uint16_t len;
}gprs_receive_t;

//gprs����״̬ö��
typedef enum{
	GPRS_SHUTDOWN,
	GPRS_RUN,
    GPRS_CONNECTED,
	GPRS_SLEEP,
    GPRS_CLOSE,
}GPRS_MODE;

//gprsATָ��ö��
typedef enum {
    GPRS_CMD_NULL,
    GPRS_CMD_AT,
    GPRS_CMD_ATE0,
    GPRS_CMD_CGATT,
    GPRS_CMD_CIPSTART,
    GPRS_CMD_SEND,
    GPRS_CMD_STATUS,
    GPRS_CMD_CIPSHUT,
    GPRS_CMD_CIPCLOSE,
    GPRS_CMD_CPIN,
    GPRS_SEND_DATA,
}GPRS_CMD_TYPE;

//�����ص�����
typedef void (*gprs_type_cb_t)(uint16_t param, uint16_t tag);
typedef void (*gprs_data_cb_t)(gprs_receive_t param);

//ע��ص�������������
typedef struct
{
    uint32_t  ip_addr;
    uint16_t  port;
    gprs_type_cb_t type_cb;
    gprs_data_cb_t data_cb;
}gprs_config_t;

/**
  *ע��gprs�ص�����
  *
  *@param: ��
  *@return: ��
  *
  */
bool_t gprs_config(const gprs_config_t *config);

/**
  *gprs��ʼ���������������е�ʱ�����
  *
  *@param: ��
  *@return: ��
  *
  */
void gprs_init(void);

void gprs_flush_receive_buf(void);
void gprs_close_handler(void);
void gprs_restart_handler(void);
void gprs_open_handler(void);
void gprs_uart_recv_handler(uint8_t ch);

/**
  *���뽫Ҫͨ��GPRS�������ݵĽӿ�
  *
  *@param: *data_p:Ҫ���͵�����ָ��
  *        len:    ���ݳ���
  *        tag:    ����֡���
  *        time:   �ȴ�Ӧ���ʱ�� ��λMS
  *@return: 0�� ����ʧ��
  *         1~8 ����֮��GPRS�������ĳ���
  *         9�� ��������������ʧ��
  */
uint8_t gprs_send(void *data_p, uint8_t len, uint16_t tag, uint16_t time);

/**
  *gprs���÷���ģʽ�ӿ�
  *
  *@param: sig������ģʽ
  *             SINGLE_MODE  ��һģʽ
  *             CONTINE_MODE ����ģʽ
  *@return: ��
  *
  */
void gprs_set_mode(gprs_mode_t sig);

#endif
