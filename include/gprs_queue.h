/**
  *
  *
  *@file gprs_queue.h
  *@author xukai
  *@data 2014-02-28
  *
  */

#ifndef __GPRS_QUEUE_H
#define __GPRS_QUEUE_H

// �궨��ѭ�����еĿռ��С
#define QUEUE_MAXLEN        8u
#define GPRS_DATA_MAXLEN    64u

#include <osel_arch.h>

//Ԫ�����Ͷ���
typedef struct
{
    uint8_t     gprs_data_len;
    uint8_t     gprs_data[ GPRS_DATA_MAXLEN ];
    uint16_t    ack_time;
    uint16_t    gprs_tag;
}queue_data_type;

//ѭ�����нṹ�Ķ���
typedef struct
{
    uint8_t front;                             // ����ͷָ��
    uint8_t rail;                              // ����βָ��
    uint8_t count;                             // ��������ͳ�ƶ�����Ԫ�ظ���
    queue_data_type item[ QUEUE_MAXLEN ];  // �洢�����е�Ԫ��
} circular_queue_t;

// ��������
void queue_create(circular_queue_t* queue);
// �ж϶����Ƿ�Ϊ��
bool_t is_queue_empty(circular_queue_t* queue);
// �ж϶��Ƿ�Ϊ��
bool_t is_queue_full(circular_queue_t* queue);
// Ԫ�ز������
bool_t queue_send(circular_queue_t* queue, queue_data_type item);
// Ԫ�س���
bool_t queue_receive(circular_queue_t* queue, queue_data_type* item);
// ȡ����Ԫ�أ����鿴
bool_t queue_peek(circular_queue_t* queue , queue_data_type* item);
//���ض��г���
uint8_t queue_count(circular_queue_t* queue);

#endif
