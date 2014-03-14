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

// 宏定义循环队列的空间大小
#define QUEUE_MAXLEN        8u
#define GPRS_DATA_MAXLEN    64u

#include <osel_arch.h>

//元素类型定义
typedef struct
{
    uint8_t     gprs_data_len;
    uint8_t     gprs_data[ GPRS_DATA_MAXLEN ];
    uint16_t    ack_time;
    uint16_t    gprs_tag;
}queue_data_type;

//循环队列结构的定义
typedef struct
{
    uint8_t front;                             // 队列头指针
    uint8_t rail;                              // 队列尾指针
    uint8_t count;                             // 计数器，统计队列中元素个数
    queue_data_type item[ QUEUE_MAXLEN ];  // 存储队列中的元素
} circular_queue_t;

// 创建队列
void queue_create(circular_queue_t* queue);
// 判断队列是否为空
bool_t is_queue_empty(circular_queue_t* queue);
// 判断队是否为满
bool_t is_queue_full(circular_queue_t* queue);
// 元素插入队列
bool_t queue_send(circular_queue_t* queue, queue_data_type item);
// 元素出队
bool_t queue_receive(circular_queue_t* queue, queue_data_type* item);
// 取队首元素，仅查看
bool_t queue_peek(circular_queue_t* queue , queue_data_type* item);
//返回队列长度
uint8_t queue_count(circular_queue_t* queue);

#endif
