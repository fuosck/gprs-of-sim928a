/**
  *
  *
  *@file gprs_queue.c
  *@author xukai
  *@data 2014-02-28
  *
  */

#include <stdio.h>
#include <string.h>
#include <gprs_queue.h>

/**
  *初始化，构造空队
  *
  *@param: 队列名称
  *@return:
  *
  */
void queue_create(circular_queue_t* queue)
{
    queue->front = queue->rail = 0;
    queue->count = 0;
}

/**
  *判断队列是否为空
  *
  *@param: 队列名称
  *@return: TRUE:空
  *         FALSE:非空
  */
bool_t is_queue_empty(circular_queue_t* queue)
{
    return 0 == queue->count ? TRUE : FALSE;
}

/**
  *判断队列是否已满
  *
  *@param: 队列名称
  *@return: TRUE:满
  *         FALSE:没满
  */
bool_t is_queue_full(circular_queue_t* queue)
{
    return QUEUE_MAXLEN == queue->count ? TRUE : FALSE;
}

/**
  *元素入队
  *
  *@param: queue：队列名称
  *         item：准备插入的元素
  *@return: TRUE:插入成功
  *         FALSE:队列已满
  */
bool_t queue_send(circular_queue_t* queue, queue_data_type item)
{
    // 入队前，判断队满
    if(is_queue_full(queue))
        return FALSE;

    queue->item[queue->rail] = item;
    queue->rail = (queue->rail + 1) % QUEUE_MAXLEN;
    queue->count++;

    return TRUE;
}

/**
  *元素出队
  *
  *@param: queue：队列名称
  *         item：出队的首元素存放的地址
  *@return: TRUE:出队成功
  *         FALSE:队列为空
  */
bool_t queue_receive(circular_queue_t* queue, queue_data_type* item)
{
    //出队前，判断队空
    if(is_queue_empty(queue))
        return FALSE;

    *item = queue->item[queue->front];
    queue->front = (queue->front + 1) % QUEUE_MAXLEN;
    queue->count--;

    return TRUE;
}

/**
  *查看队首元素
  *
  *@param: queue：队列名称
  *         item：首元素存放的地址
  *@return: TRUE:查看成功
  *         FALSE:队列为空
  */
bool_t queue_peek(circular_queue_t* queue, queue_data_type* item)
{
    //判断队空
    if(is_queue_empty(queue))
        return FALSE;

    *item = queue->item[queue->front];
    return TRUE;
}

/**
  *查看队列的长度
  *
  *@param: queue：队列名称
  *@return: 队列的长度
  *
  */
uint8_t queue_count(circular_queue_t* queue)
{
    return queue->count;
}
