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
  *��ʼ��������ն�
  *
  *@param: ��������
  *@return:
  *
  */
void queue_create(circular_queue_t* queue)
{
    queue->front = queue->rail = 0;
    queue->count = 0;
}

/**
  *�ж϶����Ƿ�Ϊ��
  *
  *@param: ��������
  *@return: TRUE:��
  *         FALSE:�ǿ�
  */
bool_t is_queue_empty(circular_queue_t* queue)
{
    return 0 == queue->count ? TRUE : FALSE;
}

/**
  *�ж϶����Ƿ�����
  *
  *@param: ��������
  *@return: TRUE:��
  *         FALSE:û��
  */
bool_t is_queue_full(circular_queue_t* queue)
{
    return QUEUE_MAXLEN == queue->count ? TRUE : FALSE;
}

/**
  *Ԫ�����
  *
  *@param: queue����������
  *         item��׼�������Ԫ��
  *@return: TRUE:����ɹ�
  *         FALSE:��������
  */
bool_t queue_send(circular_queue_t* queue, queue_data_type item)
{
    // ���ǰ���ж϶���
    if(is_queue_full(queue))
        return FALSE;

    queue->item[queue->rail] = item;
    queue->rail = (queue->rail + 1) % QUEUE_MAXLEN;
    queue->count++;

    return TRUE;
}

/**
  *Ԫ�س���
  *
  *@param: queue����������
  *         item�����ӵ���Ԫ�ش�ŵĵ�ַ
  *@return: TRUE:���ӳɹ�
  *         FALSE:����Ϊ��
  */
bool_t queue_receive(circular_queue_t* queue, queue_data_type* item)
{
    //����ǰ���ж϶ӿ�
    if(is_queue_empty(queue))
        return FALSE;

    *item = queue->item[queue->front];
    queue->front = (queue->front + 1) % QUEUE_MAXLEN;
    queue->count--;

    return TRUE;
}

/**
  *�鿴����Ԫ��
  *
  *@param: queue����������
  *         item����Ԫ�ش�ŵĵ�ַ
  *@return: TRUE:�鿴�ɹ�
  *         FALSE:����Ϊ��
  */
bool_t queue_peek(circular_queue_t* queue, queue_data_type* item)
{
    //�ж϶ӿ�
    if(is_queue_empty(queue))
        return FALSE;

    *item = queue->item[queue->front];
    return TRUE;
}

/**
  *�鿴���еĳ���
  *
  *@param: queue����������
  *@return: ���еĳ���
  *
  */
uint8_t queue_count(circular_queue_t* queue)
{
    return queue->count;
}
