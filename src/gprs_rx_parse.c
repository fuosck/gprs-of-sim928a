/**
  *Processing the data of gprs received
  *
  *@file gprs_rx_parse.c
  *@author huangya
  *@data 2014-02-28
  *
  */

#include <gprs.h>

extern queue_data_type  n_item;      //���淢����ɵ�����
      
extern uint8_t gprs_send_data_cnt;
extern uint8_t gprs_cmd_send_cpin;  
extern uint8_t gprs_cmd_send_cgatt;  
extern uint8_t gprs_cmd_send_cipstart;
extern uint8_t gprs_cmd_recv_array[GPRS_CMD_SIZE_MAX];
extern uint16_t         gprs_cmd_recv_pos;

extern GPRS_CMD_TYPE    gprs_cmd_type;
extern gprs_mode_t      gs_mode;     //GPRS��������ģʽ
extern GPRS_MODE        gprs_mode;   //GPRS��״̬

extern circular_queue_t gprs_queue; 
extern queue_data_type n_item; 

extern gprs_type_cb_t   gprs_cb;
extern gprs_data_cb_t   gprs_get_data_cb;

static hal_timer_t *gprs_wait_connect_timer = NULL;
static hal_timer_t *gprs_query_mode_timer   = NULL; 
static uint8_t gprs_wait_connect_cnt = 0; 

static void gprs_cmd_at_recv_handler(void);
static void gprs_cmd_ate0_recv_handler(void);
static void gprs_cmd_cpin_recv_handler(void);
static void gprs_cmd_cgatt_recv_handler(void);
static void gprs_cmd_cipstart_recv_handler(void);
static void gprs_cmd_send_recv_handler(void);
static void gprs_cmd_data_recv_handler(void);
static void gprs_cmd_status_recv_handler(void);
static void gprs_cmd_cipshut_recv_handler(void);

static void gprs_wait_connect_timer_cb(void *p) 
{
    if(gprs_wait_connect_timer != NULL)
    {
        hal_timer_cancel(&gprs_wait_connect_timer); 
        DBG_ASSERT(gprs_wait_connect_timer == NULL __DBG_LINE);
    }
    
    gprs_cmd_cipstart_recv_handler();           //ֱ�ӽ���CIPSTART���ս���
}

static void gprs_query_mode_cb(void *p) //ʱ�䵽���ܽ��ܵ���ƽ̨����������
{
    if(gprs_query_mode_timer != NULL)
    {
        hal_timer_cancel(&gprs_query_mode_timer);
        DBG_ASSERT(gprs_query_mode_timer == NULL __DBG_LINE);
    }
    
    if(gprs_cmd_recv_pos == 0)   //û�н��յ�Ӧ�𣬸�֪APP
    {
        (* gprs_cb)(GS_NO_DATA_RECV, n_item.gprs_tag);
    }
    else                       //��֪APP���յ�������
    {
#if GPRS_DEBUG
        serial_write(HAL_UART_2, 
                     gprs_cmd_recv_array, 
                     mystrlen((char *)gprs_cmd_recv_array)); 
#endif
        
        gprs_receive_t recive_data = {0};
        osel_memcpy((void *)recive_data.gprs_data, 
                    gprs_cmd_recv_array, 
                    mystrlen((char *)gprs_cmd_recv_array));
            
        recive_data.len = mystrlen((char *)gprs_cmd_recv_array);
        (* gprs_get_data_cb)(recive_data);      
    }
    
    if(gs_mode == SINGLE_MODE)
    {
        gprs_close_handler();
    }
    else         //CONTINUE MODE������������������
    {
        if(GPRS_QUEUE_HAVE_DATA)
        {
            osel_post(GPRS_SEND_CMD_EVENT, 
                      (uint8_t *)(GPRS_CMD_SEND), 
                      OSEL_EVENT_PRIO_LOW);
        }
        else
        {
            osel_post(GPRS_POLL_EVENT, NULL, OSEL_EVENT_PRIO_LOW); 
        }
    }
}
      
void gprs_response_event_parse(void)
{
    if(my_strstr((const char*)gprs_cmd_recv_array, 
                    "\r\nCHARGE-ONLY MODE\r\n") != NULL)
    {
         gprs_flush_receive_buf();
         gprs_restart_handler();
    }
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                         "NORMAL POWER DOWN\r\n") != NULL)
    {
         gprs_flush_receive_buf();
         gprs_open_handler();
    }
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                        "\r\nFrom CHARGE-ONLY MODE to NORMAL MODE\r\n") != NULL) 
    {
        gprs_flush_receive_buf();
        osel_post(GPRS_POLL_EVENT, NULL, OSEL_EVENT_PRIO_LOW);             
    }
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                         "NOT READY\r\n") != NULL)
    {
        gprs_cmd_send_cpin++;        //����һ��NOT READY�ͼ�1��READY����
        gprs_flush_receive_buf();
        
        if(gprs_cmd_send_cpin < 3)
        {
            osel_post(GPRS_SEND_CMD_EVENT, 
                      (uint8_t *)(GPRS_CMD_CPIN), 
                      OSEL_EVENT_PRIO_LOW);
        }
        else
        {
            gprs_cmd_send_cpin = 0;
            gprs_close_handler();
            if(GET_DATA_FROM_QUEUE)        //��ȡ���ͻ�������һλ��Ϣ������
            {
                (* gprs_cb)(GS_ERROR, n_item.gprs_tag);
            }
        }
    } 
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                         "+PDP: DEACT\r\n") != NULL)
    {
         gprs_flush_receive_buf();
         osel_post(GPRS_SEND_CMD_EVENT, 
                   (uint8_t *)(GPRS_CMD_CIPSHUT),
                   OSEL_EVENT_PRIO_LOW);
    }
    else
    {
         switch (gprs_cmd_type)
         {
         case GPRS_CMD_AT:
             gprs_cmd_at_recv_handler();
             break;
         
         case GPRS_CMD_ATE0:
             gprs_cmd_ate0_recv_handler();
             break;
         
         case GPRS_CMD_CPIN:
             gprs_cmd_cpin_recv_handler();
             break;
         
         case GPRS_CMD_CGATT:
             gprs_cmd_cgatt_recv_handler();
             break;
         
         case GPRS_CMD_CIPSTART:
             gprs_cmd_cipstart_recv_handler();
             break;
         
         case GPRS_CMD_SEND:
             gprs_cmd_send_recv_handler();
             break;
         
         case GPRS_SEND_DATA:
             gprs_cmd_data_recv_handler();
             break;
         
         case GPRS_CMD_STATUS:
             gprs_cmd_status_recv_handler();
             break;
         
         case GPRS_CMD_CIPSHUT:
             gprs_cmd_cipshut_recv_handler();
             break;
         
         default:
             break; 
         }
    }         
}

static void gprs_cmd_at_recv_handler(void)
{
    if(my_strstr((const char*)gprs_cmd_recv_array, 
                        "\r\nERROR\r\n") != NULL) 
    {
        gprs_flush_receive_buf();
        gprs_restart_handler();
    }
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                   "\r\nOK\r\n") != NULL) 
    {
        gprs_flush_receive_buf();
        osel_post(GPRS_SEND_CMD_EVENT, 
                  (uint8_t *)(GPRS_CMD_ATE0), 
                  OSEL_EVENT_PRIO_LOW);
    }
    else
    {
        osel_post(GPRS_RESPONSE_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
    }
}

static void gprs_cmd_ate0_recv_handler(void)
{
    if(my_strstr((const char*)gprs_cmd_recv_array, 
                   "\r\nERROR\r\n") != NULL) 
    {
        gprs_flush_receive_buf();
        gprs_restart_handler();
    }
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                        "\r\nOK\r\n") != NULL)
    {
        gprs_flush_receive_buf();
        osel_post(GPRS_SEND_CMD_EVENT, 
                  (uint8_t *)(GPRS_CMD_CGATT), 
                  OSEL_EVENT_PRIO_LOW);
    }
    else
    {
        osel_post(GPRS_RESPONSE_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
    }
}

static void gprs_cmd_cpin_recv_handler(void)
{
    if(my_strstr((const char*)gprs_cmd_recv_array, 
                   "\r\nREADY\r\n") != NULL)
    {
        gprs_cmd_send_cpin = 0;
        gprs_flush_receive_buf();
        osel_post(GPRS_SEND_CMD_EVENT, 
                  (uint8_t *)(GPRS_CMD_CGATT), 
                  OSEL_EVENT_PRIO_LOW);
    }
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                        "\r\nERROR\r\n") != NULL) 
    {
        gprs_flush_receive_buf();
        gprs_restart_handler();
    }
    else
    {
        osel_post(GPRS_RESPONSE_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
    }
}

static void gprs_cmd_cgatt_recv_handler(void)
{
    if(my_strstr((const char*)gprs_cmd_recv_array, 
                   "+CGATT: 1\r\n") != NULL)
    {
        gprs_cmd_send_cgatt = 0;
        gprs_flush_receive_buf();
        osel_post(GPRS_SEND_CMD_EVENT, 
                  (uint8_t *)(GPRS_CMD_CIPSTART), 
                  OSEL_EVENT_PRIO_LOW);
    }
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                        "+CGATT: 0\r\n") != NULL)
    {
        gprs_cmd_send_cgatt++;             //����+CGATT��0��1,+CGATT��1����
        gprs_flush_receive_buf();
        
        if(gprs_cmd_send_cgatt < 3)
        {
            osel_post(GPRS_SEND_CMD_EVENT, 
                      (uint8_t *)(GPRS_CMD_CGATT), 
                      OSEL_EVENT_PRIO_LOW);
        }
        else
        {
            gprs_cmd_send_cgatt = 0;
            gprs_close_handler();
            if(GET_DATA_FROM_QUEUE);       //ȡ���ӵ����ж�ͷ
            
            (* gprs_cb)(GS_NO_NETWORK, n_item.gprs_tag);
        }
    }
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                        "\r\nERROR\r\n") != NULL) 
    {
        gprs_flush_receive_buf();
        gprs_restart_handler();
    }
    else
    {
        osel_post(GPRS_RESPONSE_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
    }
}

static void gprs_cmd_cipstart_recv_handler(void)
{
    if((my_strstr((const char*)gprs_cmd_recv_array, 
                    "CONNECT OK\r\n") != NULL )
       || (my_strstr((const char*)gprs_cmd_recv_array, 
                       "\r\nALREADY CONNECT\r\n") != NULL))
    {
        gprs_flush_receive_buf();
        gprs_cmd_send_cipstart = 0;
        gprs_wait_connect_cnt  = 0;
        gprs_mode = GPRS_CONNECTED;     //GPRS��������״̬
        osel_post(GPRS_SEND_CMD_EVENT, 
                  (uint8_t *)(GPRS_CMD_SEND), 
                  OSEL_EVENT_PRIO_LOW);
    }
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                      "CONNECT FAIL\r\n") != NULL)   //OK����˵��û��������Զ������
    {
        gprs_cmd_send_cipstart++;           //����FAIL��1������CONNECT OK����
        gprs_flush_receive_buf();
        gprs_mode = GPRS_RUN;
        
        if(gprs_cmd_send_cipstart < 2)
        {
            osel_post(GPRS_SEND_CMD_EVENT, 
                      (uint8_t *)(GPRS_CMD_CIPSTART), 
                      OSEL_EVENT_PRIO_LOW);
        }
        else                                         //����3��ʧ�ܷ���������
        {
            gprs_cmd_send_cipstart = 0;
            gprs_wait_connect_cnt  = 0;
            gprs_close_handler();
            if(GET_DATA_FROM_QUEUE);       //ȡ���ӵ����ж�ͷ
            
            (* gprs_cb)(GS_NO_NETWORK, n_item.gprs_tag);
        }
    }
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                       "OK\r\n") != NULL)            //����ֻ��OK�������5s
    {
        gprs_wait_connect_cnt++;                     //CONNECT OK����
        if(gprs_wait_connect_cnt <= 5)                //5S+5X5S=30S 
        {
            if(gprs_wait_connect_timer == NULL)
            {
                HAL_TIMER_SET_REL(MS_TO_TICK(GPRS_RESPONSE_TIME_5S),   
                                  gprs_wait_connect_timer_cb,
                                  NULL,
                                  gprs_wait_connect_timer);
                DBG_ASSERT(gprs_wait_connect_timer != NULL __DBG_LINE); 
            }
        }
        else        //30S֮�󷵻�������
        {
            gprs_wait_connect_cnt = 0;
            if(GET_DATA_FROM_QUEUE);       //ȡ���ӵ����ж�ͷ
            
            (* gprs_cb)(GS_NO_NETWORK, n_item.gprs_tag);
            gprs_close_handler();
        }
    }
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                        "\r\nERROR\r\n") != NULL) 
    {
        gprs_flush_receive_buf();
        gprs_restart_handler();
    }
    else
    {
        osel_post(GPRS_RESPONSE_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
    }
}

static void gprs_cmd_send_recv_handler(void)
{
    if(my_strstr((const char*)gprs_cmd_recv_array, 
                   ">") != NULL)
    {
        gprs_flush_receive_buf();
        osel_post(GPRS_SEND_CMD_EVENT, 
                  (uint8_t *)(GPRS_SEND_DATA), 
                  OSEL_EVENT_PRIO_LOW);
    }
    //����AT+CIPSENDָ���ERROR�������Ӵ���IPSHUT״̬����ѯ����״̬
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                        "ERROR") != NULL) 
    {
        gprs_flush_receive_buf();
        osel_post(GPRS_SEND_CMD_EVENT, 
                  (uint8_t *)(GPRS_CMD_STATUS), 
                  OSEL_EVENT_PRIO_LOW);
    }
    else
    {
        osel_post(GPRS_RESPONSE_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
    }
}

static void gprs_cmd_data_recv_handler(void)
{
    if(my_strstr((const char*)gprs_cmd_recv_array, 
                   "\r\nSEND OK\r\n") != NULL)
    {
        gprs_send_data_cnt = 0;                                   
        gprs_flush_receive_buf();
        
        if(GET_DATA_FROM_QUEUE) //������ɽ����׶���                                                      
        {
            (* gprs_cb)(GS_SEND_OK, n_item.gprs_tag);
            
            if(n_item.ack_time != 0) //�����Ӧ�𱣴�����״̬
            {
                if(gprs_query_mode_timer == NULL)                     
                {
                     HAL_TIMER_SET_REL(MS_TO_TICK((uint32_t)n_item.ack_time),   
                                       gprs_query_mode_cb,
                                       NULL,
                                       gprs_query_mode_timer);
                     DBG_ASSERT(gprs_query_mode_timer != NULL __DBG_LINE); 
                }
            }
            else              //�ж�ģʽ
            {
                if(gs_mode == SINGLE_MODE)
                {
                    gprs_close_handler();
                }
                else         //CONTINUE MODE������������������
                {
                    if(GPRS_QUEUE_HAVE_DATA)
                    {
                        osel_post(GPRS_SEND_CMD_EVENT, 
                                  (uint8_t *)(GPRS_CMD_SEND), 
                                  OSEL_EVENT_PRIO_LOW);
                    }
                    else
                    {
                        osel_post(GPRS_POLL_EVENT, NULL, OSEL_EVENT_PRIO_LOW); 
                    }
                }
            }
        }
        else       //�����Ѿ�ȡ��Ϊ�յĻ�
        {
            (* gprs_cb)(GS_SEND_OK, n_item.gprs_tag);
            osel_post(GPRS_POLL_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
        }
    }
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                        "ERROR\r\n") != NULL
            || my_strstr((const char*)gprs_cmd_recv_array, 
                         "CLOSED\r\n") != NULL
            ||my_strstr((const char*)gprs_cmd_recv_array, //����ʧ��
                        "SEND FAIL\r\n") != NULL)           //���ӶϿ����߷�����
                                                            //�ر���������CLOSED
    {                        
        gprs_flush_receive_buf();
        osel_post(GPRS_SEND_CMD_EVENT, 
                  (uint8_t *)(GPRS_CMD_STATUS), 
                  OSEL_EVENT_PRIO_LOW);
    }
    else
    {
        osel_post(GPRS_RESPONSE_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
    }
}

static void gprs_cmd_status_recv_handler(void)
{
    if((my_strstr((const char*)gprs_cmd_recv_array, 
                    "ALREADY CONNECT\r\n") != NULL) 
       || (my_strstr((const char*)gprs_cmd_recv_array, 
                       "CONNECT OK\r\n") != NULL))
    {
        gprs_flush_receive_buf();
        gprs_mode = GPRS_CONNECTED;
        osel_post(GPRS_SEND_CMD_EVENT, 
                  (uint8_t *)(GPRS_CMD_SEND), 
                  OSEL_EVENT_PRIO_LOW);
    }
    else if((my_strstr((const char*)gprs_cmd_recv_array, 
                         "STATE: IP INITIAL\r\n") != NULL) 
            || (my_strstr((const char*)gprs_cmd_recv_array, 
                            "CLOSED\r\n") != NULL))
    {
        gprs_flush_receive_buf();
        gprs_mode = GPRS_RUN;      //GPRS_RUN˵��GPRS���ڴ�״̬����û����������
        osel_post(GPRS_SEND_CMD_EVENT, 
                  (uint8_t *)(GPRS_CMD_CIPSTART),
                  OSEL_EVENT_PRIO_LOW);
    }
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                        "ERROR") != NULL)
    {
        gprs_flush_receive_buf();
        gprs_restart_handler();
    }
    else
    {
        osel_post(GPRS_RESPONSE_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
    }
}

static void gprs_cmd_cipshut_recv_handler(void)
{
    if(my_strstr((const char*)gprs_cmd_recv_array, 
                   "SHUT OK\r\n") != NULL)
    {
        gprs_flush_receive_buf();
        gprs_mode = GPRS_RUN;
        osel_post(GPRS_SEND_CMD_EVENT, 
                  (uint8_t *)(GPRS_CMD_CIPSTART), 
                  OSEL_EVENT_PRIO_LOW);
    }
    else if(my_strstr((const char*)gprs_cmd_recv_array, 
                        "ERROR") != NULL)
    {
        gprs_flush_receive_buf();
        gprs_restart_handler();
    }
    else
    {
        osel_post(GPRS_RESPONSE_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
    }
}
                          