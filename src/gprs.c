/**
  *provides drive and task for gprs.
  *
  *@file gprs.c
  *@author huangya
  *@data 2014-03-01
  *
  */

#include <gprs.h>
#include <serial.h>
#include <tfp_printf.h>

#define GPRS_DEBUG     1       //是否串口打印
#define GPRS_TEST      0       //测试用

#define GPRS_POLL_TIME               2000u  

#define GPRS_SERIAL_LEN_MAX          100u
#define GPRS_SERIAL_LEN_MIN          4u

#define GPRS_SEND_CMD_MAX_CNT        3u

#define  GPRS_AT                     "AT\r"
#define  GPRS_ATE0                   "ATE0\r"
#define  GPRS_CGATT                  "AT+CGATT?\r"
#define  GPRS_CPIN                   "AT+CPIN?\r"
#define  GPRS_CIPSTART               "AT+CIPSTART="
#define  GPRS_CIPSEND                "AT+CIPSEND="
#define  GPRS_CIPSTATUS              "AT+CIPSTATUS\r"
#define  GPRS_CIPSHUT                "AT+CIPSHUT\r"
#define  GPRS_CIPCLOSE               "AT+CIPCLOSE\r"
#define  GPRS_CR                     "\r"

#define  GPRS_RESTARTING             "gprs restart\r\n"
#define  GPRS_OPENING                "gprs open\r\n"
#define  GPRS_CLOSEING               "gprs close\r\n"

/*************私有变量******************/
//远程主机地址
static uint8_t ip_config[48]; 
static const uint8_t ip_mode[] = {"\"UDP\""};                

static hal_timer_t *gprs_task_timer       = NULL; 
static hal_timer_t *gprs_response_timer   = NULL;
static hal_timer_t *gprs_open_timer       = NULL;

static uint8_t gprs_cmd_send_cnt   = 0;
static uint8_t gprs_restart_cnt    = 0;
/*************全局变量******************/
uint8_t          gprs_cmd_recv_array[GPRS_CMD_SIZE_MAX]; //GPRS 接收缓冲区数组
uint16_t         gprs_cmd_recv_pos = 0;                  //GPRS 接收缓冲区索引
circular_queue_t gprs_queue;                             //GPRS 发送缓冲区队列

queue_data_type  n_item; 

//回调函数全局变量
gprs_data_cb_t   gprs_get_data_cb;
gprs_type_cb_t   gprs_cb; 

GPRS_MODE        gprs_mode     = GPRS_CLOSE;             //GPRS的状态
gprs_mode_t      gs_mode       = SINGLE_MODE;            //GPRS发送数据模式
GPRS_CMD_TYPE    gprs_cmd_type = GPRS_CMD_NULL;

uint8_t gprs_send_data_cnt     = 0; 
uint8_t gprs_cmd_send_cpin     = 0;     
uint8_t gprs_cmd_send_cgatt    = 0;
uint8_t gprs_cmd_send_cipstart = 0;

/*************内部函数声明***************/
static void prwkey_down(void);
static void gprs_send_event_handler(void);
static void gprs_poll_event_handler(void);
static void gprs_response_event_handler(void);
static void gprs_send_cmd_event_handler(uint8_t type);
static void gprs_send_cmd(uint8_t *cmd, uint8_t len);
static void gprs_open_timer_cb(void *p);
static void gprs_response_cb(void* p);
static void gprs_switch(void);
static void gprs_restart(void);
static void gprs_uart_init(void);
static void sim928a_gprs_init(void);

void gprs_flush_receive_buf(void)
{
    osel_memset(gprs_cmd_recv_array, 0x00, sizeof(gprs_cmd_recv_array));
    gprs_cmd_recv_pos = 0;
}

void gprs_uart_recv_handler(uint8_t ch)
{
    gprs_cmd_recv_array[gprs_cmd_recv_pos] = ch; //串口中断字节放入接收缓冲区里
    gprs_cmd_recv_pos++;
}

void gprs_task(void *e)
{
    DBG_ASSERT(e != NULL __DBG_LINE);
    osel_event_t *p = (osel_event_t *)e;

    switch (p->sig)
    {
    case GPRS_SEND_EVENT:
        gprs_send_event_handler();     //APP插入数据发送就关闭任务循环定时器
        break;
        
    case GPRS_POLL_EVENT:
        gprs_poll_event_handler();     //gprs任务循环定时器
        break;
                      
    case GPRS_SEND_CMD_EVENT:          //发送CMD
        gprs_send_cmd_event_handler((uint8_t)((uint32_t)(p->param)));
        break;
        
    case GPRS_RESPONSE_EVENT:
        gprs_response_event_handler();
           
    default:
        break;
    }
}

#if GPRS_TEST
static void gprs_test_send_cb(uint16_t param, uint16_t tag)
{
    ;
}

static void gprs_test_receive_cb(gprs_receive_t param)
{
    ;
}
#endif

void gprs_init(void)
{
    sim928a_gprs_init();
    gprs_uart_init();
    queue_create(&gprs_queue);         // 初始化，构造空队
    
#if GPRS_TEST
    gprs_config_t gprs_test_config;
    
#define BUILD_IP_ADDRESS(b3, b2, b1, b0) ((uint32_t)(b3) << 24) | \
     ((uint32_t)(b2) << 16) | ((uint32_t)(b1) << 8) | ((uint32_t)(b0))        
        
    gprs_test_config.ip_addr = BUILD_IP_ADDRESS(58, 214, 236, 152);
    gprs_test_config.port = 8066;
    gprs_test_config.type_cb = gprs_test_send_cb;
    gprs_test_config.data_cb = gprs_test_receive_cb;    
    gprs_config(&gprs_test_config);
    
    uint8_t DATA[4] = {"1234"};
    gprs_set_mode(CONTINE_MODE);
    gprs_send(DATA, 0x04, 1, 0);
    gprs_send(DATA, 0x04, 2, 0);
    gprs_send(DATA, 0x04, 3, 0);
    gprs_send(DATA, 0x04, 4, 0);
    gprs_send(DATA, 0x04, 4, 1000);
    gprs_send(DATA, 0x04, 4, 1000);
    gprs_send(DATA, 0x04, 4, 1000);
#endif
    
    gprs_flush_receive_buf();
    
    wsnos_init_printf(NULL, NULL);
    
    // 创建任务
    osel_task_tcb *gprs_task_handle = osel_task_create(&gprs_task, 
                                                       GPRS_TASK_PRIO);
    // 消息绑定
    osel_subscribe(gprs_task_handle, GPRS_SEND_EVENT);
    osel_subscribe(gprs_task_handle, GPRS_POLL_EVENT);
    osel_subscribe(gprs_task_handle, GPRS_SEND_CMD_EVENT);
    osel_subscribe(gprs_task_handle, GPRS_RESPONSE_EVENT);
    
    osel_post(GPRS_POLL_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
}

void gprs_restart_handler(void)
{
#if GPRS_DEBUG
    serial_write(HAL_UART_2, GPRS_RESTARTING, sizeof(GPRS_RESTARTING)-1); 
#endif
    
    if(gprs_restart_cnt > 3)
    {
        if(GET_DATA_FROM_QUEUE)        //提取发送缓冲区第一位信息并丢弃
        {
            gprs_restart_cnt = 0;
            (* gprs_cb)(GS_ERROR, n_item.gprs_tag);
        }
        gprs_close_handler();
    }
    else
    {
        gprs_restart_cnt++;
        gprs_mode = GPRS_RUN;
        gprs_restart();
        if(gprs_open_timer == NULL)
        {
            HAL_TIMER_SET_REL(MS_TO_TICK(GPRS_RESPONSE_TIME_1S),   
                              gprs_open_timer_cb,
                              NULL,
                              gprs_open_timer);
            DBG_ASSERT(gprs_open_timer != NULL __DBG_LINE); 
        }
    }
}

void gprs_close_handler(void)
{
#if GPRS_DEBUG
    serial_write(HAL_UART_2, GPRS_CLOSEING, sizeof(GPRS_CLOSEING)-1); 
#endif
    
    gprs_switch();
    gprs_flush_receive_buf();
    
    while (GPRS_QUEUE_HAVE_DATA)         //清空发送缓冲区
    {
        queue_receive(&gprs_queue, &n_item);
    }
    
    gprs_restart_cnt    = 0;
    gprs_cmd_send_cnt   = 0;
    gprs_send_data_cnt  = 0;
    gprs_cmd_send_cpin  = 0; 
    gprs_cmd_send_cipstart = 0;
    gprs_cmd_send_cgatt = 0;
    
    gprs_mode = GPRS_CLOSE;
    gprs_cmd_type = GPRS_CMD_NULL;
    osel_post(GPRS_POLL_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
}

void gprs_open_handler(void)
{
#if GPRS_DEBUG
    serial_write(HAL_UART_2, GPRS_OPENING, sizeof(GPRS_OPENING)-1); 
#endif    
    
    gprs_switch();
    
    if(gprs_open_timer == NULL)
    {
        HAL_TIMER_SET_REL(MS_TO_TICK(GPRS_RESPONSE_TIME_1S),   
                          gprs_open_timer_cb,
                          NULL,
                          gprs_open_timer);
        DBG_ASSERT(gprs_open_timer != NULL __DBG_LINE); 
    }
    
    gprs_mode = GPRS_RUN;
    
    gprs_restart_cnt    = 0;
    gprs_cmd_send_cnt   = 0;
    gprs_send_data_cnt  = 0;
    gprs_cmd_send_cpin  = 0; 
    gprs_cmd_send_cipstart = 0;
    gprs_cmd_send_cgatt = 0;  
    
    gprs_flush_receive_buf();
    hal_uart_recv_enable(HAL_UART_4);
}

/*************与APP接口函数***************/
bool_t gprs_config(const gprs_config_t *config)
{
    uint32_t addr;
    uint16_t port;
 
    DBG_ASSERT(config != NULL __DBG_LINE);
    DBG_ASSERT(config ->data_cb != NULL __DBG_LINE);
    DBG_ASSERT(config ->type_cb != NULL __DBG_LINE);

    if(config != NULL)
    {
        OSEL_ISR_ENTRY();
        addr = config->ip_addr;
        port = config->port;

        uint8_t ip[4]; 
        ip[0]   = HI_1_UINT32(addr);
        ip[1]   = HI_2_UINT32(addr);
        ip[2]   = HI_3_UINT32(addr);
        ip[3]   = HI_4_UINT32(addr);
        wsnos_sprintf((char *)ip_config, 
                      "\"%d.%d.%d.%d\",%u", 
                      ip[0], ip[1], ip[2], ip[3], port);
        
        gprs_cb          = config->type_cb;
        gprs_get_data_cb = config->data_cb;
        OSEL_ISR_EXIT();
    }
    return TRUE;
}

uint8_t gprs_send(void *data_p, uint8_t len, uint16_t tag, uint16_t time)
{
    DBG_ASSERT(data_p != NULL __DBG_LINE);
    DBG_ASSERT(len != 0 __DBG_LINE);
    
    queue_data_type new_item;
    osel_memset(&new_item, 0x00, sizeof new_item);
    
    OSEL_ISR_ENTRY();
    osel_memcpy(new_item.gprs_data, data_p, len);
    new_item.gprs_data_len  = len;
    new_item.gprs_tag       = tag; 
    new_item.ack_time       = time;
    OSEL_ISR_EXIT();
    
    if(GET_QUEUE_LEN == QUEUE_MAXLEN)
    {
        return QUEUE_MAXLEN+1;
    }
    else
    {
        queue_send(&gprs_queue, new_item);      //入队成功
        osel_post(GPRS_SEND_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
        return(GET_QUEUE_LEN);                 //返回队列长度
    }
}

void gprs_set_mode(gprs_mode_t sig)
{
    gs_mode = sig;
}

static void sim928a_gprs_init(void)
{
#if GPRS_TEST
    P6SEL &=~BIT7;                              //EN_6130 power
    P6DIR |= BIT7;
    P6OUT |= BIT7;
#endif
    
    P2SEL &= ~BIT1;
    P2DIR |=  BIT1;
}

static void prwkey_up(void)                   //PRWKEY拉高
{
    P2OUT |= BIT1;
}

static void prwkey_down(void)
{
    P2OUT &= ~BIT1;                            //PRWKEY拉低  
}

static void gprs_switch(void)                //拉低大于1S触发
{
    prwkey_down();
    delay_ms(1000);
    prwkey_up();
}

static void gprs_restart(void)
{
    gprs_switch(); 
    delay_ms(800);
    gprs_switch();
}

static void gprs_cmd_cipstart(void)
{
    uint8_t cmd[0x30];
    osel_memset(cmd, 0x00, sizeof(cmd));

    wsnos_sprintf((char *)cmd, 
                  "AT+CIPSTART=%s,%s\r", 
                  (char *)ip_mode, 
                  (char *)ip_config);
    
    gprs_send_cmd(cmd, mystrlen((char *)cmd));
}

static void gprs_presend_cmd(void)      //"AT+CIPSEND=LEN\R"
{                
    uint8_t len;
    uint8_t cmd[0x30];
    osel_memset(cmd, 0x00, sizeof(cmd));
    
    if(PEEK_QUEUE_FIRST_DATA)            //看队首元素的长度并不出队
    {
        len = n_item.gprs_data_len;
        wsnos_sprintf((char *)cmd, "AT+CIPSEND=%d\r", len); //AT+CIPSEND=LEN\r
        gprs_send_cmd(cmd, mystrlen((char *)cmd));
    }
    else                                //队列为空
    {
        osel_post(GPRS_POLL_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
    }
}

static void gprs_uart_init(void)
{
    hal_uart_init(HAL_UART_4, 115200, 0);
    
#if 0    
    serial_reg_t gprs_serial_reg;
    gprs_serial_reg.sd.valid = FALSE;         // 起始域是否有效
    gprs_serial_reg.sd.len = 2;               // 起始域的长度
    gprs_serial_reg.sd.pos = 0;               // 起始域的偏移
    gprs_serial_reg.sd.data[0] = 'A';         // 起始域的值
    gprs_serial_reg.sd.data[1] = 'T';

    gprs_serial_reg.ld.valid = TRUE;          // 长度域是否有效
    gprs_serial_reg.argu.len_max = GPRS_SERIAL_LEN_MAX;  // 整条帧的长度限制
    gprs_serial_reg.argu.len_min = GPRS_SERIAL_LEN_MIN;

    gprs_serial_reg.ed.valid = FALSE;         // 结尾域是否存在
    gprs_serial_reg.ed.len = 1;
    gprs_serial_reg.ed.data[0] = 0x0D;        // 'CR', enter

    gprs_serial_reg.echo_en = FALSE;          // 回显
    // 不需要回调函数，定时器时间，判断是否收到数据
    gprs_serial_reg.func_ptr = NULL;          

    serial_fsm_init(SERIAL_4);                 // 初始化serial
    serial_reg(SERIAL_4, gprs_serial_reg);     // 注册serial
#endif
}

static void gprs_uart_tx(uint8_t *pData, uint16_t len)
{
    DBG_ASSERT(pData != NULL __DBG_LINE);
    DBG_ASSERT(len != 0 __DBG_LINE);
    
#if GPRS_DEBUG
    serial_write(HAL_UART_2, pData, len);            
#endif  
    serial_write(HAL_UART_4, pData, len);
}

//向gprs发送AT指令和数据
static void gprs_send_cmd(uint8_t *cmd, uint8_t len)
{
    DBG_ASSERT(cmd != NULL __DBG_LINE);
    DBG_ASSERT(len != 0 __DBG_LINE);
    
    if(gprs_cmd_send_cnt > GPRS_SEND_CMD_MAX_CNT)
    {
        gprs_cmd_send_cnt = 0;
        gprs_restart_handler();  //发了3次还是没有回应认为GPRS死机重启GPRS模块
    }
    else
    {
        gprs_cmd_send_cnt++;
        gprs_uart_tx(cmd, len);
    }
}

static void gprs_task_timer_cb(void *p)
{
    if (NULL != gprs_task_timer)
    {
        hal_timer_cancel(&gprs_task_timer);
        DBG_ASSERT(gprs_task_timer == NULL __DBG_LINE);
    }
    
    if(GPRS_QUEUE_HAVE_DATA)                //发送缓冲不为空
    {
        if(gprs_mode == GPRS_CONNECTED)     //已经连接就直接发送
        {
            osel_post(GPRS_SEND_CMD_EVENT, 
                      (uint8_t *)(GPRS_CMD_SEND), 
                      OSEL_EVENT_PRIO_LOW);
        }
        else if(gprs_mode == GPRS_CLOSE)       
        {
            gprs_open_handler();
            
        }
        else                                   
        {
            osel_post(GPRS_SEND_CMD_EVENT, 
                      (uint8_t *)(GPRS_CMD_STATUS), 
                      OSEL_EVENT_PRIO_LOW); 
        }
    }
    else                                   //没有要发送的数据
    {
        osel_post(GPRS_POLL_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
    }
}

static void gprs_open_timer_cb(void *p)
{
    if(NULL != gprs_open_timer)
    {
        hal_timer_cancel(&gprs_open_timer);
        DBG_ASSERT(gprs_open_timer == NULL __DBG_LINE);
    }
    
    osel_post(GPRS_SEND_CMD_EVENT, 
              (uint8_t *)(GPRS_CMD_AT), 
              OSEL_EVENT_PRIO_LOW);
}

static void gprs_send_event_handler(void)  //APP新插入数据直接发送
{
    if (NULL != gprs_task_timer)         
    {
        hal_timer_cancel(&gprs_task_timer);
        DBG_ASSERT(gprs_task_timer == NULL __DBG_LINE);
    }
    
    if(gprs_mode == GPRS_CONNECTED)     //已经连接就直接发送
    {
        osel_post(GPRS_SEND_CMD_EVENT, 
                  (uint8_t *)(GPRS_CMD_SEND), 
                  OSEL_EVENT_PRIO_LOW);
    }
    else if(gprs_mode == GPRS_CLOSE)       
    {
        gprs_open_handler();
        
    }
    else                              //其他状态查询连接状态
    {
       osel_post(GPRS_SEND_CMD_EVENT, 
                  (uint8_t *)(GPRS_CMD_STATUS), 
                  OSEL_EVENT_PRIO_LOW); 
    }
}

static void gprs_poll_event_handler(void)
{
    if (NULL != gprs_task_timer)
    {
        hal_timer_cancel(&gprs_task_timer);
        DBG_ASSERT(gprs_task_timer == NULL __DBG_LINE);
    }
 
    HAL_TIMER_SET_REL(MS_TO_TICK(GPRS_POLL_TIME),   //2s
                      gprs_task_timer_cb,
                      NULL,
                      gprs_task_timer);
    DBG_ASSERT(gprs_task_timer != NULL __DBG_LINE); 
   
}

static void gprs_response_cb(void *p)    //定时器时间到，读缓冲区里数据
{
    if(gprs_response_timer != NULL)
    {
        hal_timer_cancel(&gprs_response_timer); 
        DBG_ASSERT(gprs_response_timer == NULL __DBG_LINE);
    }
    
    osel_post(GPRS_RESPONSE_EVENT, NULL, OSEL_EVENT_PRIO_LOW);
}
    
static void gprs_send_data(void)
{
    gprs_send_data_cnt++;
    if(PEEK_QUEUE_FIRST_DATA)
    {
        gprs_cmd_type = GPRS_SEND_DATA;
        gprs_send_cmd(n_item.gprs_data, n_item.gprs_data_len);  
    }
    else         
    {
        (* gprs_cb)(GS_NO_DATA_SEND, n_item.gprs_tag);
    }
}

static void gprs_response_event_handler(void)
{
#if GPRS_DEBUG
    serial_write(HAL_UART_2, 
                 gprs_cmd_recv_array, 
                 mystrlen((char *)gprs_cmd_recv_array));
#endif 
    
    if(gprs_cmd_recv_pos == 0)                //缓冲区里没有数据
    {
        //发送数据如果发送不出去也会无数据返回
        if(gprs_cmd_type == GPRS_SEND_DATA)
        {
            uint16_t tag;
            gprs_flush_receive_buf();
            if(gprs_send_data_cnt < 3)
            {
                osel_post(GPRS_SEND_CMD_EVENT, 
                          (uint8_t *)(GPRS_SEND_DATA), 
                          OSEL_EVENT_PRIO_LOW);
            }
            else
            {
                //发送3次数据失败，给APP层发送失败信息并扔掉该数据，查询连接状态
                if(GET_DATA_FROM_QUEUE) 
                {
                    gprs_send_data_cnt = 0;
                    tag = n_item.gprs_tag;
                    (* gprs_cb)(GS_SEND_FAIL, tag);   
                }
                osel_post(GPRS_SEND_CMD_EVENT, 
                          (uint8_t *)(GPRS_CMD_STATUS), 
                          OSEL_EVENT_PRIO_LOW);
            }
        }
        else                            
        {
            gprs_restart_handler();
        }
    }
    else          //缓冲区里有数据进行处理
    {
        gprs_cmd_send_cnt = 0;
        gprs_response_event_parse();
    }
}

static void gprs_send_cmd_event_handler(uint8_t type)
{
    switch (type)
    {
    case GPRS_CMD_AT:
        gprs_cmd_type = GPRS_CMD_AT;
        
        if(gprs_response_timer == NULL)
        {
            HAL_TIMER_SET_REL(MS_TO_TICK(GPRS_RESPONSE_TIME_1S),   
                              gprs_response_cb,
                              NULL,
                              gprs_response_timer);
            DBG_ASSERT(gprs_response_timer != NULL __DBG_LINE); 
        }
        
        gprs_send_cmd(GPRS_AT, sizeof(GPRS_AT) - 1);
        break;
        
    case GPRS_CMD_ATE0:
        gprs_cmd_type = GPRS_CMD_ATE0;
        
        if(gprs_response_timer == NULL)
        {
            //因为发AT是GPRS刚刚启动要等待时间长点
            HAL_TIMER_SET_REL(MS_TO_TICK(GPRS_RESPONSE_TIME_1S),   
                              gprs_response_cb,
                              NULL,
                              gprs_response_timer);
            DBG_ASSERT(gprs_response_timer != NULL __DBG_LINE); 
        }
        
        gprs_send_cmd(GPRS_ATE0, sizeof(GPRS_ATE0) - 1);
        break;  
        
    case GPRS_CMD_CGATT:
        gprs_cmd_type = GPRS_CMD_CGATT;
        
        if(gprs_response_timer == NULL)
        {
            HAL_TIMER_SET_REL(MS_TO_TICK(GPRS_RESPONSE_TIME_2S),   
                              gprs_response_cb,
                              NULL,
                              gprs_response_timer);
            DBG_ASSERT(gprs_response_timer != NULL __DBG_LINE); 
        }
        
        gprs_send_cmd(GPRS_CGATT, sizeof(GPRS_CGATT) - 1); 
        break;
        
    case GPRS_CMD_CIPSTART:
        gprs_cmd_type = GPRS_CMD_CIPSTART;
        
        if(gprs_response_timer == NULL)
        {
            HAL_TIMER_SET_REL(MS_TO_TICK(GPRS_RESPONSE_TIME_5S),   
                              gprs_response_cb,
                              NULL,
                              gprs_response_timer);
            DBG_ASSERT(gprs_response_timer != NULL __DBG_LINE); 
        }
        
        gprs_cmd_cipstart();
        break;
        
    case GPRS_CMD_SEND:
        gprs_cmd_type = GPRS_CMD_SEND;
        
        if(gprs_response_timer == NULL)
        {
            HAL_TIMER_SET_REL(MS_TO_TICK(GPRS_RESPONSE_TIME_500MS),   
                              gprs_response_cb,
                              NULL,
                              gprs_response_timer);
            DBG_ASSERT(gprs_response_timer != NULL __DBG_LINE); 
        }  
        
        gprs_presend_cmd();       //"AT+CIPSEND=len\r"
        break;
        
    case GPRS_SEND_DATA:
        gprs_cmd_type = GPRS_SEND_DATA;
        
        if(gprs_response_timer == NULL)
        {
            HAL_TIMER_SET_REL(MS_TO_TICK(GPRS_RESPONSE_TIME_1S),   
                              gprs_response_cb,
                              NULL,
                              gprs_response_timer);
            DBG_ASSERT(gprs_response_timer != NULL __DBG_LINE); 
        } 
        
        gprs_send_data();        //发送数据
        break;
        
    case GPRS_CMD_STATUS:
        gprs_cmd_type = GPRS_CMD_STATUS;
        
        if(gprs_response_timer == NULL)
        {
            HAL_TIMER_SET_REL(MS_TO_TICK(GPRS_RESPONSE_TIME_1S),   
                              gprs_response_cb,
                              NULL,
                              gprs_response_timer);
            DBG_ASSERT(gprs_response_timer != NULL __DBG_LINE); 
        }        
        
        gprs_send_cmd(GPRS_CIPSTATUS, 0x0D);
        break;
        
    case GPRS_CMD_CPIN:
        gprs_cmd_type = GPRS_CMD_CPIN;
        
        if(gprs_response_timer == NULL)
        {
            HAL_TIMER_SET_REL(MS_TO_TICK(GPRS_RESPONSE_TIME_2S),   
                              gprs_response_cb,
                              NULL,
                              gprs_response_timer);
            DBG_ASSERT(gprs_response_timer != NULL __DBG_LINE); 
        } 
        
        gprs_send_cmd(GPRS_CPIN, sizeof(GPRS_CPIN) - 1);
        break;
        
    case GPRS_CMD_CIPSHUT:
        gprs_cmd_type = GPRS_CMD_CIPSHUT;
        
        if(gprs_response_timer == NULL)
        {
            HAL_TIMER_SET_REL(MS_TO_TICK(GPRS_RESPONSE_TIME_2S),  
                              gprs_response_cb,
                              NULL,
                              gprs_response_timer);
            DBG_ASSERT(gprs_response_timer != NULL __DBG_LINE); 
        } 
        
        gprs_send_cmd(GPRS_CIPSHUT, sizeof(GPRS_CIPSHUT) - 1);
        break;
        
    case GPRS_CMD_CIPCLOSE:
        gprs_cmd_type = GPRS_CMD_CIPCLOSE;
        
        if(gprs_response_timer == NULL)
        {
            HAL_TIMER_SET_REL(MS_TO_TICK(GPRS_RESPONSE_TIME_2S),   
                              gprs_response_cb,
                              NULL,
                              gprs_response_timer);
            DBG_ASSERT(gprs_response_timer != NULL __DBG_LINE); 
        } 
        
        gprs_send_cmd(GPRS_CIPCLOSE, sizeof(GPRS_CIPCLOSE) - 1);
        break;
        
    default:
        break;          
    }
}
