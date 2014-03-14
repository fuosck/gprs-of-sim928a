/**
  *ʵ��string�⺯���Ĳ��ֺ�������
  *
  *@file gprs_str.c
  *@author huangya
  *@data 2014-03-03
  *
  */

#include <debug.h>

/**
  *ʵ��strstr��������
  *
  *@param: �����ַ���
  *@return: ������str�г���sub_str֮����ַ�����
  *
  */
char const *my_strstr(const char *str, const char *sub_str)  
{
    DBG_ASSERT(str != NULL __DBG_LINE);
    DBG_ASSERT(sub_str != NULL __DBG_LINE);

    for(int i = 0; str[i] != '\0'; i++)  
    {  
        int tem = i;    
        int j = 0;  
        
        while(str[i++] == sub_str[j++])  
        {  
            if(sub_str[j] == '\0')  
            {  
                return &str[tem];  
            }  
        }  
        i = tem;  
    }  
    return NULL;  
} 

int mystrlen(const char *str)
{
    DBG_ASSERT(str != NULL __DBG_LINE);
    
    int len = 0;
    
    while ((*str++) != '\0')
    {
        len++;
    }
    
    return len;
}
