#include "Drv_Uart.h"
#include <stdio.h>
#include <string.h>
#include "stm32h743xx.h"
#include "SysConfig.h"


void NoUse(u8 data){}
#define U1GetOneByte	NoUse
#define U2GetOneByte	NoUse
#define U3GetOneByte	NoUse
#define U4GetOneByte	NoUse      //ANO_TO_H743_Data_Transmit
#define U5GetOneByte	NoUse
#define U6GetOneByte    NoUse

//====uart1
/* 告知连接器不从C库链接使用半主机的函数 */
#pragma import(__use_no_semihosting)
 
/* 定义 _sys_exit() 以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}
 
/* 标准库需要的支持类型 */
struct __FILE
{
    int handle;
};
 
FILE __stdout;
 
/*  */
int fputc(int ch, FILE *stream)
{
    /* 堵塞判断串口是否发送完成 */
    while((USART1->ISR & 0X40) == 0);
 
    /* 串口发送完成，将该字符发送 */
    USART1->TDR = (uint8_t) ch;
 
    return ch;
}


