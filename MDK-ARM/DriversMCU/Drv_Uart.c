#include "Drv_Uart.h"
#include <stdio.h>
#include <string.h>
#include "stm32h743xx.h"
#include "SysConfig.h"
#include "usart.h"
#include "fifo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os2.h"
#include "ANO_TO_H743_Data_Transmit.h"


void NoUse(u8 data){}
#define U1GetOneByte	NoUse
#define U2GetOneByte	NoUse
#define U3GetOneByte	NoUse
#define U4GetOneByte	H743_Data_Receive      //ANO_TO_H743_Data_Transmit
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

//uart4   H743通过凌霄IMU接收或发送数据
#define RX_Buffer_Size 15

//====uart4 //连接凌霄IMU
#define LX_RXBufferSize 15
#define LX_RXFIFOBufferSize (LX_RXBufferSize * 40)

uint8_t LX_RxBuffer[LX_RXBufferSize];
uint8_t LX_RxFIFOBuffer[LX_RXFIFOBufferSize];
FIFO_Type LX_fifo;
FIFO_Type *pLXfifo = NULL;

extern DMA_HandleTypeDef hdma_uart4_rx;
extern osSemaphoreId_t LX_UART_Binary_SemaphoreHandle;

void DrvUart4_Receive_Enable(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, LX_RxBuffer, LX_RXBufferSize);
    __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
}

void DrvUart4_Fifo_Init(void)
{
    pLXfifo = &LX_fifo;
    FIFO_Init(pLXfifo, LX_RxFIFOBuffer, sizeof(uint8_t), LX_RXFIFOBufferSize);
}

void DrvUart4_RxEventCallback(uint16_t size)
{
    if(pLXfifo != NULL && size <= LX_RXBufferSize)
    {
        FIFO_Add(pLXfifo, LX_RxBuffer, size);
    }

    DrvUart4_Receive_Enable();
}

void DrvUart4_TxCpltCallback(void)
{
    BaseType_t highTaskWoken = pdFALSE;

    if(LX_UART_Binary_SemaphoreHandle != NULL)
    {
        xSemaphoreGiveFromISR(LX_UART_Binary_SemaphoreHandle, &highTaskWoken);
        portYIELD_FROM_ISR(highTaskWoken);
    }
}

void DrvUart4_ErrorCallback(void)
{
    DrvUart4_Receive_Enable();
}

void drvU4DataCheck(void)
{
    uint8_t data_temp;
    uint8_t has_data;

    if(pLXfifo == NULL)
    {
        return;
    }

    for(;;)
    {
        taskENTER_CRITICAL();
        has_data = FIFO_GetOne(pLXfifo, &data_temp);
        taskEXIT_CRITICAL();

        if(!has_data)
        {
            break;
        }

        U4GetOneByte(data_temp);
    }
}

void DrvUart4SendBuf(unsigned char *DataToSend, uint8_t data_num)
{
    if(DataToSend == NULL || data_num == 0)
    {
        return;
    }

    if(HAL_UART_Transmit_DMA(&huart4, DataToSend, data_num) == HAL_OK)
    {
        if(LX_UART_Binary_SemaphoreHandle != NULL)
        {
            xSemaphoreTake(LX_UART_Binary_SemaphoreHandle, portMAX_DELAY);
        }
    }
}




