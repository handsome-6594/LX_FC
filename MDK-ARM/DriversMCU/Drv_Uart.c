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

static void DrvUart_NoUse(uint8_t data)
{
    (void)data;
}

static DrvUartByteHandler Uart2RxByteHandler = DrvUart_NoUse;
static DrvUartByteHandler Uart3RxByteHandler = DrvUart_NoUse;
static DrvUartByteHandler Uart4RxByteHandler = DrvUart_NoUse;
static TaskHandle_t Uart2NotifyTaskHandle = NULL;
static TaskHandle_t Uart3NotifyTaskHandle = NULL;

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
    (void)stream;
    /* 堵塞判断串口是否发送完成 */
    while((USART1->ISR & 0X40) == 0);
 
    /* 串口发送完成，将该字符发送 */
    USART1->TDR = (uint8_t) ch;
 
    return ch;
}

//====uart2 optical flow
#define OF_RXBufferSize 5
#define OF_RXFIFOBufferSize (OF_RXBufferSize * 30)

uint8_t OF_RxBuffer[OF_RXBufferSize];
uint8_t OF_RxFIFOBuffer[OF_RXFIFOBufferSize];
FIFO_Type OF_fifo;
FIFO_Type *pOFfifo = NULL;

extern DMA_HandleTypeDef hdma_usart2_rx;

void DrvUart2_Receive_Enable(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, OF_RxBuffer, OF_RXBufferSize);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

void DrvUart2_Fifo_Init(void)
{
    pOFfifo = &OF_fifo;
    FIFO_Init(pOFfifo, OF_RxFIFOBuffer, sizeof(uint8_t), OF_RXFIFOBufferSize);
}

void DrvUart2_RegisterRxByteHandler(DrvUartByteHandler handler)
{
    Uart2RxByteHandler = (handler != NULL) ? handler : DrvUart_NoUse;
}

void DrvUart2_RegisterNotifyTask(TaskHandle_t task_handle)
{
    Uart2NotifyTaskHandle = task_handle;
}

void DrvUart2_RxEventCallback(uint16_t size)
{
    BaseType_t highTaskWoken = pdFALSE;

    if(pOFfifo != NULL && size <= OF_RXBufferSize)
    {
        FIFO_Add(pOFfifo, OF_RxBuffer, size);
    }

    DrvUart2_Receive_Enable();

    if(Uart2NotifyTaskHandle != NULL)
    {
        vTaskNotifyGiveFromISR(Uart2NotifyTaskHandle, &highTaskWoken);
        portYIELD_FROM_ISR(highTaskWoken);
    }
}

void DrvUart2_ErrorCallback(void)
{
    BaseType_t highTaskWoken = pdFALSE;

    DrvUart2_Receive_Enable();

    if(Uart2NotifyTaskHandle != NULL)
    {
        vTaskNotifyGiveFromISR(Uart2NotifyTaskHandle, &highTaskWoken);
        portYIELD_FROM_ISR(highTaskWoken);
    }
}

void drvU2DataCheck(void)
{
    uint8_t data_temp;
    uint8_t has_data;

    if(pOFfifo == NULL)
    {
        return;
    }

    for(;;)
    {
        taskENTER_CRITICAL();
        has_data = FIFO_GetOne(pOFfifo, &data_temp);
        taskEXIT_CRITICAL();

        if(!has_data)
        {
            break;
        }

        Uart2RxByteHandler(data_temp);
    }
}

void DrvUart2SendBuf(unsigned char *DataToSend, uint8_t data_num)
{
    if(DataToSend == NULL || data_num == 0)
    {
        return;
    }

    HAL_UART_Transmit(&huart2, DataToSend, data_num, 0xffff);
}

//====usart3 JetsonNano
#define JN_RXBufferSize 64
#define JN_RXFIFOBufferSize (JN_RXBufferSize * 16)

uint8_t JN_RxBuffer[JN_RXBufferSize];
uint8_t JN_RxFIFOBuffer[JN_RXFIFOBufferSize];
FIFO_Type JN_fifo;
FIFO_Type *pJNfifo = NULL;

extern DMA_HandleTypeDef hdma_usart3_rx;

void DrvUart3_Receive_Enable(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, JN_RxBuffer, JN_RXBufferSize);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

void DrvUart3_Fifo_Init(void)
{
    pJNfifo = &JN_fifo;
    FIFO_Init(pJNfifo, JN_RxFIFOBuffer, sizeof(uint8_t), JN_RXFIFOBufferSize);
}

void DrvUart3_RegisterRxByteHandler(DrvUartByteHandler handler)
{
    Uart3RxByteHandler = (handler != NULL) ? handler : DrvUart_NoUse;
}

void DrvUart3_RegisterNotifyTask(TaskHandle_t task_handle)
{
    Uart3NotifyTaskHandle = task_handle;
}

//串口空闲中断触发
void DrvUart3_RxEventCallback(uint16_t size)
{
    BaseType_t highTaskWoken = pdFALSE;

    if(pJNfifo != NULL && size <= JN_RXBufferSize)
    {
        FIFO_Add(pJNfifo, JN_RxBuffer, size);
    }

    DrvUart3_Receive_Enable();

    if(Uart3NotifyTaskHandle != NULL)
    {
        vTaskNotifyGiveFromISR(Uart3NotifyTaskHandle, &highTaskWoken);
        portYIELD_FROM_ISR(highTaskWoken);
    }
}

void DrvUart3_ErrorCallback(void)
{
    BaseType_t highTaskWoken = pdFALSE;

    DrvUart3_Receive_Enable();

    if(Uart3NotifyTaskHandle != NULL)
    {
        vTaskNotifyGiveFromISR(Uart3NotifyTaskHandle, &highTaskWoken);
        portYIELD_FROM_ISR(highTaskWoken);
    }
}

void drvU3DataCheck(void)
{
    uint8_t data_temp;
    uint8_t has_data;

    if(pJNfifo == NULL)
    {
        return;
    }

    for(;;)
    {
        taskENTER_CRITICAL();
        has_data = FIFO_GetOne(pJNfifo, &data_temp);
        taskEXIT_CRITICAL();

        if(!has_data)
        {
            break;
        }

        // printf("%02X ", data_temp);
        Uart3RxByteHandler(data_temp);
    }
}

u8 DrvUart3SendBuf(unsigned char *DataToSend, uint8_t data_num)
{
    if(DataToSend == NULL || data_num == 0)
    {
        return 0;
    }

    return (HAL_UART_Transmit(&huart3, DataToSend, data_num, 0xffff) == HAL_OK) ? 1 : 0;
}

//uart4   H743通过凌霄IMU接收或发送数据
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

void DrvUart4_RegisterRxByteHandler(DrvUartByteHandler handler)
{
    Uart4RxByteHandler = (handler != NULL) ? handler : DrvUart_NoUse;
}

void DrvUart4_RxEventCallback(uint16_t size)
{
    if(pLXfifo != NULL && size <= LX_RXBufferSize)
    {
        FIFO_Add(pLXfifo, LX_RxBuffer, size);
    }

    DrvUart4_Receive_Enable();
}

//H743通过串口4发送数据给凌霄IMU的发送完成回调函数   
//在这里释放释放信号量表示DMA搬运完成，并且UART4发送数据寄存器和移位寄存器空了
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

        Uart4RxByteHandler(data_temp);
    }
}
//通过串口4发送指定的数据，take完信号量表示发送完成，返回1
u8 DrvUart4SendBuf(unsigned char *DataToSend, uint8_t data_num)
{
    if(DataToSend == NULL || data_num == 0)
    {
        return 0;
    }

    if(HAL_UART_Transmit_DMA(&huart4, DataToSend, data_num) == HAL_OK)
    {
        if(LX_UART_Binary_SemaphoreHandle != NULL)
        {
            xSemaphoreTake(LX_UART_Binary_SemaphoreHandle, portMAX_DELAY);
        }

        return 1;
    }

    return 0;
}




