#ifndef DRV_UART_H
#define DRV_UART_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

typedef void (*DrvUartByteHandler)(uint8_t data);

void DrvUart2_Fifo_Init(void);
void DrvUart2_Receive_Enable(void);
void DrvUart2_RegisterRxByteHandler(DrvUartByteHandler handler);
void DrvUart2_RegisterNotifyTask(TaskHandle_t task_handle);
void DrvUart2_RxEventCallback(uint16_t size);
void DrvUart2_ErrorCallback(void);
void drvU2DataCheck(void);
void DrvUart2SendBuf(unsigned char *DataToSend, uint8_t data_num);

extern volatile uint32_t of_uart2_rx_event_cnt;
extern volatile uint32_t of_uart2_rx_byte_cnt;
extern volatile uint32_t of_uart2_receive_enable_cnt;
extern volatile uint32_t of_uart2_receive_enable_fail_cnt;
extern volatile uint32_t of_uart2_receive_enable_status;
extern volatile uint32_t of_uart2_error_cnt;
extern volatile uint32_t of_uart2_error_code;

void DrvUart3_Fifo_Init(void);
void DrvUart3_Receive_Enable(void);
void DrvUart3_RegisterRxByteHandler(DrvUartByteHandler handler);
void DrvUart3_RegisterNotifyTask(TaskHandle_t task_handle);
void DrvUart3_RxEventCallback(uint16_t size);
void DrvUart3_ErrorCallback(void);
void drvU3DataCheck(void);
u8 DrvUart3SendBuf(unsigned char *DataToSend, uint8_t data_num);

void DrvUart4_Fifo_Init(void);
void DrvUart4_Receive_Enable(void);
void DrvUart4_RegisterRxByteHandler(DrvUartByteHandler handler);
void DrvUart4_RxEventCallback(uint16_t size);
void DrvUart4_TxCpltCallback(void);
void DrvUart4_ErrorCallback(void);
void drvU4DataCheck(void);
u8 DrvUart4SendBuf(unsigned char *DataToSend, uint8_t data_num);


#endif
