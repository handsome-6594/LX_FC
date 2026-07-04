#ifndef DRV_UART_H
#define DRV_UART_H

#include "main.h"

void DrvUart4_Fifo_Init(void);
void DrvUart4_Receive_Enable(void);
void DrvUart4_RxEventCallback(uint16_t size);
void DrvUart4_TxCpltCallback(void);
void DrvUart4_ErrorCallback(void);
void drvU4DataCheck(void);
void DrvUart4SendBuf(unsigned char *DataToSend, uint8_t data_num);


#endif
