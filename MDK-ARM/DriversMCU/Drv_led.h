#ifndef DRV_LED_H
#define DRV_LED_H

#include "SysConfig.h"
#include "main.h"

//
#define LED_R_BIT 0x01
#define LED_G_BIT 0x02
#define LED_B_BIT 0x04
//
#define LED_A_BIT 0X08
#define LED_ALL_BIT 0x0f


#define LED1_ON HAL_GPIO_WritePin(ANO_GPIO_LED, ANO_PIN_LED1, GPIO_PIN_SET)
#define LED1_OFF HAL_GPIO_WritePin(ANO_GPIO_LED, ANO_PIN_LED1, GPIO_PIN_RESET)
#define LED2_ON HAL_GPIO_WritePin(ANO_GPIO_LED, ANO_PIN_LED2, GPIO_PIN_SET)
#define LED2_OFF HAL_GPIO_WritePin(ANO_GPIO_LED, ANO_PIN_LED2, GPIO_PIN_RESET)
#define LED3_ON HAL_GPIO_WritePin(ANO_GPIO_LED, ANO_PIN_LED3, GPIO_PIN_SET)
#define LED3_OFF HAL_GPIO_WritePin(ANO_GPIO_LED, ANO_PIN_LED3, GPIO_PIN_RESET)

#define Lazer_OFF HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET)
#define Lazer_ON HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);



#define ANO_GPIO_LED GPIOD

//
#define ANO_PIN_LED1 GPIO_PIN_2 //r
#define ANO_PIN_LED2 GPIO_PIN_1 //g
#define ANO_PIN_LED3 GPIO_PIN_3 //b

#define LED_NUM 4

typedef struct {
	//
	u8 brightness[LED_NUM];
	
}led_RGB;

extern led_RGB Led_LX;

void DvrLedInit(void);
void LED_On_Off(uint16_t leds);


#endif

