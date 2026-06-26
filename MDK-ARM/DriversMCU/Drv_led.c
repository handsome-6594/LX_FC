#include "Drv_led.h"

led_RGB Led_LX;

void DvrLedInit()
{
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
}

void LED_On_Off(uint16_t leds)
{
	if (leds & LED_R_BIT)
	{
		LED1_ON;
	}
	else
	{
		LED1_OFF;
	}
	if (leds & LED_G_BIT)
	{
		LED2_ON;
	}
	else
	{
		LED2_OFF;
	}
	if (leds & LED_B_BIT)
	{
		LED3_ON;
	}
	else
	{
		LED3_OFF;
	}
}

