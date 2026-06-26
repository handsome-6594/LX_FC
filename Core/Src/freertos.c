/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Drv_led.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "Key.h"
#include "OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//osSemaphoreId_t OLED_i2c_Binary_SemaphoreHandle;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t keyTaskHandle;
const osThreadAttr_t keyTask_attributes = {
  .name = "keyTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

// /* Definitions for uartTestTask */
// osThreadId_t uartTestTaskHandle;
// const osThreadAttr_t uartTestTask_attributes = {
//   .name = "uartTestTask",
//   .stack_size = 256 * 4,
//   .priority = (osPriority_t) osPriorityNormal,
// };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */
void StartKeyTestTask(void *argument);
void StartOledTestTask(void *argument);
void StartOledTestTask(void *argument);
// void StartUartTestTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  //OLED_i2c_Binary_SemaphoreHandle = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  keyTaskHandle = osThreadNew(StartKeyTestTask, NULL, &keyTask_attributes);

  oledTaskHandle = osThreadNew(StartOledTestTask, NULL, &oledTask_attributes);
  /* creation of uartTestTask */
  // uartTestTaskHandle = osThreadNew(StartUartTestTask, NULL, &uartTestTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
void StartKeyTestTask(void *argument)
{
  uint8_t led_on = 0;
  uint8_t last_key = 0;

  for (;;)
  {
    Key_Status_t key = GetKeyStatus();

    if(key.key_back_pressed && !last_key)
    {
      led_on = !led_on;
      LED_On_Off(led_on ? LED_B_BIT : 0);
    }

    last_key = key.key_back_pressed;
    osDelay(10);
  }
}

/* USER CODE BEGIN Header_StartUartTestTask */
/**
  * @brief  Function implementing the uartTestTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUartTestTask */
// void StartUartTestTask(void *argument)
// {
//   /* USER CODE BEGIN StartUartTestTask */
//   uint32_t count = 0;
//   char msg[64];

//   /* Infinite loop */
//   for(;;)
//   {
//     count++;
//     int len = snprintf(msg, sizeof(msg), "USART1 alive: %lu\r\n", count);
//     HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)len, HAL_MAX_DELAY);
//     osDelay(1000);
//   }
//   /* USER CODE END StartUartTestTask */
// }

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartOledTestTask(void *argument)
{
  uint32_t count = 0;

  // 初始化 OLED
  OLED_Init();
  OLED_Clear();

  // 显示固定文本
  OLED_ShowString(0, 0, "STM32H743", OLED_8X16);
  OLED_ShowString(0, 16, "OLED Test", OLED_8X16);
  OLED_Update();

  for(;;)
  {
    count++;

    // 显示计数器
    OLED_ShowString(0, 32, "Count:", OLED_6X8);
    OLED_ShowNum(42, 32, count, 8, OLED_6X8);
    OLED_Update();

    osDelay(100);  // 每 100ms 更新一次
  }
}
/* USER CODE END Application */

