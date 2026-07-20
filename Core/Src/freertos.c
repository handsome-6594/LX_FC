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
#include <math.h>
#include "Key.h"
#include "OLED.h"
#include "ANO_TO_H743_Data_Transmit.h"
#include "Drv_Uart.h"
#include "Remote_Control.h"
#include "RC_Channel.h"
#include "PWM.h"
#include "FC_State.h"
#include "Optical_Flow_Sensor.h"
#include "Of_Radar_Fusion.h"
#include "Drv_adc.h"
#include "To_LX_Fun.h"
#include "JetsonNano_Data_Transmit.h"
#include "Point_Navigation.h"
#include "User_Task.h"
#include "Drv_Menu.h"
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
osSemaphoreId_t LX_UART_Binary_SemaphoreHandle = NULL;
/* USER CODE END Variables */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t pwmPrintTaskHandle;
const osThreadAttr_t pwmPrintTask_attributes = {
  .name = "pwmPrintTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t uart4LXTaskHandle;
const osThreadAttr_t uart4LXTask_attributes = {
  .name = "uart4LXTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t opticalFlowTaskHandle;
const osThreadAttr_t opticalFlowTask_attributes = {
  .name = "optFlowTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t jetsonRxTaskHandle;
const osThreadAttr_t jetsonRxTask_attributes = {
  .name = "Jetson_RX",
  .stack_size = 512 * 4,
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
void StartOledTestTask(void *argument);
void StartpwmPrintTask(void *argument);
void Startuart4LXTask(void *argument);
void StartOpticalFlowTask(void *argument);
void StartJetsonRxTask(void *argument);
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
  LX_UART_Binary_SemaphoreHandle = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  oledTaskHandle = osThreadNew(StartOledTestTask, NULL, &oledTask_attributes);

  pwmPrintTaskHandle = osThreadNew(StartpwmPrintTask, NULL, &pwmPrintTask_attributes);

  uart4LXTaskHandle = osThreadNew(Startuart4LXTask, NULL, &uart4LXTask_attributes);

  opticalFlowTaskHandle = osThreadNew(StartOpticalFlowTask, NULL, &opticalFlowTask_attributes);

  jetsonRxTaskHandle = osThreadNew(StartJetsonRxTask, NULL, &jetsonRxTask_attributes);
  /* creation of uartTestTask */
  // uartTestTaskHandle = osThreadNew(StartUartTestTask, NULL, &uartTestTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartUartTestTask */
/* USER CODE END Header_StartUartTestTask */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void StartOledTestTask(void *argument)
{
  (void)argument;

  OLED_Init();
  DrvMenu_Init();

  for(;;)
  {
    DrvMenu_Task();
    osDelay(20);
  }
}

void StartpwmPrintTask(void *argument)
{
  uint8_t last_lx_unlocked = 0;

  for(;;)
  {
    if(last_lx_unlocked && !state.is_unlocked)
    {
      RC_MotorForceLock();
    }
    last_lx_unlocked = state.is_unlocked;

    ESC_Output(RC_MotorIsUnlocked() && state.is_unlocked);


    osDelay(2);
  }
}

void Startuart4LXTask(void *argument)
{
  (void)argument;
  uint32_t last_bat_send_ms = 0;

  Data_Init();
  DrvRcInputInit();
  DrvESCInit();
  DrvAdcInit();
  DrvUart4_Fifo_Init();
  DrvUart4_Receive_Enable();
  PointNavigation_Init();
#if USER_TASK_ENABLE != 0U
  UserTask_Init();
#endif

  for(;;)
  {
    DrvRcInputTask(0.001f);
    RC_Data_Task(0.001f);
#if USER_TASK_ENABLE != 0U
    UserTask_Update();
#else
    PointNavigation_TestPointTask();
#endif
    PointNavigation_Update();
    drvU4DataCheck();
    H743_Data_Transmit_Check();

    if(HAL_GetTick() - last_bat_send_ms >= 100)
    {
      last_bat_send_ms = HAL_GetTick();
      union_of_bat.data_of_bat.voltage_100 = Drv_AdcGetBatVolt100();
      union_of_bat.data_of_bat.current_100 = Drv_AdcGetBatCurr100();
      Data.fun[0x0D].wait_to_send = 1;
    }


    osDelay(1);
  }
}

void StartOpticalFlowTask(void *argument)
{
  (void)argument;
  uint32_t last_check_ms = HAL_GetTick();

  OpticalFlow_Init();
  ExtSensorFusion_Init();
  DrvUart2_Fifo_Init();
  DrvUart2_RegisterNotifyTask(xTaskGetCurrentTaskHandle());// optical flow task notify
  DrvUart2_Receive_Enable();

  for(;;)
  {
    uint32_t now_ms;
    float dt_s;

    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    drvU2DataCheck();

    now_ms = HAL_GetTick();
    dt_s = (float)(now_ms - last_check_ms) * 0.001f;
    last_check_ms = now_ms;

    if(dt_s <= 0.0f)
    {
      dt_s = 0.001f;
    }

    OpticalFlow_CheckState(dt_s);
    ExtSensor_UpdateFromOpticalFlow(dt_s);
  }
}

void StartJetsonRxTask(void *argument)
{
  (void)argument;

  DrvUart3_Fifo_Init();
  DrvUart3_RegisterRxByteHandler(JetsonNano_To_H743_Data_Prepare);
  DrvUart3_RegisterNotifyTask(xTaskGetCurrentTaskHandle());
  DrvUart3_Receive_Enable();

  for(;;)
  {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
    drvU3DataCheck();
    JN_Data_Transmit_Check();
  }
}
/* USER CODE END Application */

