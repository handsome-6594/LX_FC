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
void StartKeyTestTask(void *argument);
void StartOledTestTask(void *argument);
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
  keyTaskHandle = osThreadNew(StartKeyTestTask, NULL, &keyTask_attributes);

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
// static void LX_GetEulerAngleX100(s16 *roll_x100, s16 *pitch_x100, s16 *yaw_x100)
// {
//   attitude_quat quat;
//   float qw;
//   float qx;
//   float qy;
//   float qz;
//   float sinp;
//   const float rad_to_deg_x100 = 5729.57795f;

//   taskENTER_CRITICAL();
//   quat = LX_quat;
//   taskEXIT_CRITICAL();

//   qw = (float)quat.quat_w_10000 * 0.0001f;
//   qx = (float)quat.quat_x_10000 * 0.0001f;
//   qy = (float)quat.quat_y_10000 * 0.0001f;
//   qz = (float)quat.quat_z_10000 * 0.0001f;

//   *roll_x100 = (s16)(atan2f(2.0f * (qw * qx + qy * qz),
//                             1.0f - 2.0f * (qx * qx + qy * qy)) * rad_to_deg_x100);

//   sinp = 2.0f * (qw * qy - qz * qx);
//   if(sinp > 1.0f)
//   {
//     sinp = 1.0f;
//   }
//   else if(sinp < -1.0f)
//   {
//     sinp = -1.0f;
//   }

//   *pitch_x100 = (s16)(asinf(sinp) * rad_to_deg_x100);
//   *yaw_x100 = (s16)(atan2f(2.0f * (qw * qz + qx * qy),
//                            1.0f - 2.0f * (qy * qy + qz * qz)) * rad_to_deg_x100);
// }

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

void StartpwmPrintTask(void *argument)
{
   pwm_value pwm = {0};
  // uint16_t print_cnt = 0;
   uint8_t has_new_pwm = 0;
  uint8_t last_lx_unlocked = 0;

  for(;;)
  {
    if(pwm_update_flag)
    {
      taskENTER_CRITICAL();
      pwm = pwm_to_esc;
      pwm_update_flag = 0;
      taskEXIT_CRITICAL();

      has_new_pwm = 1;
    }

    if(last_lx_unlocked && !state.is_unlocked)
    {
      RC_MotorForceLock();
    }
    last_lx_unlocked = state.is_unlocked;

    ESC_Output(RC_MotorIsUnlocked() && state.is_unlocked);

    // if(has_new_pwm && ++print_cnt >= 250)
    // {
    //   s16 lx_roll_x100;
    //   s16 lx_pitch_x100;
    //   s16 lx_yaw_x100;

    //   print_cnt = 0;
    //   LX_GetEulerAngleX100(&lx_roll_x100, &lx_pitch_x100, &lx_yaw_x100);
    //   printf("sbus byte=%lu frame=%lu lost=%d rc=%d lx=%d mode=%d ch=%d,%d,%d,%d tar=%d,%d,%d,%d pwm=%u,%u,%u,%u att=%d,%d,%d\r\n",
    //    sbus_dma_byte_cnt,
    //    sbus_frame_cnt,
    //    RemoteControl_IsSignalLost(),
    //    RC_MotorIsUnlocked(),
    //    state.is_unlocked,
    //    state.mode,
    //    Channel_of_rc.data.ch[ch_1_rol],
    //    Channel_of_rc.data.ch[ch_2_pit],
    //    Channel_of_rc.data.ch[ch_3_thr],
    //    Channel_of_rc.data.ch[ch_4_yaw],
    //    ctrl_of_realtime.data.roll,
    //    ctrl_of_realtime.data.pitch,
    //    ctrl_of_realtime.data.throttle,
    //    ctrl_of_realtime.data.yaw_dps,
    //    pwm.pwm_value1,
    //    pwm.pwm_value2,
    //    pwm.pwm_value3,
    //    pwm.pwm_value4,
    //    lx_roll_x100,
    //    lx_pitch_x100,
    //    lx_yaw_x100);
    // }

    osDelay(2);
  }
}

void Startuart4LXTask(void *argument)
{
  (void)argument;
  uint32_t last_swc_print_ms = 0;
  uint32_t last_bat_send_ms = 0;

  Data_Init();
  DrvRcInputInit();
  DrvESCInit();
  DrvAdcInit();
  DrvUart4_Fifo_Init();
  DrvUart4_Receive_Enable();
  PointNavigation_Init();

  for(;;)
  {
    DrvRcInputTask(0.001f);
    RC_Data_Task(0.001f);
    PointNavigation_TestPointTask();
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

    if(HAL_GetTick() - last_swc_print_ms >= 200)
    {
      last_swc_print_ms = HAL_GetTick();
// printf("mode=%d ch3=%d thr=%d alt_cm=%lu of=%d/%d/%d/%d pwm=%u,%u,%u,%u\r\n",
//        state.mode,
//        Channel_of_rc.data.ch[ch_3_thr],
//        ctrl_of_realtime.data.throttle,
//        optical_flow.alt_cm,
//        optical_flow.link_sta,
//        optical_flow.flow_sta,
//        optical_flow.alt_sta,
//        optical_flow.work_sta,
//        pwm_to_esc.pwm_value1,
//        pwm_to_esc.pwm_value2,
//        pwm_to_esc.pwm_value3,
//        pwm_to_esc.pwm_value4);
    }

    osDelay(1);
  }
}

void StartOpticalFlowTask(void *argument)
{
  (void)argument;
  uint32_t last_check_ms = HAL_GetTick();

  OpticalFlow_Init();
  DrvUart2_Fifo_Init();
  DrvUart2_RegisterNotifyTask(xTaskGetCurrentTaskHandle());//光流任务注册自己
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

