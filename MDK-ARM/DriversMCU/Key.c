#include "Key.h"

#include "stdbool.h"

#define DEBOUNCE_DELAY_MS 20


/* 按键信息结构体 */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    bool active_level;       // 有效电平
    uint32_t last_check_time;
    bool last_state;         // 上次IO状态
    bool stable_state;       // 消抖后稳定状态
    bool prev_stable_state;  // 前次稳定状态（用于边沿检测）
    Key_Callback_t callback; // 按键回调函数
} Key_Info_t;

#define KEY_GPIO_Port GPIOA
/* 按键配置数组（根据实际硬件修改） */
static Key_Info_t keys[] = {
    {KEY_GPIO_Port, GPIO_PIN_5, false,  0, false, false, false, NULL},
    {KEY_GPIO_Port, GPIO_PIN_6, true,  0, false, false, false, NULL},
    {KEY_GPIO_Port, GPIO_PIN_7, true,  0, false, false, false, NULL},
    {KEY_GPIO_Port, GPIO_PIN_4, true, 0, false, false, false, NULL}
};



/* 注册按键回调函数 */
void RegisterKeyCallback(uint8_t key_index, Key_Callback_t callback) {
    if (key_index < sizeof(keys)/sizeof(keys[0])) {
        keys[key_index].callback = callback;
    }
}




Key_Status_t GetKeyStatus(void) {
    Key_Status_t result = {0};
    uint32_t current_tick = HAL_GetTick();

    for (uint8_t i = 0; i < sizeof(keys)/sizeof(keys[0]); i++) {
        /* 读取当前IO状态 */
        bool current_io = HAL_GPIO_ReadPin(keys[i].port, keys[i].pin) == 
                         (keys[i].active_level ? GPIO_PIN_SET : GPIO_PIN_RESET);

        /* 状态变化检测 */
        if (current_io != keys[i].last_state) {
            keys[i].last_state = current_io;
            keys[i].last_check_time = current_tick;
        }
        /* 稳定状态更新 */
        else if (current_tick - keys[i].last_check_time >= DEBOUNCE_DELAY_MS) {
            keys[i].stable_state = (bool)current_io;
        }

        /* 检测上升沿（按下事件） */
        if (keys[i].stable_state && !keys[i].prev_stable_state) {
            if (keys[i].callback != NULL) {
                keys[i].callback(); // 触发回调
            }
        }
        keys[i].prev_stable_state = keys[i].stable_state;

        /* 更新返回结果 */
        switch(i) {
             case 0: result.key_up_pressed = keys[i].stable_state; break;
             case 1: result.key_down_pressed = keys[i].stable_state; break;
             case 2: result.key_enter_pressed = keys[i].stable_state; break;
             case 3: result.key_back_pressed = keys[i].stable_state; break;
        }
    }

    return result;
}



