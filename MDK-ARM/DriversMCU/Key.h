#ifndef KEY_H
#define KEY_H

#include "main.h"
#include "stdbool.h"
/* 返回按键状态的结构体 */
typedef struct {
    bool key_up_pressed;
    bool key_down_pressed;
    bool key_enter_pressed;
    bool key_back_pressed;
} Key_Status_t;

Key_Status_t GetKeyStatus(void);

/* 按键事件回调函数类型定义 */
typedef void (*Key_Callback_t)(void);
void RegisterKeyCallback(uint8_t key_index, Key_Callback_t callback);

Key_Status_t GetKeyStatus(void);

#endif
