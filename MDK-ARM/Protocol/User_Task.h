#ifndef USER_TASK_H
#define USER_TASK_H

#include "SysConfig.h"

#define USER_TASK_ENABLE (1U)
#define USER_TASK_GROUND_TEST_ENABLE (0U)

void UserTask_Init(void);
void UserTask_Update(void);

#endif
