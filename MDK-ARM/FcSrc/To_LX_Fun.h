#ifndef _TO_LX_FUN_H
#define _TO_LX_FUN_H

#include "SysConfig.h"
#include "ANO_TO_H743_Data_Transmit.h"

extern volatile u32 lx_mode_cmd_ok_cnt;
extern volatile u32 lx_mode_cmd_busy_cnt;
extern volatile u8 lx_mode_cmd_target;

u8 CMD_Send(u8 goal_addr, Cmd_data *cmd);
u8 FC_Unlock(void);
u8 LX_Change_Mode(u8 new_mode);

#endif
