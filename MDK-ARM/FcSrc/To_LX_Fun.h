#ifndef _TO_LX_FUN_H
#define _TO_LX_FUN_H

#include "SysConfig.h"
#include "ANO_TO_H743_Data_Transmit.h"

u8 CMD_Send(u8 goal_addr, Cmd_data *cmd);
u8 FC_Unlock(void);

#endif
