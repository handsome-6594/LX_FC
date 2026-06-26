#ifndef _FC_STATE_H
#define _FC_STATE_H

#include "SysConfig.h"

typedef struct
{
    u8 CID;
    u8 Cmd_0;
    u8 Cmd_1;
}cmd_value;

//飞控状态
typedef struct
{
    u8 mode;
    u8 is_unlocked;  //1 解锁   0 锁定
    cmd_value cmd;

}fc_state;

extern fc_state state;

#endif