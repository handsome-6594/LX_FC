#include "To_LX_Fun.h"

volatile u32 lx_mode_cmd_ok_cnt = 0;
volatile u32 lx_mode_cmd_busy_cnt = 0;
volatile u8 lx_mode_cmd_target = 0;

u8 CMD_Send(u8 goal_addr, Cmd_data *cmd)
{
    if(cmd == 0)
    {
        return 0;
    }

    if(Data.cmd_wait_ack != 0)
    {
        return 0;
    }

    Data.data_of_cmd = *cmd;
    Data.fun[0xE0].Addr = goal_addr;
    Data.fun[0xE0].wait_to_send = 1;
    Data.cmd_wait_ack = 1;

    return 1;
}

u8 FC_Unlock(void)
{
    Cmd_data cmd;

    cmd.CID = 0x10;
    for(u8 i = 0; i < 10; i++)
    {
        cmd.CMD[i] = 0;
    }

    cmd.CMD[0] = 0x00;
    cmd.CMD[1] = 0x01;

    return CMD_Send(0xFF, &cmd);
}

u8 LX_Change_Mode(u8 new_mode)
{
    static u8 old_mode = 0;
    Cmd_data cmd;

    if(new_mode < 1 || new_mode > 3)
    {
        return 0;
    }

    if(old_mode == new_mode)
    {
        return 1;
    }

    cmd.CID = 0x01;
    for(u8 i = 0; i < 10; i++)
    {
        cmd.CMD[i] = 0;
    }

    cmd.CMD[0] = 0x01;
    cmd.CMD[1] = 0x01;
    cmd.CMD[2] = new_mode;

    if(CMD_Send(0xFF, &cmd))
    {
        old_mode = new_mode;
        lx_mode_cmd_target = new_mode;
        lx_mode_cmd_ok_cnt++;
        return 1;
    }

    lx_mode_cmd_busy_cnt++;
    return 0;
}
