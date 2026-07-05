#include "To_LX_Fun.h"

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
