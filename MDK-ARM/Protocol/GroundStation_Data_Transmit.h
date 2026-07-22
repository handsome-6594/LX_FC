#ifndef GROUNDSTATION_DATA_TRANSMIT_H
#define GROUNDSTATION_DATA_TRANSMIT_H

#include "main.h"
#include "ANO_TO_H743_Data_Transmit.h"
#include "Map_Update.h"
#include "JetsonNano_Data_Transmit.h"

typedef struct 
{
    Data_Frame fun[256];

    u8 wait_ack;

    Cmd_data cmd_send;
    check_ack ack_of_check;
    check_ack checksum_ok;    //期望收到的应答帧
    param par_data;

}GD_Data_Transmit;

typedef union 
{
    u8 byte_data[16];
    pwm_value st_data;
}pwm_value_un;

typedef struct 
{
    u16 Volt_x100;
    u16 Curr_x100;
    s32 ALT_FU;
    u8 Process;
}__attribute__((__packed__))Batt_Curr_Height_Process;

typedef union 
{
    Batt_Curr_Height_Process st_data;
    u8 byte_data[9];
}Batt_Curr_Height_Process_un;

typedef union 
{
    Radar_Speed st_data;
    u8 byte_data[6];
}__attribute__((__packed__))Vel_Fu_un;

extern GD_Data_Transmit GD_Data;
extern Point No_Fly_Zone[3];
extern Batt_Curr_Height_Process_un un_of_Batt_Height_Process;
extern Vel_Fu_un un_of_Vel_Fu;


#endif

