#ifndef _RADAR_POSITION_H
#define _RADAR_POSITION_H

#include "SysConfig.h"
#include "JetsonNano_Data_Transmit.h"

typedef struct 
{
    s16 init_pos_x_x100;
    s16 init_pos_y_x100;
    s16 init_pos_z_x100;
    u8 init_flag;
    
}radar_pos_init;

extern radar_pos_init radar_pos_init_val;

Radar_Pos_16_un RadarPos_ToRelative(Radar_Pos_16_un raw_pos);
void RadarPos_SetInitPoint(Radar_Pos_16_un raw_pos);


#endif


