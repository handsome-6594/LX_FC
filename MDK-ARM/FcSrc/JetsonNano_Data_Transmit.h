#ifndef _JETSONNANO_DATA_TRANSMIT_H
#define _JETSONNANO_DATA_TRANSMIT_H

#include "SysConfig.h"


typedef struct  
{
    s32 POS_X_x100;
    s32 POS_Y_x100;
    s32 POS_Z_x100;
}__attribute__ ((__packed__))Radar_Pos;

typedef union 
{
    u8 byte_data[12];
    Radar_Pos pos_data;

}Radar_Pos_un;


extern Radar_Pos_un Pos_of_Radar;


#endif

