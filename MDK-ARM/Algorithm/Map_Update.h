#ifndef _MAP_UPDATE_H
#define _MAP_UPDATE_H

#include "SysConfig.h"
#include "JetsonNano_Data_Transmit.h"

typedef struct
{
    u8 x,y;
    
}Point;


void Update_Map_OnRawData(Camera_data_un Raw_Cam_un);
void Update_Now_Pos(void);


#endif
