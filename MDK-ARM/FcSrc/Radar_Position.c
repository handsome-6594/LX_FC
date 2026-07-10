#include "Radar_Position.h"

radar_pos_init radar_pos_init_val = {0};

//转成相对坐标
Radar_Pos_16_un RadarPos_ToRelative(Radar_Pos_16_un raw_pos)
{
    Radar_Pos_16_un rel_pos = raw_pos;

    if(radar_pos_init_val.init_flag == 1)
    {
        rel_pos.pos_data.x_x100 -= radar_pos_init_val.init_pos_x_x100;
        rel_pos.pos_data.y_x100 -= radar_pos_init_val.init_pos_y_x100;
        rel_pos.pos_data.z_x100 -= radar_pos_init_val.init_pos_z_x100;
    }

    return rel_pos;
}

//设置初始化的坐标
void RadarPos_SetInitPoint(Radar_Pos_16_un raw_pos)
{
    radar_pos_init_val.init_pos_x_x100 = raw_pos.pos_data.x_x100;
    radar_pos_init_val.init_pos_y_x100 = raw_pos.pos_data.y_x100;
    radar_pos_init_val.init_pos_z_x100 = raw_pos.pos_data.z_x100;
    radar_pos_init_val.init_flag = 1;
}
