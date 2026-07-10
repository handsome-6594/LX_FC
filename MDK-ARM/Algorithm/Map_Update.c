#include "Map_Update.h"
#include "JetsonNano_Data_Transmit.h"

Point Now_Pos;

void Update_Now_Pos(void)
{
    Now_Pos.x = 8 - (Pos16_of_Radar.pos_data.y_x100 + 25) / 50;
    Now_Pos.y = (Pos16_of_Radar.pos_data.x_x100 + 25) / 50;
}



void Update_Map_OnRawData(Camera_data_un Raw_Cam_un)
{
    //将视觉识别的结果转换到实际的雷达坐标系
    Camera_data_un temp_cam_tar = Raw_Cam_un;
    temp_cam_tar.data.x_distance = - Raw_Cam_un.data.x_distance + Pos16_of_Radar.pos_data.x_x100 + 2.5;
    temp_cam_tar.data.y_distance = - Raw_Cam_un.data.y_distance + Pos16_of_Radar.pos_data.y_x100 + 5;

    Camera_data_un detection = Raw_Cam_un;

    detection.data.x_distance = 8 - (temp_cam_tar.data.y_distance + 25) / 50;
    detection.data.y_distance = (temp_cam_tar.data.x_distance + 25) / 50;
    
    // if(detection.data.x_distance == Now_Pos.x && detection.data.y_distance == Now_Pos.y)
    // {
        
    // }

}


