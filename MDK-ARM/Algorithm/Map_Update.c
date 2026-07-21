#include "Map_Update.h"
#include "JetsonNano_Data_Transmit.h"
#include "fifo.h"

#define MAP_WAYPOINT_FIFO_DEPTH       (126U)

//定义航点
const Point_t All_Point[12] = {
    {60, 50},  /* 1 */
    {20, -20}, /* 8 */
    {80, 30},
};

//两个投放点/目标点
Point_t delivery_point[2] = {
    {0, 0},
    {0, 0}
};

Point_t Now_Pos;

static FIFO_Type Way_Point_Fifo;
static FIFO_Type ReturnHome_WayPoint_fifo;

//初始化两个航点队列的fifo
void WayPointFifoInit(void)
{
    static Point_t Way_Point_Buffer[MAP_WAYPOINT_FIFO_DEPTH];
    static Point_t ReturnHome_WayPoint_Buffer[MAP_WAYPOINT_FIFO_DEPTH];

    FIFO_Init(&Way_Point_Fifo,
              Way_Point_Buffer,
              sizeof(Point_t),
              sizeof(Way_Point_Buffer) / sizeof(Way_Point_Buffer[0]));
    FIFO_Init(&ReturnHome_WayPoint_fifo,
              ReturnHome_WayPoint_Buffer,
              sizeof(Point_t),
              sizeof(ReturnHome_WayPoint_Buffer) / sizeof(ReturnHome_WayPoint_Buffer[0]));
}

//添加一系列航点
bool Add_WayPoint(Point_t *WayPoint, uint16_t size)
{
    if(WayPoint == 0 || size == 0U)
    {
        return false;
    }

    if(FIFO_Add(&Way_Point_Fifo, WayPoint, size) == size)
    {
        return true;
    }

    return false;
}

//清空航点队列
void WayPointClear(void)
{
    FIFO_Clear(&Way_Point_Fifo);
}

//从普通航点队列取出一个点。
//如果队列为空，它会返回默认 {0, 0}，因为代码没有检查 FIFO_GetOne() 的返回值
Point_t WayPointTake(void)
{
    Point_t temp_Point = {0, 0};
    (void)FIFO_GetOne(&Way_Point_Fifo, &temp_Point);
    return temp_Point;
}

//判断普通航点队列是否为空
u8 WayPointEmpty(void)
{
    return FIFO_IsEmpty(&Way_Point_Fifo);
}

//向返航航点队列加入一组点。逻辑和 Add_WayPoint() 一样
bool Add_ReturnHomeWayPoint(Point_t *WayPoint, uint16_t size)
{
    if(WayPoint == 0 || size == 0U)
    {
        return false;
    }

    if(FIFO_Add(&ReturnHome_WayPoint_fifo, WayPoint, size) == size)
    {
        return true;
    }

    return false;
}

//清空返航航点队列
void ReturnHomeWayPointClear(void)
{
    FIFO_Clear(&ReturnHome_WayPoint_fifo);
}

//从返航航点队列取一个点
Point_t ReturnHomeWayPointTake(void)
{
    Point_t temp_Point = {0, 0};
    (void)FIFO_GetOne(&ReturnHome_WayPoint_fifo, &temp_Point);
    return temp_Point;
}

//判断返航航点队列是否为空
u8 ReturnHomeWayPointEmpty(void)
{
    return FIFO_IsEmpty(&ReturnHome_WayPoint_fifo);
}

//判断一个地图点是否有效
u8 MapPoint_IsValid(Point_t point)
{
    return (point.x != 0 || point.y != 0) ? 1U : 0U;
}


////////////////////////////////////////////////////////////
//用雷达当前位置更新当前地图格子位置
void Update_Now_Pos(void)
{
    Now_Pos.x = 8 - (Pos16_of_Radar.pos_data.y_x100 + 25) / 50;
    Now_Pos.y = (Pos16_of_Radar.pos_data.x_x100 + 25) / 50;
}

//在 Jetson 收到相机识别数据 0x03 时被调用
//这里 2.5 和 5 加到 s16 字段里会被截断成整数，实际效果分别接近 +2 和 +5，这一点后续可能要注意
void Update_Map_OnRawData(Camera_data_un Raw_Cam_un)
{
    //将视觉识别的结果转换到实际的雷达坐标系
    Camera_data_un temp_cam_tar = Raw_Cam_un;
    temp_cam_tar.data.x_distance = - Raw_Cam_un.data.x_distance + Pos16_of_Radar.pos_data.x_x100 + 2.5;
    temp_cam_tar.data.y_distance = - Raw_Cam_un.data.y_distance + Pos16_of_Radar.pos_data.y_x100 + 5;

    Camera_data_un detection = Raw_Cam_un;

    detection.data.x_distance = 8 - (temp_cam_tar.data.y_distance + 25) / 50;
    detection.data.y_distance = (temp_cam_tar.data.x_distance + 25) / 50;
    (void)detection;
    
    // if(detection.data.x_distance == Now_Pos.x && detection.data.y_distance == Now_Pos.y)
    // {
        
    // }

}


