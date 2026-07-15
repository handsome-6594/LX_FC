#include "Map_Update.h"
#include "JetsonNano_Data_Transmit.h"
#include "fifo.h"
#include <stdio.h>

#define MAP_WAYPOINT_FIFO_DEPTH       (126U)
#define MAP_WAYPOINT_PRINT_ENABLE     (1U)

const Point_t All_Point[12] = {
    {275, -50},  /* 1 */
    {275, -200}, /* 11 */
    {275, -350}, /* 5 */
    {200, -125}, /* 8 */
    {200, -275}, /* 3 */
    {125, -50},  /* 9 */
    {125, -200}, /* 2 */
    {125, -350}, /* 12 */
    {50,  -125}, /* 7 */
    {50,  -275}, /* 6 */
    {-25, -200}, /* 10 */
    {-25, -350}  /* 4 */
};

Point_t delivery_point[2] = {
    {0, 0},
    {0, 0}
};

Point_t Now_Pos;

static FIFO_Type Way_Point_Fifo;
static FIFO_Type ReturnHome_WayPoint_fifo;

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

static void MapPoint_PrintRoute(const char *name, Point_t *way_point, uint16_t size)
{
#if MAP_WAYPOINT_PRINT_ENABLE
    printf("\r\n%s:", name);
    for(uint16_t i = 0; i < size; i++)
    {
        printf("(%d,%d) ", way_point[i].x, way_point[i].y);
    }
    printf("\r\n");
#else
    (void)name;
    (void)way_point;
    (void)size;
#endif
}

bool Add_WayPoint(Point_t *WayPoint, uint16_t size)
{
    if(WayPoint == 0 || size == 0U)
    {
        return false;
    }

    if(FIFO_Add(&Way_Point_Fifo, WayPoint, size) == size)
    {
        MapPoint_PrintRoute("waypoint", WayPoint, size);
        return true;
    }

#if MAP_WAYPOINT_PRINT_ENABLE
    printf("waypoint add failed\r\n");
#endif
    return false;
}

void WayPointClear(void)
{
    FIFO_Clear(&Way_Point_Fifo);
}

Point_t WayPointTake(void)
{
    Point_t temp_Point = {0, 0};
    (void)FIFO_GetOne(&Way_Point_Fifo, &temp_Point);
    return temp_Point;
}

u8 WayPointEmpty(void)
{
    return FIFO_IsEmpty(&Way_Point_Fifo);
}

bool Add_ReturnHomeWayPoint(Point_t *WayPoint, uint16_t size)
{
    if(WayPoint == 0 || size == 0U)
    {
        return false;
    }

    if(FIFO_Add(&ReturnHome_WayPoint_fifo, WayPoint, size) == size)
    {
        MapPoint_PrintRoute("return", WayPoint, size);
        return true;
    }

#if MAP_WAYPOINT_PRINT_ENABLE
    printf("return waypoint add failed\r\n");
#endif
    return false;
}

void ReturnHomeWayPointClear(void)
{
    FIFO_Clear(&ReturnHome_WayPoint_fifo);
}

Point_t ReturnHomeWayPointTake(void)
{
    Point_t temp_Point = {0, 0};
    (void)FIFO_GetOne(&ReturnHome_WayPoint_fifo, &temp_Point);
    return temp_Point;
}

u8 ReturnHomeWayPointEmpty(void)
{
    return FIFO_IsEmpty(&ReturnHome_WayPoint_fifo);
}

u8 MapPoint_IsValid(Point_t point)
{
    return (point.x != 0 || point.y != 0) ? 1U : 0U;
}

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
    (void)detection;
    
    // if(detection.data.x_distance == Now_Pos.x && detection.data.y_distance == Now_Pos.y)
    // {
        
    // }

}


