#ifndef _MAP_UPDATE_H
#define _MAP_UPDATE_H

#include "SysConfig.h"
#include "JetsonNano_Data_Transmit.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    s16 x;
    s16 y;
}Point_t;

typedef Point_t Point;

extern const Point_t All_Point[12];
extern Point_t delivery_point[2];
extern Point_t Now_Pos;

void WayPointFifoInit(void);
bool Add_WayPoint(Point_t *WayPoint, uint16_t size);
void WayPointClear(void);
Point_t WayPointTake(void);
u8 WayPointEmpty(void);

bool Add_ReturnHomeWayPoint(Point_t *WayPoint, uint16_t size);
void ReturnHomeWayPointClear(void);
Point_t ReturnHomeWayPointTake(void);
u8 ReturnHomeWayPointEmpty(void);
u8 MapPoint_IsValid(Point_t point);

void Update_Now_Pos(void);
void Update_Map_OnRawData(Camera_data_un Raw_Cam_un);


#endif
