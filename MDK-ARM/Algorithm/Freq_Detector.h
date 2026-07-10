#ifndef __FREQ_DETECTOR_H__
#define __FREQ_DETECTOR_H__

#include <stdint.h>
#include "main.h"  // 根据芯片型号包含头文件


typedef enum {
    Data_stream_Radar_Pos,
    Data_stream_Radar_Speed,
    Data_stream_Radar_cmd_vel,
    Data_stream_Radar_qua ,
	Data_stream_Radar_Yaw,
	Data_stream_cam_loc,
	Data_stream_cam_cmd_vel,
	Data_stream_Vel_Fu,
	Data_stream_NUM  // 用于自动记录枚举成员数量
} DataStreamType;



// 频率检测器结构体（模块核心）
typedef struct {
    volatile uint32_t data_count;    // 数据计数器（volatile防止编译器优化）
    uint32_t last_update_tick;       // 上次更新时间戳（ms）
    double current_freq;              // 当前计算的频率（Hz）
    uint32_t sample_window_ms;       // 采样窗口时间（默认500ms）
    void (*on_freq_update)(float);   // 频率更新回调（可选）
} FreqDetector;



// 初始化检测器
void FreqDetector_Init(FreqDetector *detector, uint32_t sample_window_ms);
void App_InitFreqDetectors(void);

// 数据到达时调用
void FreqDetector_OnData(FreqDetector *detector);

// 更新频率计算（主循环调用）
void FreqDetector_Update(FreqDetector *detector);
void App_UpdateAllFreqDetectors(void) ;

// 获取当前频率
double FreqDetector_GetFrequency(FreqDetector *detector);


extern FreqDetector JN_freq_detector[Data_stream_NUM];


#endif
