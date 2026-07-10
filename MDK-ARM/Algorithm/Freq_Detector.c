#include "freq_detector.h"


FreqDetector JN_freq_detector[Data_stream_NUM];


// 初始化检测器
void FreqDetector_Init(FreqDetector *detector, uint32_t sample_window_ms) {
    detector->data_count = 0;
    detector->last_update_tick = HAL_GetTick();
    detector->current_freq = 0.0f;
    detector->sample_window_ms = sample_window_ms;
    detector->on_freq_update = NULL; // 默认无回调
}


void FreqDetector_OnData(FreqDetector *detector)   
{	
	detector->data_count++;
}


// 更新频率计算（非阻塞）
void FreqDetector_Update(FreqDetector *detector) {
    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed = current_tick - detector->last_update_tick;

	// 超时检测（例如：超过3倍采样窗口无数据）
    if (elapsed > 4 * detector->sample_window_ms) {
        detector->current_freq = 0.0f;
        detector->data_count = 0;
        detector->last_update_tick = current_tick;
        return;
    }
	
    if (elapsed >= detector->sample_window_ms) {
        if (elapsed > 0) {
            // 计算频率并重置计数器
			double raw_freq = detector->data_count / (elapsed / 1000.0);
            detector->current_freq = (raw_freq > 6553.5) ? 6553.5: raw_freq;
            detector->data_count = 0;
            detector->last_update_tick = current_tick;

            // 触发回调（如频率超过阈值报警）
            if (detector->on_freq_update != NULL) {
                detector->on_freq_update(detector->current_freq);
            }
        }
    }
	
	
	
}

// 获取当前频率
double FreqDetector_GetFrequency(FreqDetector *detector) {
    return detector->current_freq;
}

void App_UpdateAllFreqDetectors(void) {
    for (u8 i = 0; i < Data_stream_NUM; i++) {
        FreqDetector_Update(&JN_freq_detector[i]);
    }
}


void App_InitFreqDetectors(void) {
    FreqDetector_Init(&JN_freq_detector[Data_stream_Radar_Pos], 1000);   // 雷达数据：0.5秒窗口
    FreqDetector_Init(&JN_freq_detector[Data_stream_Radar_Speed], 1000); // 雷达速度速度数据：0.5秒窗口
    FreqDetector_Init(&JN_freq_detector[Data_stream_Radar_cmd_vel], 1000); //   雷达yaw轴：0.5秒窗口
	FreqDetector_Init(&JN_freq_detector[Data_stream_Radar_qua], 1000); //   雷达四元数：0.5s
	FreqDetector_Init(&JN_freq_detector[Data_stream_Radar_Yaw], 1000); //   摄像头视觉坐标：0.5s
	FreqDetector_Init(&JN_freq_detector[Data_stream_cam_loc], 1000); //   
	FreqDetector_Init(&JN_freq_detector[Data_stream_cam_cmd_vel], 1000); //   
	FreqDetector_Init(&JN_freq_detector[Data_stream_Vel_Fu], 1000);   // 雷达数据：0.5秒窗口
}

