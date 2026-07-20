#include "Remote_Control.h"
#include "usart.h"

#define SBUS_FRAME_LEN 25
#define SBUS_HEADER 0x0F
#define SBUS_END_BYTE 0x00
#define SBUS_TIMEOUT_MS 300
#define SBUS_RAW_MIN 172
#define SBUS_RAW_MAX 1811
#define SBUS_DMA_BUF_LEN 64

volatile u32 sbus_dma_byte_cnt = 0;
volatile u32 sbus_frame_cnt = 0;

rc_channel_un Channel_of_rc;
realtime_ctrl_un ctrl_of_realtime;
realtime_ctrl_un rc_ctrl_cmd;
realtime_ctrl_un nav_ctrl_cmd;
realtime_ctrl_un failsafe_ctrl_cmd;
SwitchStateSet Switch_sta_st;

static u8 sbus_dma_buf[SBUS_DMA_BUF_LEN];
static u8 sbus_frame[SBUS_FRAME_LEN];
static u8 sbus_index;
static u16 sbus_dma_pos;
static u32 sbus_last_update_ms;
static u8 sbus_signal_lost = 1;

//限幅函数
static s16 LimitS16(s16 value, s16 min, s16 max)
{
    if(value < min)
    {
        return min;
    }

    if(value > max)
    {
        return max;
    }

    return value;
}

//将Sbus原始值映射成pwm 1000-2000
static s16 SbusRawToPwm(u16 raw)
{
    s32 pwm = ((s32)raw - SBUS_RAW_MIN) * 1000 / (SBUS_RAW_MAX - SBUS_RAW_MIN) + 1000;
    return LimitS16((s16)pwm, 1000, 2000);
}

//Sbus协议解码函数，只使用前10个通道的数据
static void SbusDecodeFrame(const u8 frame[SBUS_FRAME_LEN])
{
    u16 raw_ch[16];

    raw_ch[0]  = ((u16)frame[1]       | ((u16)frame[2]  << 8)) & 0x07FF;
    raw_ch[1]  = (((u16)frame[2] >> 3) | ((u16)frame[3]  << 5)) & 0x07FF;
    raw_ch[2]  = (((u16)frame[3] >> 6) | ((u16)frame[4]  << 2) | ((u16)frame[5] << 10)) & 0x07FF;
    raw_ch[3]  = (((u16)frame[5] >> 1) | ((u16)frame[6]  << 7)) & 0x07FF;
    raw_ch[4]  = (((u16)frame[6] >> 4) | ((u16)frame[7]  << 4)) & 0x07FF;
    raw_ch[5]  = (((u16)frame[7] >> 7) | ((u16)frame[8]  << 1) | ((u16)frame[9] << 9)) & 0x07FF;
    raw_ch[6]  = (((u16)frame[9] >> 2) | ((u16)frame[10] << 6)) & 0x07FF;
    raw_ch[7]  = (((u16)frame[10] >> 5) | ((u16)frame[11] << 3)) & 0x07FF;
    raw_ch[8]  = ((u16)frame[12]      | ((u16)frame[13] << 8)) & 0x07FF;
    raw_ch[9]  = (((u16)frame[13] >> 3) | ((u16)frame[14] << 5)) & 0x07FF;
    raw_ch[10] = (((u16)frame[14] >> 6) | ((u16)frame[15] << 2) | ((u16)frame[16] << 10)) & 0x07FF;
    raw_ch[11] = (((u16)frame[16] >> 1) | ((u16)frame[17] << 7)) & 0x07FF;
    raw_ch[12] = (((u16)frame[17] >> 4) | ((u16)frame[18] << 4)) & 0x07FF;
    raw_ch[13] = (((u16)frame[18] >> 7) | ((u16)frame[19] << 1) | ((u16)frame[20] << 9)) & 0x07FF;
    raw_ch[14] = (((u16)frame[20] >> 2) | ((u16)frame[21] << 6)) & 0x07FF;
    raw_ch[15] = (((u16)frame[21] >> 5) | ((u16)frame[22] << 3)) & 0x07FF;

    for(u8 i = 0; i < 10; i++)
    {
        Channel_of_rc.data.ch[i] = SbusRawToPwm(raw_ch[i]);
    }
    sbus_frame_cnt++;

    sbus_signal_lost = ((frame[23] & 0x0C) != 0) ? 1 : 0;
    sbus_last_update_ms = HAL_GetTick();
}

//每一字节累计，累计够25字节后判断帧头，然后调用解码函数
static void SbusPushByte(u8 data)
{
    if(sbus_index == 0 && data != SBUS_HEADER)
    {
        return;
    }

    sbus_frame[sbus_index++] = data;

    if(sbus_index >= SBUS_FRAME_LEN)
    {
        if(sbus_frame[0] == SBUS_HEADER && sbus_frame[SBUS_FRAME_LEN - 1] == SBUS_END_BYTE)
        {
            SbusDecodeFrame(sbus_frame);
        }

        sbus_index = 0;
    }
}

//处理DMA（循环模式）写入数组的数据
static void SbusProcessDmaData(void)
{
    u16 pos;

    if(huart8.hdmarx == NULL)
    {
        return;
    }

    pos = (u16)(SBUS_DMA_BUF_LEN - __HAL_DMA_GET_COUNTER(huart8.hdmarx));

    while(sbus_dma_pos != pos)
    {
        SbusPushByte(sbus_dma_buf[sbus_dma_pos]);
        sbus_dma_pos++;
        sbus_dma_byte_cnt++;

        if(sbus_dma_pos >= SBUS_DMA_BUF_LEN)
        {
            sbus_dma_pos = 0;
        }
    }
}

//开启循环DMA搬运和串口空闲中断
static void SbusStartDmaReceive(void)
{
    HAL_UART_DMAStop(&huart8);

    sbus_index = 0;
    sbus_dma_pos = 0;

    if(HAL_UART_Receive_DMA(&huart8, sbus_dma_buf, SBUS_DMA_BUF_LEN) == HAL_OK)
    {
        if(huart8.hdmarx != NULL)
        {
            __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);
            __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_TC);
        }

        __HAL_UART_CLEAR_IDLEFLAG(&huart8);
        __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);
    }
}

//刚上电给它设为安全值 
//在RcInputInit里面调用
void RemoteControl_InitDefault(void)
{
    for(u8 i = 0; i < 10; i++)
    {
        Channel_of_rc.data.ch[i] = 1500;
    }

    Channel_of_rc.data.ch[2] = 1000;

    ctrl_of_realtime.data.roll = 0;
    ctrl_of_realtime.data.pitch = 0;
    ctrl_of_realtime.data.throttle = 0;
    ctrl_of_realtime.data.yaw_dps = 0;
    ctrl_of_realtime.data.vel_x = 0;
    ctrl_of_realtime.data.vel_y = 0;
    ctrl_of_realtime.data.vel_z = 0;

    rc_ctrl_cmd = ctrl_of_realtime;
    nav_ctrl_cmd = ctrl_of_realtime;
    failsafe_ctrl_cmd = ctrl_of_realtime;

    Switch_sta_st.SWA = Switch_Mid;
    Switch_sta_st.SWB = Switch_Mid;
    Switch_sta_st.SWC = Switch_Mid;
    Switch_sta_st.SWD = Switch_Mid;
    Switch_sta_st.VRA = Switch_Mid;
    Switch_sta_st.VRB = Switch_Mid;
}

//上电初始化为默认值 之后启用串口8的DMA接收
void DrvRcInputInit(void)
{
    RemoteControl_InitDefault();
    sbus_index = 0;
    sbus_dma_pos = 0;
    sbus_signal_lost = 1;
    sbus_last_update_ms = HAL_GetTick();
    SbusStartDmaReceive();
}

//判断上次更新遥控器的时间距离现在过了多久，判断遥控器是否失联
void DrvRcInputTask(float dt)
{
    (void)dt;

    SbusProcessDmaData();

    if((HAL_GetTick() - sbus_last_update_ms) > SBUS_TIMEOUT_MS)
    {
        sbus_signal_lost = 1;
    }
}

void RemoteControl_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART8)
    {
        SbusProcessDmaData();
    }
}

void RemoteControl_UartIdleCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART8)
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        SbusProcessDmaData();
    }
}

void RemoteControl_UartErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART8)
    {
        sbus_signal_lost = 1;
        SbusStartDmaReceive();
    }
}

//把遥控器信号丢失封装为一个函数
u8 RemoteControl_IsSignalLost(void)
{
    return sbus_signal_lost;
}
