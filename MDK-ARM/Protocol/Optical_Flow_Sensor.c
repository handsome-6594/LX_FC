#include "Optical_Flow_Sensor.h"
#include "Drv_Uart.h"
#include <stdio.h>
#include <string.h>

#define OF_FRAME_HEAD 0xAA
#define OF_FRAME_MAX_LEN 50
#define OF_LINK_TIMEOUT_MS 5000.0f

#define OF_MSG_INERTIAL 0x01
#define OF_MSG_QUATERNION 0x04
#define OF_MSG_ALTITUDE 0x34
#define OF_MSG_FLOW 0x51

#define OF_FLOW_RAW 0
#define OF_FLOW_FUSION 1
#define OF_FLOW_INERTIAL 2

#define OF_DEBUG_ENABLE (0U)
#define OF_DEBUG_PERIOD_MS (500U)

OpticalFlowData optical_flow;

static u8 frame_buf[OF_FRAME_MAX_LEN];
static float link_time_ms;
static float flow_time_ms;
static float alt_time_ms;
static u8 flow_updated;
static u8 alt_updated;
static u32 of_frame_ok_cnt;
static u32 of_frame_len_fail_cnt;
static u32 of_frame_sum_fail_cnt;
static u32 of_frame_flow_cnt;
static u32 of_frame_alt_cnt;
static u8 of_last_alt_payload[7];

static s16 ReadS16(const u8 *data)
{
    return (s16)((u16)data[0] | ((u16)data[1] << 8));
}

static u32 ReadU32(const u8 *data)
{
    return (u32)data[0] |
           ((u32)data[1] << 8) |
           ((u32)data[2] << 16) |
           ((u32)data[3] << 24);
}

static void MarkLinkUpdated(void)
{
    link_time_ms = 0.0f;
    optical_flow.link_sta = 1;
}
//光流数据解包
static void OpticalFlow_ParseFrame(const u8 *data, u8 len)
{
    u8 check_sum1 = 0;
    u8 check_sum2 = 0;

    if(len < 6 || data[3] != (u8)(len - 6))
    {
        of_frame_len_fail_cnt++;
        return;
    }

    for(u8 i = 0; i < len - 2; i++)
    {
        check_sum1 += data[i];
        check_sum2 += check_sum1;
    }

    if(check_sum1 != data[len - 2] || check_sum2 != data[len - 1])
    {
        of_frame_sum_fail_cnt++;
        return;
    }

    of_frame_ok_cnt++;
    MarkLinkUpdated();

    if(data[2] == OF_MSG_FLOW)
    {
        of_frame_flow_cnt++;
        if(data[3] < 5)
        {
            return;
        }

        if(data[4] == OF_FLOW_RAW)
        {
            optical_flow.raw_flow_sta = data[5];
            optical_flow.raw_flow_dx = (s8)data[6];
            optical_flow.raw_flow_dy = (s8)data[7];
            optical_flow.flow_quality = data[8];
        }
        else if(data[4] == OF_FLOW_FUSION)
        {
            if(data[3] < 7)
            {
                return;
            }

            optical_flow.fusion_flow_sta = data[5];
            optical_flow.fusion_flow_dx = ReadS16(&data[6]);
            optical_flow.fusion_flow_dy = ReadS16(&data[8]);
            optical_flow.flow_quality = data[10];

            flow_time_ms = 0.0f;
            optical_flow.flow_update_cnt++;
            flow_updated = 1;
        }
        else if(data[4] == OF_FLOW_INERTIAL)
        {
            if(data[3] < 15)
            {
                return;
            }

            optical_flow.inertial_flow_sta = data[5];
            optical_flow.inertial_flow_dx = ReadS16(&data[6]);
            optical_flow.inertial_flow_dy = ReadS16(&data[8]);
            optical_flow.inertial_flow_dx_fix = ReadS16(&data[10]);
            optical_flow.inertial_flow_dy_fix = ReadS16(&data[12]);
            optical_flow.integral_x = ReadS16(&data[14]);
            optical_flow.integral_y = ReadS16(&data[16]);
            optical_flow.flow_quality = data[18];
        }
    }
    else if(data[2] == OF_MSG_ALTITUDE)
    {
        of_frame_alt_cnt++;
        if(data[3] < 7)
        {
            return;
        }

        memcpy(of_last_alt_payload, &data[4], sizeof(of_last_alt_payload));
        optical_flow.alt_cm = ReadU32(&data[7]);
        alt_time_ms = 0.0f;
        optical_flow.alt_update_cnt++;
        alt_updated = 1;
    }
    else if(data[2] == OF_MSG_INERTIAL)
    {
        if(data[3] < 12)
        {
            return;
        }

        optical_flow.acc_x = ReadS16(&data[4]);
        optical_flow.acc_y = ReadS16(&data[6]);
        optical_flow.acc_z = ReadS16(&data[8]);
        optical_flow.gyro_x = ReadS16(&data[10]);
        optical_flow.gyro_y = ReadS16(&data[12]);
        optical_flow.gyro_z = ReadS16(&data[14]);
    }
    else if(data[2] == OF_MSG_QUATERNION)
    {
        if(data[3] < 8)
        {
            return;
        }

        optical_flow.quaternion[0] = (float)ReadS16(&data[4]) * 0.0001f;
        optical_flow.quaternion[1] = (float)ReadS16(&data[6]) * 0.0001f;
        optical_flow.quaternion[2] = (float)ReadS16(&data[8]) * 0.0001f;
        optical_flow.quaternion[3] = (float)ReadS16(&data[10]) * 0.0001f;
    }
}

void OpticalFlow_Init(void)
{
    memset(&optical_flow, 0, sizeof(optical_flow));
    link_time_ms = OF_LINK_TIMEOUT_MS;
    flow_time_ms = OF_LINK_TIMEOUT_MS;
    alt_time_ms = OF_LINK_TIMEOUT_MS;
    flow_updated = 0;
    alt_updated = 0;
    of_frame_ok_cnt = 0;
    of_frame_len_fail_cnt = 0;
    of_frame_sum_fail_cnt = 0;
    of_frame_flow_cnt = 0;
    of_frame_alt_cnt = 0;
    memset(of_last_alt_payload, 0, sizeof(of_last_alt_payload));

    DrvUart2_RegisterRxByteHandler(OpticalFlow_ReceiveByte);
}
//光流数据超时状态检查
void OpticalFlow_CheckState(float dT_s)
{
    float dT_ms = dT_s * 1000.0f;
#if OF_DEBUG_ENABLE
    static u32 last_debug_ms;
    u32 now_ms;
#endif

    if(link_time_ms < OF_LINK_TIMEOUT_MS)
    {
        link_time_ms += dT_ms;
    }

    if(flow_time_ms < OF_LINK_TIMEOUT_MS)
    {
        flow_time_ms += dT_ms;
    }

    if(alt_time_ms < OF_LINK_TIMEOUT_MS)
    {
        alt_time_ms += dT_ms;
    }

    optical_flow.link_sta = (link_time_ms < OF_LINK_TIMEOUT_MS) ? 1 : 0;
    optical_flow.flow_sta = (flow_time_ms < OF_LINK_TIMEOUT_MS) ? 1 : 0;
    optical_flow.alt_sta = (alt_time_ms < OF_LINK_TIMEOUT_MS) ? 1 : 0;
    optical_flow.work_sta = (optical_flow.flow_sta && optical_flow.alt_sta) ? 1 : 0;

#if OF_DEBUG_ENABLE
    now_ms = HAL_GetTick();
    if((now_ms - last_debug_ms) >= OF_DEBUG_PERIOD_MS)
    {
        last_debug_ms = now_ms;
        printf("of uart ev=%lu byte=%lu err=%lu ecode=%lu rx_en=%lu fail=%lu frame ok=%lu len=%lu sum=%lu flowf=%lu altf=%lu sta l=%u f=%u a=%u w=%u q=%u fs=%u cnt[%u,%u] vel[%d,%d] alt=%lu alt_raw=%02X %02X %02X %02X %02X %02X %02X\r\n",
               (unsigned long)of_uart2_rx_event_cnt,
               (unsigned long)of_uart2_rx_byte_cnt,
               (unsigned long)of_uart2_error_cnt,
               (unsigned long)of_uart2_error_code,
               (unsigned long)of_uart2_receive_enable_cnt,
               (unsigned long)of_uart2_receive_enable_fail_cnt,
               (unsigned long)of_frame_ok_cnt,
               (unsigned long)of_frame_len_fail_cnt,
               (unsigned long)of_frame_sum_fail_cnt,
               (unsigned long)of_frame_flow_cnt,
               (unsigned long)of_frame_alt_cnt,
               optical_flow.link_sta,
               optical_flow.flow_sta,
               optical_flow.alt_sta,
               optical_flow.work_sta,
               optical_flow.flow_quality,
               optical_flow.fusion_flow_sta,
               optical_flow.flow_update_cnt,
               optical_flow.alt_update_cnt,
               optical_flow.fusion_flow_dx,
               optical_flow.fusion_flow_dy,
               (unsigned long)optical_flow.alt_cm,
               (unsigned int)of_last_alt_payload[0],
               (unsigned int)of_last_alt_payload[1],
               (unsigned int)of_last_alt_payload[2],
               (unsigned int)of_last_alt_payload[3],
               (unsigned int)of_last_alt_payload[4],
               (unsigned int)of_last_alt_payload[5],
               (unsigned int)of_last_alt_payload[6]);
    }
#endif
}

//逐字节收数据
void OpticalFlow_ReceiveByte(u8 data)
{
    static u8 data_len = 0;
    static u8 data_cnt = 0;
    static u8 rx_state = 0;

    if(rx_state == 0 && data == OF_FRAME_HEAD)
    {
        rx_state = 1;
        frame_buf[0] = data;
    }
    else if(rx_state == 1 && (data == HW_TYPE || data == HW_ALL))
    {
        rx_state = 2;
        frame_buf[1] = data;
    }
    else if(rx_state == 2)
    {
        rx_state = 3;
        frame_buf[2] = data;
    }
    else if(rx_state == 3 && data <= (OF_FRAME_MAX_LEN - 6))
    {
        rx_state = 4;
        frame_buf[3] = data;
        data_len = data;
        data_cnt = 0;

        if(data_len == 0)
        {
            rx_state = 5;
        }
    }
    else if(rx_state == 4 && data_len > 0)
    {
        data_len--;
        frame_buf[4 + data_cnt++] = data;

        if(data_len == 0)
        {
            rx_state = 5;
        }
    }
    else if(rx_state == 5)
    {
        rx_state = 6;
        frame_buf[4 + data_cnt++] = data;
    }
    else if(rx_state == 6)
    {
        rx_state = 0;
        frame_buf[4 + data_cnt] = data;
        OpticalFlow_ParseFrame(frame_buf, data_cnt + 5);
    }
    else
    {
        rx_state = 0;
    }
}

u8 OpticalFlow_IsFlowUpdated(void)
{
    u8 updated = flow_updated;
    flow_updated = 0;
    return updated;
}

u8 OpticalFlow_IsAltUpdated(void)
{
    u8 updated = alt_updated;
    alt_updated = 0;
    return updated;
}
