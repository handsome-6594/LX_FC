#include "JetsonNano_Data_Transmit.h"
#include <stdio.h>
#include "Radar_Position.h"
#include "Of_Radar_Fusion.h"
#include "Optical_Flow_Sensor.h"
#include "Freq_Detector.h"
#include "usart.h"
#include "Map_Update.h"
#include "Drv_Uart.h"

#define JN_CMD_ACK_RETRY_INTERVAL_MS 80
#define JN_CMD_ACK_RETRY_MAX         8

JetsonNano_Data JN_DT_st;
Radar_Pos_un Pos_of_Radar;
volatile Radar_Pos_16_un Pos16_of_Radar;
volatile Radar_Speed_un Speed_of_Radar;
volatile Radar_Cmd_Vel_un speed_cmd_un;
volatile u8 radar_pos_update_cnt = 0;
volatile u8 radar_qua_update_cnt = 0;
Camera_data_un Camera_Pos_data;
x10000_Radar_qua_un Radar_qua_x10000;
Radar_qua real_Radar_qua;

volatile _update_Flag_st update_Flag = {
    .Radar_Pos = 0,
    .Radar_Speed = 0,
    .Cam_pix = 0,
    .Radar_Cmd_Vel = 0,
    .Radar_PID_Cmd_Vel = 0,
    .Camera_PID_Cmd_Vel = 0,
    .Radar_qua = 0
};

static u8 rx_Buffer[256];
static u8 send_buffer[50];
static u8 Data_cnt = 0;

static void H743_Received_Data_From_JetsonNano_Analysis(const u8 *data, u8 len);
static void JN_Send_Data_Buffer(u8 frame_num, frame_pack *pack);
static void H743_To_JN_FrameSend(u8 frame_num, Data_Frame *frame);
static u8 H743_To_JN_Send_Data(u8 *data, u8 length);
static void JN_Check_To_Send(u8 frame_num);
static inline void JN_CK_Back_Check(void);

static u8 jn_cmd_repeat_cnt = 0;
static u16 jn_cmd_ack_time_ms = 0;

static inline u16 JN_GetU16Le(const u8 *data)
{
    return (u16)data[0] | ((u16)data[1] << 8);
}

//发送0XE0命令帧
void JN_CMD_Send(u8 dest_addr, _cmd_st *cmd)
{
    if(cmd == NULL)
    {
        return;
    }

    JN_DT_st.data_of_cmd = *cmd;
    JN_DT_st.fun[0XE0].Addr = dest_addr;
    JN_DT_st.fun[0XE0].wait_to_send = 1;
    JN_DT_st.cmd_wait_ack = 1;
    jn_cmd_repeat_cnt = 0;
    jn_cmd_ack_time_ms = 0;
}

//0x00校验帧返回
void JN_CK_Back(u8 dest_addr, _ck_st *ck)
{
    if(ck == NULL)
    {
        return;
    }

    JN_DT_st.ack_of_check = *ck;
    JN_DT_st.fun[0X00].Addr = dest_addr;
    JN_DT_st.fun[0X00].wait_to_send = 1;
}

//参数返回
void JN_PAR_Back(u8 dest_addr, _par_st *par)
{
    if(par == NULL)
    {
        return;
    }

    JN_DT_st.param_data = *par;
    JN_DT_st.fun[0XE2].Addr = dest_addr;
    JN_DT_st.fun[0XE2].wait_to_send = 1;
}

//一个字节一个字节收，组包
void JetsonNano_To_H743_Data_Prepare(u8 data)
{
    static u8 Data_len = 0, payload_cnt = 0;
    static u8 rxstate = 0;
    
    if(rxstate == 0 && data == 0xAA)
    {
        rxstate = 1;
        Data_len = 0;
        payload_cnt = 0;
        rx_Buffer[0] = data;
    }else if (rxstate ==1 && (data == HW_ALL || data == HW_TYPE))
    {
        rxstate = 2;
        rx_Buffer[1] = data;
    }else if(rxstate == 2)
    {
        rxstate = 3;
        rx_Buffer[2] = data;
    }else if(rxstate == 3 && data < 250)
    {
        rxstate = 4;
        rx_Buffer[3] = data;
        Data_len = data;
    }else if(rxstate == 4 && Data_len > 0)
    {
        Data_len--;
        rx_Buffer[4 + payload_cnt++] = data;
        if(Data_len == 0){
            rxstate = 5;
        }
    }else if(rxstate == 5){
        rxstate = 6; 
        rx_Buffer[4 + payload_cnt++] = data;  
    }else if(rxstate == 6)
    {
        rxstate = 0;
        rx_Buffer[4+ payload_cnt] = data;
        Data_cnt = payload_cnt + 5;
        H743_Received_Data_From_JetsonNano_Analysis(rx_Buffer, Data_cnt);
    }
    else {
        rxstate = 0;
        Data_len = 0;
        payload_cnt = 0;
    }
}

//H743将从JetsonNano过来的数据进行解包
static void H743_Received_Data_From_JetsonNano_Analysis(const u8 *data, u8 len)
{
    u8 sum1_check = 0, sum2_check = 0;
    //判断数据长度是否正确
    if(*(data + 3) != len - 6)
    return;

    //计算校验位
    for(u8 i = 0; i < len - 2; i++)
    {
        sum1_check += *(data + i);
        sum2_check += sum1_check;
    }
    //检验校验位是否一致
    if(sum1_check != *(data + len -2) || sum2_check != *(data + len -1))
    return;

    //再次判断帧头和广播地址
    if((*(data) != 0xAA) || (*(data+1) != HW_ALL && *(data+1) != HW_TYPE))
    return;

    //解析相应的数据

    //从初始位置开始，根据传感器数据不断积分，进而估计当前位姿
    //如此得来的雷达里程计数据，也就是
    //位置数据
    if(*(data + 2) == 0X01)
    {
        Radar_Pos_16_un raw_pos;
        Radar_Pos_16_un rel_pos;

        if(data[3] != sizeof(raw_pos.byte_data))
        {
            return;
        }
        for(u8 i = 0; i < sizeof(raw_pos.byte_data); i++)
        {
            raw_pos.byte_data[i] = data[4 + i];
        }

        if(radar_pos_init_val.init_flag == 0)
        {
            RadarPos_SetInitPoint(raw_pos);
        }

        rel_pos = RadarPos_ToRelative(raw_pos);
        if(alt_soruce == 1 && optical_flow.work_sta)
        {
            rel_pos.pos_data.z_x100 = (s16)optical_flow.alt_cm;
        }

        Pos16_of_Radar = rel_pos;
        Pos_of_Radar.pos_data.POS_X_x100 = rel_pos.pos_data.x_x100;
        Pos_of_Radar.pos_data.POS_Y_x100 = rel_pos.pos_data.y_x100;
        Pos_of_Radar.pos_data.POS_Z_x100 = rel_pos.pos_data.z_x100;
        radar_pos_update_cnt++;
        update_Flag.Radar_Pos = 1;

        // static u32 last_radar_pos_print_ms = 0;
        // u32 now_ms = HAL_GetTick();
        // if(now_ms - last_radar_pos_print_ms >= 200)
        // {
        //     last_radar_pos_print_ms = now_ms;
        //     printf("radar pos raw=%d,%d,%d rel=%d,%d,%d cnt=%u\r\n",
        //            raw_pos.pos_data.x_x100,
        //            raw_pos.pos_data.y_x100,
        //            raw_pos.pos_data.z_x100,
        //            rel_pos.pos_data.x_x100,
        //            rel_pos.pos_data.y_x100,
        //            rel_pos.pos_data.z_x100,
        //            radar_pos_update_cnt);
        // }
    }
    //雷达里程计的速度数据
    else if(*(data + 2) == 0X02)
    {
        if(data[3] != sizeof(Speed_of_Radar.byte_data))
        {
            return;
        }
        for(u8 i = 0; i < sizeof(Speed_of_Radar.byte_data); i++)
        {
            Speed_of_Radar.byte_data[i] = data[4 + i];
        }
        update_Flag.Radar_Speed = 1;
        FreqDetector_OnData(&JN_freq_detector[Data_stream_Radar_Speed]);
        // printf("radar vel vx = %d, vy = %d, vz = %d\r\n", Speed_of_Radar.speed_data.vx_x100, Speed_of_Radar.speed_data.vy_x100, Speed_of_Radar.speed_data.vz_x100);
    }
    //摄像头指示发现了什么，要往哪里走
    else if(*(data + 2) == 0X03)
    {
        for(u8 i = 0; i < sizeof(Camera_Pos_data); i++)
        {
            Camera_Pos_data.byte_data[i] = data[4 + i];
        }
        Update_Map_OnRawData(Camera_Pos_data);
        FreqDetector_OnData(&JN_freq_detector[Data_stream_cam_loc]);
        update_Flag.Cam_pix = 1;
        // printf("cam x=%d y=%d state=%u type=%u id=%d\r\n",
        //        Camera_Pos_data.data.x_distance,
        //        Camera_Pos_data.data.y_distance,
        //        Camera_Pos_data.data.state,
        //        Camera_Pos_data.data.type,
        //        Camera_Pos_data.data.id);
    }
    //雷达yaw轴数据
    else if(*(data + 2) == 0X04)
    {
    //需要的时候再实现
    }

    //上位机速度控制量
    else if(*(data + 2) == 0X05)
    {
        if(data[3] != sizeof(speed_cmd_un.byte_data))
        {
            return;
        }
        for(u8 i = 0; i < sizeof(speed_cmd_un.byte_data); i++)
        {
            speed_cmd_un.byte_data[i] = data[4 + i];
        }
        update_Flag.Radar_Cmd_Vel = 1;
        FreqDetector_OnData(&JN_freq_detector[Data_stream_Radar_cmd_vel]);
    }
    //雷达四元数
    else if(*(data + 2) == 0X06)
    {
        if(data[3] != sizeof(Radar_qua_x10000.byte_data))
        {
            return;
        }
        for(u8 i = 0; i < sizeof(Radar_qua_x10000.byte_data); i++)
        {
            Radar_qua_x10000.byte_data[i] = data[4 + i];
        }
        update_Flag.Radar_qua = 1;
        real_Radar_qua.qx = Radar_qua_x10000.data.qx_x10000 / 10000.0f;
        real_Radar_qua.qy = Radar_qua_x10000.data.qy_x10000 / 10000.0f;
        real_Radar_qua.qz = Radar_qua_x10000.data.qz_x10000 / 10000.0f;
        real_Radar_qua.qw = Radar_qua_x10000.data.qw_x10000 / 10000.0f;
        radar_qua_update_cnt++;
        FreqDetector_OnData(&JN_freq_detector[Data_stream_Radar_qua]);
    }
    //0x00 校验返回帧
    else if(*(data + 2) == 0X00)
    {
        if(data[3] != 3)
        {
            return;
        }
        if(JN_DT_st.checksum_ok.ID == *(data + 4) && JN_DT_st.checksum_ok.SC == *(data + 5) && JN_DT_st.checksum_ok.AC == *(data + 6))
        {
            JN_DT_st.cmd_wait_ack = 0;
        }
    }
    //0xE0 预留一些命令接口
    else if(*(data + 2) == 0XE0)
    {
        //根据命令ID，执行相应命令
        switch(*(data + 4))
        {
            case 0X01:
            {

            }
            break;
            case 0X02:
            {

            }
            break;
            case 0X10:
            {

            }
            break;
            case 0X11:
            {

            }
            break;
            default:
            break;
        }
        //H743收到命令并执行完上面的switch case后，给JetsonNano回一个ack
        
        _ck_st the_check_ack;
        the_check_ack.ID = *(data + 4);
        the_check_ack.SC = sum1_check;
        the_check_ack.AC = sum2_check;
        JN_CK_Back(HW_ALL, &the_check_ack);
    }
    //JetsonNano要从H743读数据
    else if(*(data + 2) == 0XE1)
    {
        if(data[3] != 2)
        {
            return;
        }

        JN_DT_st.param_data.par_ID = JN_GetU16Le(data + 4);
        JN_DT_st.param_data.par_value = 0;
        JN_PAR_Back(HW_ALL, &JN_DT_st.param_data);
    }
    //JetsonNano要向H743写参数
    else if(*(data + 2) == 0XE2)
    {
        if(data[3] < 1)
        {
            return;
        }

        JN_DT_st.ack_of_check.ID = *(data + 4);
        JN_DT_st.ack_of_check.SC = sum1_check;
        JN_DT_st.ack_of_check.AC = sum2_check;
        JN_CK_Back(HW_ALL, &JN_DT_st.ack_of_check);
    }

}

//H743组有效数据区
static void JN_Send_Data_Buffer(u8 frame_num, frame_pack *pack)
{
    switch(frame_num)
    {
        case 0X00:
        {
            FramePack_PutByte(pack, JN_DT_st.ack_of_check.ID);
            FramePack_PutByte(pack, JN_DT_st.ack_of_check.SC);
            FramePack_PutByte(pack, JN_DT_st.ack_of_check.AC);
        }
        break;

        case 0XE0:
        {
            FramePack_PutByte(pack, JN_DT_st.data_of_cmd.CID);
            FramePack_PutBuf(pack, JN_DT_st.data_of_cmd.CMD, sizeof(JN_DT_st.data_of_cmd.CMD));
        }
        break;

        case 0XE2:
        {
            FramePack_U16(pack, JN_DT_st.param_data.par_ID);
            FramePack_U32(pack, JN_DT_st.param_data.par_value);
        }
        break;

        default:
            break;
    }
}

//H743给JetsonNano回一个完整的帧
static void H743_To_JN_FrameSend(u8 frame_num, Data_Frame *frame)
{
    frame_pack pack;
    u8 check_sum1 = 0;
    u8 check_sum2 = 0;

    if(frame == NULL)
    {
        return;
    }

    FramePack_Init(&pack, send_buffer, sizeof(send_buffer));
    FramePack_PutByte(&pack, 0XAA);
    FramePack_PutByte(&pack, frame->Addr);
    FramePack_PutByte(&pack, frame_num);
    FramePack_PutByte(&pack, 0);

    JN_Send_Data_Buffer(frame_num, &pack);
    send_buffer[3] = (u8)(pack.len - 4);

    for(u16 i = 0; i < pack.len; i++)
    {
        check_sum1 += send_buffer[i];
        check_sum2 += check_sum1;
    }

    FramePack_PutByte(&pack, check_sum1);
    FramePack_PutByte(&pack, check_sum2);

    if(JN_DT_st.cmd_wait_ack != 0 && frame_num == 0XE0)
    {
        JN_DT_st.checksum_ok.ID = frame_num;
        JN_DT_st.checksum_ok.SC = check_sum1;
        JN_DT_st.checksum_ok.AC = check_sum2;
    }

    H743_To_JN_Send_Data(send_buffer, pack.len);
}

static u8 H743_To_JN_Send_Data(u8 *data, u8 length)
{
    return DrvUart3SendBuf(data, length);
}

static void JN_Check_To_Send(u8 frame_num)
{
    if(JN_DT_st.fun[frame_num].wait_to_send)
    {
        JN_DT_st.fun[frame_num].wait_to_send = 0;
        H743_To_JN_FrameSend(frame_num, &JN_DT_st.fun[frame_num]);
    }
}

//若0xE0命令帧没有收到0x00校验返回，则按固定间隔重发
static inline void JN_CK_Back_Check(void)
{
    if(JN_DT_st.cmd_wait_ack == 0)
    {
        jn_cmd_ack_time_ms = 0;
        jn_cmd_repeat_cnt = 0;
        return;
    }

    if(JN_DT_st.fun[0XE0].wait_to_send)
    {
        return;
    }

    if(jn_cmd_ack_time_ms < JN_CMD_ACK_RETRY_INTERVAL_MS)
    {
        jn_cmd_ack_time_ms++;
        return;
    }

    jn_cmd_ack_time_ms = 0;
    jn_cmd_repeat_cnt++;

    if(jn_cmd_repeat_cnt < JN_CMD_ACK_RETRY_MAX)
    {
        JN_DT_st.fun[0XE0].wait_to_send = 1;
    }
    else
    {
        jn_cmd_repeat_cnt = 0;
        JN_DT_st.cmd_wait_ack = 0;
    }
}

void JN_Data_Transmit_Check(void)
{
    JN_Check_To_Send(0X00);
    JN_Check_To_Send(0XE0);
    JN_Check_To_Send(0XE2);
    JN_CK_Back_Check();
}
