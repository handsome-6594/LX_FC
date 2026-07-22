#include "GroundStation_Data_Transmit.h"
#include "Freq_Detector.h"
#include "Remote_Control.h"


#define Radar_Pos_Frame_NUM 0x01
#define Radar_Speed_Frame_NUM 0x02
#define JN_Cam_Frame_NUM 0x03
#define Radar_YAW_Frame_NUM 0x04

#define Motor_Speed_Frame_NUM 0x10
#define Batt_Curr_Height_Process_Frame_NUM 0x11
#define Cmd_Vel_Frame_NUM 0x12
#define VEL_FU_FRAME_NUM 0x13

#define Dev_Address 0xff
#define DT_Head 0xAA


GD_Data_Transmit GD_Data;
static pwm_value_un value_of_pwm;
Batt_Curr_Height_Process_un un_of_Batt_Height_Process;
Vel_Fu_un un_of_Vel_Fu;

static u8 rx_Buffer[256];
static u8 send_buffer[50];
static u8 Data_cnt = 0;

Point No_Fly_Zone[3] = {0};

static inline u16 GD_GetU16Le(const u8 *data)
{
    return (u16)data[0] | ((u16)data[1] << 8);
}

void GD_Data_Init(void)
{
    //返回校验帧
    GD_Data.fun[0x00].Addr = 0xff;
    GD_Data.fun[0x00].fre_ms = 0;  //由外部触发
    GD_Data.fun[0x00].count_mstime = 0;

    //cmd命令帧
    GD_Data.fun[0xe0].Addr = 0xff;
    GD_Data.fun[0xe0].fre_ms = 0;
    GD_Data.fun[0xe0].count_mstime = 0;

    //参数写入、参数读取的返回
    GD_Data.fun[0xe2].Addr = 0xff;
    GD_Data.fun[0xe2].fre_ms = 0;
    GD_Data.fun[0xe2].count_mstime = 0;

    //雷达位置数据
    GD_Data.fun[Radar_Pos_Frame_NUM].Addr = Dev_Address;
    GD_Data.fun[Radar_Pos_Frame_NUM].fre_ms = 100;
    GD_Data.fun[Radar_Pos_Frame_NUM].count_mstime = 0;  //相当于是一个初始相位，数字从几开始计数

    //雷达速度数据
    GD_Data.fun[Radar_Speed_Frame_NUM].Addr = Dev_Address;
    GD_Data.fun[Radar_Speed_Frame_NUM].fre_ms = 100;
    GD_Data.fun[Radar_Speed_Frame_NUM].count_mstime = 3;

    //视觉坐标
    GD_Data.fun[JN_Cam_Frame_NUM].Addr = Dev_Address;
    GD_Data.fun[JN_Cam_Frame_NUM].fre_ms = 100;
    GD_Data.fun[JN_Cam_Frame_NUM].count_mstime = 6;

    //雷达yaw轴
    GD_Data.fun[Radar_YAW_Frame_NUM].Addr = Dev_Address;
    GD_Data.fun[Radar_YAW_Frame_NUM].fre_ms = 100;
    GD_Data.fun[Radar_YAW_Frame_NUM].count_mstime = 9;

    //电机转速
    GD_Data.fun[Motor_Speed_Frame_NUM].Addr = Dev_Address;
    GD_Data.fun[Motor_Speed_Frame_NUM].fre_ms = 300;
    GD_Data.fun[Motor_Speed_Frame_NUM].count_mstime = 12;

    //电流、电压、高度数据
    GD_Data.fun[Batt_Curr_Height_Process_Frame_NUM].Addr = Dev_Address;
    GD_Data.fun[Batt_Curr_Height_Process_Frame_NUM].fre_ms = 300;
    GD_Data.fun[Batt_Curr_Height_Process_Frame_NUM].count_mstime = 16;

    //实时控制帧
    GD_Data.fun[Cmd_Vel_Frame_NUM].Addr = Dev_Address;
    GD_Data.fun[Cmd_Vel_Frame_NUM].fre_ms = 30;
    GD_Data.fun[Cmd_Vel_Frame_NUM].count_mstime = 19;

    //速度传感器融合数据回传
    GD_Data.fun[VEL_FU_FRAME_NUM].Addr = Dev_Address;
    GD_Data.fun[VEL_FU_FRAME_NUM].fre_ms = 30;
    GD_Data.fun[VEL_FU_FRAME_NUM].count_mstime = 21;

}

//一个字节一个字节收包
void H743_Data_Receive_From_GroundStation(u8 data)
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
        H743_Received_Data_From_GroundStation_Analysis(rx_Buffer, Data_cnt);
    }
    else {
        rxstate = 0;
        Data_len = 0;
        payload_cnt = 0;
    }
}

//H743将从地面站收过来的数据解包
static void H743_Received_Data_From_GroundStation_Analysis(const u8 *data, u8 len)
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

    //根据帧进行数据解析
    else if(*(data + 2) == 0xE0)
    {
        switch(*(data + 4))
        {
            case 0x01:
            {
            }
            break;
            case 0x02:
            {
            }
            break;
            case 0x10:
            {
                for(u8 i = 0; i < 3; i++)
                {
                    No_Fly_Zone[i].x = data[5 + i * 2];
                    No_Fly_Zone[i].y = data[6 + i * 2];
                }
            }
            break;
            case 0x11:
            {
            }
            break;
            default:
                break;
        }
        //H743收到命令后，要给地面站返回应答信息
        check_ack the_check_ack;
        the_check_ack.ID = *(data + 4);
        the_check_ack.SC = sum1_check;
        the_check_ack.AC = sum2_check;
        GD_CK_Back(HW_ALL, &the_check_ack);
    }
    //如果收到了0x00的ack返回
    else if(*(data + 2) == 0x00)
    {
        if(data[3] != 3)
        {
            return;
        }
        //判断收到的ack信息和发送的ack信息是否相等
        if((GD_Data.checksum_ok.ID == *(data + 4)) && (GD_Data.checksum_ok.SC == *(data + 5)) && (GD_Data.checksum_ok.AC == *(data + 6)))
        {
            //校验成功
            GD_Data.wait_ack = 0;
        }
    }
    //地面站要从H743读取参数(参数值固定写死为0，目前为保留接口)
    else if(*(data + 2) == 0xE1)
    {
        if(data[3] != 2)
        {
            return;
        }
        GD_Data.par_data.par_ID = GD_GetU16Le(data + 4);
        GD_Data.par_data.par_value = 0;
        //发送该参数
        GD_PAR_Back(HW_ALL, &GD_Data.par_data);
    }
    //地面站要向H743写入参数
    else if(*(data + 2) == 0xE2)
    {
        //不做什么接收动作，只是一味的发送ack
        GD_Data.ack_of_check.ID = *(data + 4);
        GD_Data.ack_of_check.SC = sum1_check;
        GD_Data.ack_of_check.AC = sum2_check;
        
        GD_CK_Back(HW_ALL, &GD_Data.ack_of_check);

    }
}

//0x00校验帧返回
void GD_CK_Back(u8 dest_addr, check_ack *ck)
{
    if(ck == NULL)
    {
        return;
    }

    GD_Data.ack_of_check = *ck;
    GD_Data.fun[0x00].Addr = dest_addr;
    GD_Data.fun[0x00].wait_to_send = 1; //标记等待发送
}

//参数返回
void GD_PAR_Back(u8 dest_addr, param *par)
{
    if(par == NULL)
    {
        return;
    }

    GD_Data.par_data = *par;
    GD_Data.fun[0xE2].Addr = dest_addr;
    GD_Data.fun[0xE2].wait_to_send = 1;
}

//H743要向串口屏发送数据
//先把需要的实际数据打包好
static void H743_To_GD_Buffer(u8 frame_num, frame_pack *pack)
{
    s16 temp_data_16;
    s32 temp_data_32;

    switch(frame_num)
    {
        case Radar_Pos_Frame_NUM:
        {
            FramePack_PutBuf(pack, Pos16_of_Radar.byte_data, 6);
            u16 temp_freq_x10 = (u16)(FreqDetector_GetFrequency(&JN_freq_detector[Data_stream_Radar_Pos]) * 10);
            FramePack_U16(pack, temp_freq_x10);
        }
        break;
        case Radar_Speed_Frame_NUM:
        {
            FramePack_PutBuf(pack, Speed_of_Radar.byte_data, 6);
            u16 temp_freq_x10 = (u16)(FreqDetector_GetFrequency(&JN_freq_detector[Data_stream_Radar_Speed]) * 10);
            FramePack_U16(pack, temp_freq_x10);
        }
        break;
        case JN_Cam_Frame_NUM:
        {
            FramePack_PutBuf(pack, Camera_Pos_data.byte_data, sizeof(Camera_Pos_data));
            u16 temp_freq_x10 = (u16)(FreqDetector_GetFrequency(&JN_freq_detector[Data_stream_cam_loc]) * 10);
            FramePack_U16(pack, temp_freq_x10);
        }
        break;
        case Radar_YAW_Frame_NUM:
        {
            FramePack_PutBuf(pack, Radar_YAW_tar_un.byte_data, 2);
            u16 temp_freq_x10 = (u16)(FreqDetector_GetFrequency(&JN_freq_detector[Data_stream_Radar_Yaw]) * 10);
            FramePack_U16(pack, temp_freq_x10);
        }
        break;
        case Motor_Speed_Frame_NUM:
        {
            value_of_pwm.st_data = pwm_to_esc;
            FramePack_PutBuf(pack, value_of_pwm.byte_data, 16);

        }
        break;
        case Batt_Curr_Height_Process_Frame_NUM:
        {
            FramePack_PutBuf(pack, un_of_Batt_Height_Process.byte_data, 9);

        }
        break;
        case Cmd_Vel_Frame_NUM:
        {
            FramePack_PutBuf(pack, ctrl_of_realtime.byte, 14);
            u16 temp_freq_x10 = (u16)(FreqDetector_GetFrequency(&JN_freq_detector[Data_stream_Radar_cmd_vel]) * 10);
            FramePack_U16(pack, temp_freq_x10);
        }
        break;
        case VEL_FU_FRAME_NUM:
        {
            FramePack_PutBuf(pack, un_of_Vel_Fu.byte_data, sizeof(un_of_Vel_Fu));
            u16 temp_freq_x10 = (u16)(FreqDetector_GetFrequency(&JN_freq_detector[Data_stream_Vel_Fu]) * 10);
            FramePack_U16(pack, temp_freq_x10);
        }
        break;
        case 0x00:
        {
            FramePack_PutByte(pack, GD_Data.ack_of_check.ID);
            FramePack_PutByte(pack, GD_Data.ack_of_check.SC);
            FramePack_PutByte(pack, GD_Data.ack_of_check.AC);
        }
        break;
        case 0xe0://Cmd命令帧
        {
            FramePack_PutByte(pack, GD_Data.cmd_send.CID);
            FramePack_PutBuf(pack, GD_Data.cmd_send.CMD, 10);

        }
        break;
        case 0xe2://参数写入，参数读取完后的返回
        {
            temp_data_16 = GD_Data.par_data.par_ID;
            FramePack_U16(pack, temp_data_16);
            temp_data_32 = GD_Data.par_data.par_value;
            FramePack_U32(pack, temp_data_32);
        }
        break;
        default:
            break;
    }
}

//H743:我要组帧发给地面站了
static u8 H743_To_GD_FrameSend(u8 frame_num, Data_Frame *frame)
{
    frame_pack pack;
    u8 check_sum1 = 0;
    u8 check_sum2 = 0;

    if(frame == NULL)
    {
        return 0;
    }
    FramePack_Init(&pack, send_buffer, sizeof(send_buffer));
    FramePack_PutByte(&pack, 0XAA);
    FramePack_PutByte(&pack, frame->Addr);
    FramePack_PutByte(&pack, frame_num);
    FramePack_PutByte(&pack, 0);

    H743_To_GD_Buffer(frame_num, &pack);
    send_buffer[3] = (u8)(pack.len - 4);

    for(u16 i = 0; i < pack.len; i++)
    {
        check_sum1 += send_buffer[i];
        check_sum2 += check_sum1;
    }

    FramePack_PutByte(&pack, check_sum1);
    FramePack_PutByte(&pack, check_sum2);

    if(GD_Data.wait_ack != 0 && frame_num == 0XE0)
    {
        GD_Data.checksum_ok.ID = frame_num;
        GD_Data.checksum_ok.SC = check_sum1;
        GD_Data.checksum_ok.AC = check_sum2;
    }
    return H743_To_GD_Send_Data(send_buffer, pack.len);

}

static u8 H743_To_GD_Send_Data(u8 *data, u8 length)
{
    return 
}

