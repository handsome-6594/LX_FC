#include "ANO_TO_H743_Data_Transmit.h"
#include "FC_State.h"
#include "JetsonNano_Data_Transmit.h"
#include "Of_Radar_Fusion.h"
#include "Remote_Control.h"
#include "Drv_Uart.h"

Data_Transmit Data;
attitude_quat LX_quat;
volatile u8 pwm_update_flag = 0;
pwm_value pwm_to_esc;
raw_speed_union current_speed;
altitude_option_un altitude_of_option;
Gyro_Sensor Gyro_acc_sense;
check_ack the_check_ack;
param param_aux;
frame_pack pack;
bat_union union_of_bat;


static u8 rx_Buffer[256];
static u8 Data_cnt = 0;
u8 tx_buf[256];
static u8 send_buffer[50];


static void H743_Received_Data_Analysis(const u8 *data, u8 len);
static void H743_To_LX_Send_Data(u8 *data, u8 length);
void Check_ack(u8 goal_addr, check_ack *ck);
void param_back(u8 goal_addr, param *para);

void FramePack_Init(frame_pack *pack, u8 *buf, u16 max_len);
void FramePack_PutByte(frame_pack *pack, u8 data);
void FramePack_PutBuf(frame_pack *pack, uc8 *data, u16 len);
void FramePack_U16(frame_pack *pack, u16 data);
void FramePack_U32(frame_pack *pack, u32 data);


void Data_Init(void)
{

    FramePack_Init(&pack, tx_buf, 256);

    //电池电压数据
    Data.fun[0x0d].Addr = 0xFF;
    Data.fun[0x0d].fre_ms = 0;
    Data.fun[0x0d].count_mstime = 0;

    //遥控器数据
    Data.fun[0x40].Addr = 0xFF;
    Data.fun[0x40].fre_ms = 20;
    Data.fun[0x40].count_mstime = 1;  

    //通用速度型传感器信息
    Data.fun[0x33].Addr = 0xFF;
    Data.fun[0x33].fre_ms = 0;
    Data.fun[0x33].count_mstime = 0;
    
    //通用测距传感器信息
    Data.fun[0x34].Addr = 0xFF;
    Data.fun[0x34].fre_ms = 0;
    Data.fun[0x34].count_mstime = 0;

    //实时控制帧（也就是期望飞机角度的目标值，发给凌霄imu，由凌霄imu处理）
    Data.fun[0x41].Addr = 0xFF;
    Data.fun[0x41].fre_ms = 0;
    Data.fun[0x41].count_mstime = 0;   

    //cmd命令帧
    Data.fun[0xE0].Addr = 0xFF;
    Data.fun[0xE0].fre_ms = 0;
    Data.fun[0xE0].count_mstime = 0; 

	//参数写入，读取，返回
    Data.fun[0xE2].Addr = 0xFF;
    Data.fun[0xE2].fre_ms = 0;
    Data.fun[0xE2].count_mstime = 0;   
}

//接收数据组包并解析
void H743_Data_Receive(u8 data)
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
        H743_Received_Data_Analysis(rx_Buffer, Data_cnt);
    }
    else {
        rxstate = 0;
        Data_len = 0;
        payload_cnt = 0;
    }
}
/// @brief    解析数据将其转化为合适的形式
/// @param p 
/// @return 
static inline u16 get_u16_le(const u8 *p)
{
    return (u16)p[0] | ((u16)p[1] << 8);
}

static inline s16 get_s16_le(const u8 *p)
{
    return (s16)get_u16_le(p);
}

// static inline u32 get_u32_le(const u8 *p)
// {
//     return (u32)p[0] | 
//                     ((u32)p[1] << 8) | 
//                     ((u32)p[2] << 16) | 
//                     ((u32)p[3] << 24);
// }

// static inline s32 get_s32_le(const u8 *p)
// {
//     return (s32)get_u32_le(p);
// }
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

static void H743_Received_Data_Analysis(const u8 *data, u8 len)
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

    //解析对应数据
    //PWM数据
    if(*(data + 2) == 0X20)
    {
        pwm_to_esc.pwm_value1 = get_u16_le(data + 4);
        pwm_to_esc.pwm_value2 = get_u16_le(data + 6);
        pwm_to_esc.pwm_value3 = get_u16_le(data + 8);
        pwm_to_esc.pwm_value4 = get_u16_le(data + 10);
        pwm_to_esc.pwm_value5 = get_u16_le(data + 12);
        pwm_to_esc.pwm_value6 = get_u16_le(data + 14);
        pwm_to_esc.pwm_value7 = get_u16_le(data + 16);
        pwm_to_esc.pwm_value8 = get_u16_le(data + 18);     
        
        pwm_update_flag = 1;
    }

    //凌霄IMU发出的RGB灯光数据
    else if(*(data + 2) == 0X0F)
    {
        Led_LX.brightness[0] = *(data + 4);
        Led_LX.brightness[1] = *(data + 5);
        Led_LX.brightness[2] = *(data + 6);
        Led_LX.brightness[3] = *(data + 7);
    }
    //凌霄飞控当前的运行模式
    else if(*(data + 2) == 0X06)
    {
        state.mode = *(data + 4);
        state.is_unlocked = *(data + 5);
        state.cmd.CID = *(data + 6);
        state.cmd.Cmd_0 = *(data + 7);
        state.cmd.Cmd_1 = *(data + 8);
    }
    //飞行速度
    else if(*(data + 2) == 0X07)
    {
        for(u8 i = 0; i < 6; i++)
        {
            current_speed.byte_data[i] = *(data + 4 + i);           
        }
    }
    //高度数据
    else if(*(data + 2) == 0X05)
    {
        for(u8 i = 0; i < 9; i++)
        {
            altitude_of_option.byte_data[i] = *(data + 4 + i);
        }
    }

    //传感器数据
    else if(*(data + 2) == 0X01)
    {
        Gyro_acc_sense.acc_gyro_data.acc_x = get_s16_le(data + 4);
        Gyro_acc_sense.acc_gyro_data.acc_y = get_s16_le(data + 6);
        Gyro_acc_sense.acc_gyro_data.acc_z = get_s16_le(data + 8);
        Gyro_acc_sense.acc_gyro_data.gyr_x = get_s16_le(data + 10);
        Gyro_acc_sense.acc_gyro_data.gyr_y = get_s16_le(data + 12);
        Gyro_acc_sense.acc_gyro_data.gyr_z = get_s16_le(data + 14);
        Gyro_acc_sense.acc_gyro_data.shock_sta = *(data + 16);
    }
    
    //姿态四元数
    else if(*(data + 2) == 0x04)
    {
        LX_quat.quat_w_10000 = get_s16_le(data + 4);
        LX_quat.quat_x_10000 = get_s16_le(data + 6);
        LX_quat.quat_y_10000 = get_s16_le(data + 8);
        LX_quat.quat_z_10000 = get_s16_le(data + 10);
        LX_quat.fusion_sta = *(data + 12);
        
    }
    //CMD命令帧
    else if(*(data + 2) == 0XE0)
    {
        switch(*(data + 4))
        {
            case 0X01:
            {
                break;
            }
            case 0X10:
            {
                break;
            }
            default:
            {
                break;
            }
        }
        //飞控收到本帧后，需返回校验信息，即返回帧 ID=0x00 的校验帧。
        the_check_ack.ID = *(data + 4);
        the_check_ack.SC = sum1_check;
        the_check_ack.AC = sum2_check;
        Check_ack(SWJ_ADDR, &the_check_ack);      
    }
    //收到ck返回值
    else if(*(data + 2) == 0X00)
    {
        if(Data.checksum_ok.ID == *(data + 4) && Data.checksum_ok.SC == *(data + 5) && Data.checksum_ok.AC == *(data + 6))
        {
            Data.cmd_wait_ack = 0;
        }
    }
    //凌霄IMU想读取参数，H743返回
    else if(*(data + 2) == 0XE1)
    {
        u16 par_Id = get_u16_le(&data[4]);
        param_aux.par_ID = par_Id;
        param_aux.par_value = 0;
        //H743返回参数0，目前没有实际意义
        param_back(0XFF, &param_aux);
    }
    //凌霄IMU向H743写参数，没什么好写的，H743只一味的回复ACK
    else if(*(data + 2) == 0XE2)
    {
        the_check_ack.ID = *(data + 4);
        the_check_ack.SC = sum1_check;
        the_check_ack.AC = sum2_check;
        Check_ack(0XFF, &the_check_ack);
    }   
}

//check返回的帧
void Check_ack(u8 goal_addr, check_ack *ck)
{
    if(ck == NULL)
        return;

    Data.ack_of_check = *ck;
    Data.fun[0X00].Addr = goal_addr;
    Data.fun[0X00].wait_to_send = 1;
}

//返回参数信息
void param_back(u8 goal_addr, param *para)
{
    if(para == NULL)
        return;
    
    Data.param_data = *para;
    Data.fun[0XE2].Addr = goal_addr;
    Data.fun[0XE2].wait_to_send = 1;
}
///////////////////发送函数准备字节的过程///////////////////////////
//初始化那个结构体
void FramePack_Init(frame_pack *pack, u8 *buf, u16 max_len)
{
    pack->buf = buf;
    pack->len = 0;
    pack->max_len = max_len;
}

//存一个字节
void FramePack_PutByte(frame_pack *pack, u8 data)
{
    if(pack->len < pack->max_len)
    {
        pack->buf[pack->len++] = data;
    }
}
//存多个字节
void FramePack_PutBuf(frame_pack *pack, uc8 *data, u16 len)
{
    if(pack->len + len <= pack->max_len)
    {
        for(u16 i = 0; i < len; i++)
        {
            pack->buf[pack->len++] = data[i];
        }
    }
}
//u16小端
void FramePack_U16(frame_pack *pack, u16 data)
{
    FramePack_PutByte(pack, (u8)(data & 0XFF));
    FramePack_PutByte(pack, (u8)(data >> 8));
}

//u32小端
void FramePack_U32(frame_pack *pack, u32 data)
{
    for(u8 i = 0; i < 4; i++)
    {
        FramePack_PutByte(pack, (u8)(data >> (8 * i)));
    }
}

///////////////////////////////////////////////////////
//H743给凌霄IMU发送数据：高度，速度等，塞给凌霄让他算
///////////////////////////////////////////////////////
static void Send_Data_Buffer(u8 frame_num, frame_pack *pack)
{
    s16 temp_data_16;
    s32 temp_data_32;

    switch(frame_num)
    {
        case 0X00://返回那些需要0x00校验帧的
        {
            FramePack_PutByte(pack, the_check_ack.ID);
            FramePack_PutByte(pack, the_check_ack.SC);
            FramePack_PutByte(pack, the_check_ack.AC);
        }
        break;  
        case 0X0D://ADC采集到的电池电压和电流
        {
            FramePack_PutBuf(pack, union_of_bat.byte_data, 4);
        }
        break;
        case 0X30://GPS数据
        {
            FramePack_PutBuf(pack, ex_sensor.gps.byte_data, 23);
        }
        break;
        case 0X32://雷达位置数据
        {
            FramePack_PutBuf(pack, Pos_of_Radar.byte_data, 12);
        }
        break;
        case 0X33://H743融合通用速度传感器数据给凌霄
        {
            FramePack_PutBuf(pack, ex_sensor.vel_general.byte, 6);
        }
        break;
        case 0X34://H743 给凌霄IMU测距的信息
        {
            FramePack_PutBuf(pack, ex_sensor.distance_general.byte, 7);
        }
        break;

        case 0X40://遥控器通道的数据
        {
            FramePack_PutBuf(pack, Channel_of_rc.byte,20);
        }
        break;

        case 0X41://实时控制帧
        {
            FramePack_PutBuf(pack, ctrl_of_realtime.byte, 14);
        }
        break;

        case 0XE0://CMD命令帧
        {
            FramePack_PutByte(pack, Data.data_of_cmd.CID);
            FramePack_PutBuf(pack, Data.data_of_cmd.CMD, 10);
        }
        break;

        case 0XE2: //H743往凌霄IMU里面写参数
        {
            temp_data_16 = Data.param_data.par_ID;
            FramePack_U16(pack, temp_data_16);
            temp_data_32 = Data.param_data.par_value;
            FramePack_U32(pack, temp_data_32);
        }
        break;

        default:
            break;
    }
}

//==========================================================
//H743:我要组帧发给凌霄喽
static void H743_To_LX_FrameSend(u8 frame_num, Data_Frame *frame)
{
    frame_pack pack;
    u8 check_sum1 = 0;
    u8 check_sum2 = 0;
    u16 data_len = 0;

    FramePack_Init(&pack, send_buffer, sizeof(send_buffer));

    //帧头
    FramePack_PutByte(&pack, 0XAA);

    //目标地址
    FramePack_PutByte(&pack, frame->Addr);

    //功能码
    FramePack_PutByte(&pack, frame_num);

    //数据长度先初始化为0
    FramePack_PutByte(&pack, 0);

    //填充数据区
    Send_Data_Buffer(frame_num, &pack);

    //数据区长度= 当前长度 - 4
    data_len = pack.len - 4;
    send_buffer[3] = (u8)data_len;

    //计算校验
    for(u16 i = 0; i < pack.len; i++)
    {
        check_sum1 += send_buffer[i];
        check_sum2 += check_sum1;
    }

    //添加校验字节
    FramePack_PutByte(&pack, check_sum1);
    FramePack_PutByte(&pack, check_sum2);

    //如果是cmd命令帧，要记录一下两个校验位，等凌霄IMU返回0x00的校验帧来了之后进行校验
    if(Data.cmd_wait_ack != 0 && frame_num == 0XE0)
    {
        Data.checksum_ok.ID = frame_num;
        Data.checksum_ok.SC = check_sum1;
        Data.checksum_ok.AC = check_sum2;
    }

    H743_To_LX_Send_Data(send_buffer, pack.len);

}

static void H743_To_LX_Send_Data(u8 *data, u8 length)
{
    DrvUart4SendBuf(data, length);
}

static void LX_Check_To_Send(u8 frame_num)
{
    if(Data.fun[frame_num].wait_to_send)
    {
        Data.fun[frame_num].wait_to_send = 0;
        H743_To_LX_FrameSend(frame_num, &Data.fun[frame_num]);
    }
    else if(Data.fun[frame_num].fre_ms != 0)
    {
        if(Data.fun[frame_num].count_mstime >= Data.fun[frame_num].fre_ms)
        {
            Data.fun[frame_num].count_mstime = 1;
            H743_To_LX_FrameSend(frame_num, &Data.fun[frame_num]);
        }
        else
        {
            Data.fun[frame_num].count_mstime++;
        }
    }
}

void H743_Data_Transmit_Check(void)
{
    LX_Check_To_Send(0x40); // remote control data
}


                                            
