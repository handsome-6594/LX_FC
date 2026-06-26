#include "ANO_TO_H743_Data_Transmit.h"
#include "FC_State.h"

Data_Transmit Data;
attitude_quat LX_quat;
pwm_value pwm_to_esc;
raw_speed_union current_speed;
altitude_option_un altitude_of_option;

static u8 rx_Buffer[256];
static u8 Data_cnt = 0;

static void H743_Received_Data_Analysis(const u8 *data, u8 len);

void Data_Init(void)
{
    //电池电压数据3
    Data.fun[0x0d].Addr = 0xFF;
    Data.fun[0x0d].fre_ms = 100;
    Data.fun[0x0d].count_mstime = 1;

    //遥控器数据
    Data.fun[0x0d].Addr = 0xFF;
    Data.fun[0x0d].fre_ms = 20;
    Data.fun[0x0d].count_mstime = 1;  

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

void H743_Data_Receive(u8 data)
{
    static u8 Data_len = 0, payload_cnt = 0;
    static u8 rxstate = 0;
    
    if(rxstate == 0 && data == 0xAA)
    {
        rxstate = 1;
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
    }
}

static u16 get_ui6_le(const u8 *p)
{
    return (u16)p[0] | 
}


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
        pwm_to_esc.pwm_value1 = *((u16 *)(data + 4));
        pwm_to_esc.pwm_value2 = *((u16 *)(data + 6));
        pwm_to_esc.pwm_value3 = *((u16 *)(data + 8));
        pwm_to_esc.pwm_value4 = *((u16 *)(data + 10));
        pwm_to_esc.pwm_value5 = *((u16 *)(data + 12));
        pwm_to_esc.pwm_value6 = *((u16 *)(data + 14));
        pwm_to_esc.pwm_value7 = *((u16 *)(data + 16));
        pwm_to_esc.pwm_value8 = *((u16 *)(data + 18));       
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
    
    // //姿态四元数
    // if(*(data + 2) == 0x04)
    // {
    //     LX_quat.quat_w_10000 = *(data + 4);
    //     LX_quat.quat_x_10000 = *(data + 5);
    //     LX_quat.quat_y_10000 = *(data + 6);
    //     LX_quat.quat_z_10000 = *(data + 7);
    //     LX_quat.fusion_sta = *(data + 8);
       
    // } 
    

}



