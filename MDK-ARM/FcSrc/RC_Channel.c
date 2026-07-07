#include "RC_Channel.h"
#include "Remote_Control.h"
#include "ANO_TO_H743_Data_Transmit.h"
#include "PWM.h"
#include "adc.h"
#include "To_LX_Fun.h"

#define RC_MID_VALUE 1500
#define RC_LOW_VALUE 1000

#define RC_DEADZONE_ROLL_PITCH 40
#define RC_DEADZONE_THR_YAW 80

#define MAX_ANGLE 3500
#define MAX_YAW_DPS 200
#define FAILSAFE_THROTTLE 350
#define STICK_LOW 1150
#define STICK_HIGH 1850
#define ARM_HOLD_MS 500
#define UNLOCK_CMD_RETRY_MS 100
#define MODE_CH_LOW_LIMIT 1200
#define MODE_CH_HIGH_LIMIT 1700

static u8 motor_unlocked;
static u16 arm_hold_ms;
static u16 unlock_cmd_retry_ms;

static u8 FlightModeFromChannel(s16 channel)
{
    if(channel < MODE_CH_LOW_LIMIT)
    {
        return 1;
    }

    if(channel < MODE_CH_HIGH_LIMIT)
    {
        return 2;
    }

    return 3;
}

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

static s16 Deadzone(s16 value, s16 deadzone)
{
    if(value > deadzone)
    {
        return value - deadzone;
    }

    if(value < -deadzone)
    {
        return value + deadzone;
    }

    return 0;
}

static SwitchState SwitchStateFromChannel(s16 channel)
{
    if(channel < 1200)
    {
        return Switch_Low;
    }

    if(channel < 1700)
    {
        return Switch_Mid;
    }

    return Switch_High;
}

static u32 ReadAdcOnce(ADC_HandleTypeDef *hadc)
{
    u32 value = 0;

    if(HAL_ADC_Start(hadc) == HAL_OK)
    {
        if(HAL_ADC_PollForConversion(hadc, 2) == HAL_OK)
        {
            value = HAL_ADC_GetValue(hadc);
        }

        HAL_ADC_Stop(hadc);
    }

    return value;
}

static void SyncSwitchState(void)
{
    Switch_sta_st.SWA = SwitchStateFromChannel(Channel_of_rc.data.ch[ch_7_aux3]);
    Switch_sta_st.SWB = SwitchStateFromChannel(Channel_of_rc.data.ch[ch_6_aux2]);
    Switch_sta_st.SWC = SwitchStateFromChannel(Channel_of_rc.data.ch[ch_5_aux1]);
    Switch_sta_st.SWD = SwitchStateFromChannel(Channel_of_rc.data.ch[ch_8_aux4]);
    Switch_sta_st.VRA = SwitchStateFromChannel(Channel_of_rc.data.ch[ch_9_aux5]);
    Switch_sta_st.VRB = SwitchStateFromChannel(Channel_of_rc.data.ch[ch_10_aux6]);
}

static u8 IsStickArmCommand(void)
{
    return (Channel_of_rc.data.ch[ch_3_thr] < STICK_LOW) &&
           (Channel_of_rc.data.ch[ch_4_yaw] > STICK_HIGH) &&
           (Channel_of_rc.data.ch[ch_1_rol] < STICK_LOW) &&
           (Channel_of_rc.data.ch[ch_2_pit] < STICK_LOW);
}

static u8 IsStickDisarmCommand(void)
{
    return (Channel_of_rc.data.ch[ch_3_thr] < STICK_LOW) &&
           (Channel_of_rc.data.ch[ch_4_yaw] < STICK_LOW) &&
           (Channel_of_rc.data.ch[ch_1_rol] > STICK_HIGH) &&
           (Channel_of_rc.data.ch[ch_2_pit] < STICK_LOW);
}

static void RcUnlockTask(float dT_s)
{
    u16 dt_ms = (u16)(dT_s * 1000.0f);

    if(dt_ms == 0)
    {
        dt_ms = 1;
    }

    if(RemoteControl_IsSignalLost())
    {
        motor_unlocked = 0;
        arm_hold_ms = 0;
        unlock_cmd_retry_ms = 0;
        return;
    }

    if(IsStickDisarmCommand())
    {
        motor_unlocked = 0;
        arm_hold_ms = 0;
        unlock_cmd_retry_ms = 0;
        return;
    }

    if(!motor_unlocked && IsStickArmCommand())
    {
        if(arm_hold_ms < ARM_HOLD_MS)
        {
            arm_hold_ms += dt_ms;
        }
        else
        {
            motor_unlocked = 1;
            unlock_cmd_retry_ms = UNLOCK_CMD_RETRY_MS;
            FC_Unlock();
        }
    }
    else
    {
        arm_hold_ms = 0;
    }

    if(motor_unlocked)
    {
        if(unlock_cmd_retry_ms < UNLOCK_CMD_RETRY_MS)
        {
            unlock_cmd_retry_ms += dt_ms;
        }
        else
        {
            unlock_cmd_retry_ms = 0;
            FC_Unlock();
        }
    }
}

u8 RC_MotorIsUnlocked(void)
{
    return motor_unlocked;
}

void RC_MotorForceLock(void)
{
    motor_unlocked = 0;
    arm_hold_ms = 0;
    unlock_cmd_retry_ms = 0;
}

void RC_Data_Task(float dT_s)
{
    s16 roll;
    s16 pitch;
    s16 throttle;
    s16 yaw;

    SyncSwitchState();
    RcUnlockTask(dT_s);

    if(RemoteControl_IsSignalLost())
    {
        ctrl_of_realtime.data.roll = 0;
        ctrl_of_realtime.data.pitch = 0;
        ctrl_of_realtime.data.throttle = FAILSAFE_THROTTLE;
        ctrl_of_realtime.data.yaw_dps = 0;
        ctrl_of_realtime.data.vel_x = 0;
        ctrl_of_realtime.data.vel_y = 0;
        ctrl_of_realtime.data.vel_z = 0;
        Data.fun[0x41].wait_to_send = 1;
        return;
    }

    LX_Change_Mode(FlightModeFromChannel(Channel_of_rc.data.ch[ch_5_aux1]));

    roll = Deadzone(Channel_of_rc.data.ch[ch_1_rol] - RC_MID_VALUE, RC_DEADZONE_ROLL_PITCH);
    pitch = Deadzone(Channel_of_rc.data.ch[ch_2_pit] - RC_MID_VALUE, RC_DEADZONE_ROLL_PITCH);
    throttle = Channel_of_rc.data.ch[ch_3_thr] - RC_LOW_VALUE;
    yaw = Deadzone(Channel_of_rc.data.ch[ch_4_yaw] - RC_MID_VALUE, RC_DEADZONE_THR_YAW);

    throttle = LimitS16(throttle, 0, 1000);

    if(!motor_unlocked)
    {
        roll = 0;
        pitch = 0;
        throttle = 0;
        yaw = 0;
    }

    ctrl_of_realtime.data.roll = (s16)((s32)roll * MAX_ANGLE / 460);
    ctrl_of_realtime.data.pitch = (s16)(-((s32)pitch * MAX_ANGLE / 460));
    ctrl_of_realtime.data.throttle = throttle;
    ctrl_of_realtime.data.yaw_dps = (s16)(-((s32)yaw * MAX_YAW_DPS / 420));
    ctrl_of_realtime.data.vel_x = 0;
    ctrl_of_realtime.data.vel_y = 0;
    ctrl_of_realtime.data.vel_z = 0;

    Data.fun[0x41].wait_to_send = 1;
}

void ESC_Output(u8 unlocked)
{
    int16_t pwm[MOTOR_NUM] = {0};

    if(unlocked)
    {
        pwm[0] = LimitS16((s16)(pwm_to_esc.pwm_value1 / 5), 0, 2000);
        pwm[1] = LimitS16((s16)(pwm_to_esc.pwm_value2 / 5), 0, 2000);
        pwm[2] = LimitS16((s16)(pwm_to_esc.pwm_value3 / 5), 0, 2000);
        pwm[3] = LimitS16((s16)(pwm_to_esc.pwm_value4 / 5), 0, 2000);
        pwm[4] = LimitS16((s16)(pwm_to_esc.pwm_value5 / 5), 0, 2000);
        pwm[5] = LimitS16((s16)(pwm_to_esc.pwm_value6 / 5), 0, 2000);
        pwm[6] = LimitS16((s16)(pwm_to_esc.pwm_value7 / 5), 0, 2000);
        pwm[7] = LimitS16((s16)(pwm_to_esc.pwm_value8 / 5), 0, 2000);
    }

    DrvMotorPWMSet(pwm);
}

void Bat_Voltage_Data_Handle(void)
{
    u32 raw = ReadAdcOnce(&hadc1);

    union_of_bat.data_of_bat.voltage_100 = (u16)((raw * 330UL) / 65535UL);
}

void Bat_Curr_Data_Handle(void)
{
    u32 raw = ReadAdcOnce(&hadc2);

    union_of_bat.data_of_bat.current_100 = (u16)((raw * 330UL) / 65535UL);
}


