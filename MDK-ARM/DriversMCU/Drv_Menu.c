#include "Drv_Menu.h"
#include "Adaptive_PID.h"
#include "JetsonNano_Data_Transmit.h"
#include "Of_Radar_Fusion.h"
#include "Optical_Flow_Sensor.h"
#include "Key.h"
#include "OLED.h"

#define DRV_MENU_PARAM_COUNT       9U
#define DRV_MENU_PID_ITEM_COUNT    (DRV_MENU_PARAM_COUNT + 1U)
#define DRV_MENU_MAIN_ITEM_COUNT   2U
#define DRV_MENU_VISIBLE_ITEMS     5U
#define DRV_MENU_REFRESH_MS        100U

#define MENU_PID_ITEM_AXIS         0U
#define MENU_PID_ITEM_TO_PARAM(item) ((u8)((item) - 1U))

typedef enum
{
    MENU_PARAM_KP = 0,
    MENU_PARAM_KI,
    MENU_PARAM_KD,
    MENU_PARAM_OUT_MIN,
    MENU_PARAM_OUT_MAX,
    MENU_PARAM_INT_MIN,
    MENU_PARAM_INT_MAX,
    MENU_PARAM_DEADBAND,
    MENU_PARAM_D_ALPHA,
} DrvMenuParam_e;

typedef enum
{
    MENU_PAGE_MAIN = 0,
    MENU_PAGE_PID,
    MENU_PAGE_RADAR,
} DrvMenuPage_e;

typedef struct
{
    Radar_Pos_16 pos;
    s32 display_z_x100;
    u8 display_alt_source;
    s16 yaw_x100;
    u8 pos_update_cnt;
    u8 qua_update_cnt;
    u8 yaw_update_cnt;
} DrvMenuRadarSnapshot_t;

static u8 menu_page = MENU_PAGE_MAIN;
static u8 menu_main_item = 0;
static u8 menu_axis = PID_X;
static u8 menu_pid_item = MENU_PID_ITEM_AXIS;
static u8 menu_editing = 0;
static u8 menu_redraw = 1;
static u32 menu_last_draw_ms = 0;
static Key_Status_t menu_last_key = {0};

static const char * const menu_axis_name[PID_NUM] = {
    "X",
    "Y",
    "Z",
    "YAW",
};

static const char * const menu_param_name[DRV_MENU_PARAM_COUNT] = {
    "KP",
    "KI",
    "KD",
    "OUTMIN",
    "OUTMAX",
    "INTMIN",
    "INTMAX",
    "DEAD",
    "DALPHA",
};

static const char * const menu_main_item_name[DRV_MENU_MAIN_ITEM_COUNT] = {
    "PID TUNE",
    "RADAR POS",
};

static float DrvMenu_GetStep(u8 param)
{
    switch(param)
    {
        case MENU_PARAM_KP:
        case MENU_PARAM_KI:
        case MENU_PARAM_KD:
            return 0.01f;

        case MENU_PARAM_D_ALPHA:
            return 0.05f;

        case MENU_PARAM_DEADBAND:
            return 0.5f;

        case MENU_PARAM_OUT_MIN:
        case MENU_PARAM_OUT_MAX:
        case MENU_PARAM_INT_MIN:
        case MENU_PARAM_INT_MAX:
        default:
            return 1.0f;
    }
}

static float DrvMenu_LimitFloat(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }

    if(value > max_value)
    {
        return max_value;
    }

    return value;
}

static float *DrvMenu_GetParamPtr(u8 axis, u8 param)
{
    PID_t *pid;

    if(axis >= PID_NUM)
    {
        return 0;
    }

    pid = &loc_pid[axis];

    switch(param)
    {
        case MENU_PARAM_KP:
            return &pid->kp;

        case MENU_PARAM_KI:
            return &pid->ki;

        case MENU_PARAM_KD:
            return &pid->kd;

        case MENU_PARAM_OUT_MIN:
            return &pid->output_min;

        case MENU_PARAM_OUT_MAX:
            return &pid->output_max;

        case MENU_PARAM_INT_MIN:
            return &pid->integral_min;

        case MENU_PARAM_INT_MAX:
            return &pid->integral_max;

        case MENU_PARAM_DEADBAND:
            return &pid->deadband;

        case MENU_PARAM_D_ALPHA:
            return &pid->d_filter_alpha;

        default:
            return 0;
    }
}

static void DrvMenu_FixPidLimitPair(u8 param)
{
    PID_t *pid = &loc_pid[menu_axis];

    if(param == MENU_PARAM_OUT_MIN && pid->output_min > pid->output_max)
    {
        pid->output_min = pid->output_max;
    }
    else if(param == MENU_PARAM_OUT_MAX && pid->output_max < pid->output_min)
    {
        pid->output_max = pid->output_min;
    }
    else if(param == MENU_PARAM_INT_MIN && pid->integral_min > pid->integral_max)
    {
        pid->integral_min = pid->integral_max;
    }
    else if(param == MENU_PARAM_INT_MAX && pid->integral_max < pid->integral_min)
    {
        pid->integral_max = pid->integral_min;
    }
}

static void DrvMenu_SelectNextAxis(void)
{
    menu_axis++;
    if(menu_axis >= PID_NUM)
    {
        menu_axis = PID_X;
    }

    menu_redraw = 1;
}

static void DrvMenu_SelectPrevAxis(void)
{
    if(menu_axis == PID_X)
    {
        menu_axis = PID_NUM - 1U;
    }
    else
    {
        menu_axis--;
    }

    menu_redraw = 1;
}

static void DrvMenu_ChangePidItem(s8 dir)
{
    float *value;
    float step;
    u8 param;

    if(menu_pid_item == MENU_PID_ITEM_AXIS)
    {
        if(dir > 0)
        {
            DrvMenu_SelectNextAxis();
        }
        else
        {
            DrvMenu_SelectPrevAxis();
        }
        return;
    }

    param = MENU_PID_ITEM_TO_PARAM(menu_pid_item);
    value = DrvMenu_GetParamPtr(menu_axis, param);
    if(value == 0)
    {
        return;
    }

    step = DrvMenu_GetStep(param);
    *value += (dir > 0) ? step : -step;

    switch(param)
    {
        case MENU_PARAM_KP:
        case MENU_PARAM_KI:
        case MENU_PARAM_KD:
            *value = DrvMenu_LimitFloat(*value, 0.0f, 50.0f);
            break;

        case MENU_PARAM_OUT_MIN:
        case MENU_PARAM_OUT_MAX:
        case MENU_PARAM_INT_MIN:
        case MENU_PARAM_INT_MAX:
            *value = DrvMenu_LimitFloat(*value, -500.0f, 500.0f);
            DrvMenu_FixPidLimitPair(param);
            break;

        case MENU_PARAM_DEADBAND:
            *value = DrvMenu_LimitFloat(*value, 0.0f, 200.0f);
            break;

        case MENU_PARAM_D_ALPHA:
            *value = DrvMenu_LimitFloat(*value, 0.0f, 1.0f);
            break;

        default:
            break;
    }

    menu_redraw = 1;
}

static void DrvMenu_SelectPrevWrap(u8 *item, u8 count)
{
    if(item == 0 || count == 0U)
    {
        return;
    }

    if(*item == 0)
    {
        *item = count - 1U;
    }
    else
    {
        (*item)--;
    }

    menu_redraw = 1;
}

static void DrvMenu_SelectNextWrap(u8 *item, u8 count)
{
    if(item == 0 || count == 0U)
    {
        return;
    }

    (*item)++;
    if(*item >= count)
    {
        *item = 0;
    }

    menu_redraw = 1;
}

static void DrvMenu_EnterPage(u8 page)
{
    menu_page = page;
    menu_editing = 0;
    menu_redraw = 1;
}

static void DrvMenu_HandleMainKeys(u8 up_press, u8 down_press, u8 enter_press)
{
    if(up_press)
    {
        DrvMenu_SelectPrevWrap(&menu_main_item, DRV_MENU_MAIN_ITEM_COUNT);
    }

    if(down_press)
    {
        DrvMenu_SelectNextWrap(&menu_main_item, DRV_MENU_MAIN_ITEM_COUNT);
    }

    if(enter_press)
    {
        if(menu_main_item == 0U)
        {
            DrvMenu_EnterPage(MENU_PAGE_PID);
        }
        else
        {
            DrvMenu_EnterPage(MENU_PAGE_RADAR);
        }
    }
}

static void DrvMenu_HandlePidKeys(u8 up_press, u8 down_press, u8 enter_press, u8 back_press)
{
    if(enter_press)
    {
        menu_editing = (menu_editing == 0U) ? 1U : 0U;
        menu_redraw = 1;
    }

    if(back_press)
    {
        if(menu_editing)
        {
            menu_editing = 0;
            menu_redraw = 1;
        }
        else
        {
            DrvMenu_EnterPage(MENU_PAGE_MAIN);
        }
    }

    if(up_press)
    {
        if(menu_editing)
        {
            DrvMenu_ChangePidItem(1);
        }
        else
        {
            DrvMenu_SelectPrevWrap(&menu_pid_item, DRV_MENU_PID_ITEM_COUNT);
        }
    }

    if(down_press)
    {
        if(menu_editing)
        {
            DrvMenu_ChangePidItem(-1);
        }
        else
        {
            DrvMenu_SelectNextWrap(&menu_pid_item, DRV_MENU_PID_ITEM_COUNT);
        }
    }
}

static void DrvMenu_HandleRadarKeys(u8 back_press)
{
    if(back_press)
    {
        DrvMenu_EnterPage(MENU_PAGE_MAIN);
    }
}

static void DrvMenu_HandleKeys(void)
{
    Key_Status_t key = GetKeyStatus();
    u8 up_press = (key.key_up_pressed && !menu_last_key.key_up_pressed) ? 1U : 0U;
    u8 down_press = (key.key_down_pressed && !menu_last_key.key_down_pressed) ? 1U : 0U;
    u8 enter_press = (key.key_enter_pressed && !menu_last_key.key_enter_pressed) ? 1U : 0U;
    u8 back_press = (key.key_back_pressed && !menu_last_key.key_back_pressed) ? 1U : 0U;

    menu_last_key = key;

    if(menu_page == MENU_PAGE_MAIN)
    {
        DrvMenu_HandleMainKeys(up_press, down_press, enter_press);
    }
    else if(menu_page == MENU_PAGE_PID)
    {
        DrvMenu_HandlePidKeys(up_press, down_press, enter_press, back_press);
    }
    else
    {
        DrvMenu_HandleRadarKeys(back_press);
    }
}

static void DrvMenu_ShowFixed2(int16_t x, int16_t y, float value)
{
    s32 scaled;
    u32 abs_scaled;
    u32 int_part;
    u32 frac_part;

    scaled = (s32)((value >= 0.0f) ? (value * 100.0f + 0.5f) : (value * 100.0f - 0.5f));
    abs_scaled = (scaled < 0) ? (u32)(-scaled) : (u32)scaled;
    int_part = abs_scaled / 100U;
    frac_part = abs_scaled % 100U;

    if(int_part > 999U)
    {
        int_part = 999U;
        frac_part = 99U;
    }

    OLED_ShowChar(x, y, (scaled < 0) ? '-' : '+', OLED_6X8);
    OLED_ShowNum(x + 6, y, int_part, 3, OLED_6X8);
    OLED_ShowChar(x + 24, y, '.', OLED_6X8);
    OLED_ShowNum(x + 30, y, frac_part, 2, OLED_6X8);
}

static void DrvMenu_ShowSignedValue(int16_t x, int16_t y, s32 value, u8 length)
{
    OLED_ShowSignedNum(x, y, value, length, OLED_6X8);
}

static void DrvMenu_GetRadarSnapshot(DrvMenuRadarSnapshot_t *snapshot)
{
    if(snapshot == 0)
    {
        return;
    }

    snapshot->pos = Pos16_of_Radar.pos_data;
    snapshot->display_alt_source = alt_soruce;
    if(snapshot->display_alt_source == 1U)
    {
        snapshot->display_z_x100 = (s32)optical_flow.alt_cm;
    }
    else
    {
        snapshot->display_z_x100 = (s32)snapshot->pos.z_x100;
    }

    snapshot->yaw_x100 = Radar_YAW_tar_un.st_data.yaw_x100;
    snapshot->pos_update_cnt = radar_pos_update_cnt;
    snapshot->qua_update_cnt = radar_qua_update_cnt;
    snapshot->yaw_update_cnt = radar_yaw_update_cnt;
}

static u8 DrvMenu_GetFirstVisiblePidItem(void)
{
    if(menu_pid_item < 2U)
    {
        return 0;
    }

    if(menu_pid_item > (DRV_MENU_PID_ITEM_COUNT - 3U))
    {
        return DRV_MENU_PID_ITEM_COUNT - DRV_MENU_VISIBLE_ITEMS;
    }

    return menu_pid_item - 2U;
}

static void DrvMenu_DrawPidItem(u8 index, u8 row)
{
    float *value;
    int16_t y = (int16_t)(16 + row * 8);
    u8 param;

    OLED_ShowChar(0, y, (index == menu_pid_item) ? '>' : ' ', OLED_6X8);

    if(index == MENU_PID_ITEM_AXIS)
    {
        OLED_ShowString(6, y, "AXIS", OLED_6X8);
        OLED_ShowString(84, y, menu_axis_name[menu_axis], OLED_6X8);
    }
    else
    {
        param = MENU_PID_ITEM_TO_PARAM(index);
        OLED_ShowString(6, y, menu_param_name[param], OLED_6X8);

        value = DrvMenu_GetParamPtr(menu_axis, param);
        if(value != 0)
        {
            DrvMenu_ShowFixed2(78, y, *value);
        }
    }

    if(index == menu_pid_item)
    {
        OLED_ReverseArea(0, y, 128, 8);
    }
}

static void DrvMenu_DrawMain(void)
{
    u8 i;
    int16_t y;

    OLED_Clear();
    OLED_ShowString(0, 0, "MENU", OLED_8X16);

    for(i = 0; i < DRV_MENU_MAIN_ITEM_COUNT; i++)
    {
        y = (int16_t)(24 + i * 12);
        OLED_ShowChar(0, y, (i == menu_main_item) ? '>' : ' ', OLED_6X8);
        OLED_ShowString(12, y, menu_main_item_name[i], OLED_6X8);

        if(i == menu_main_item)
        {
            OLED_ReverseArea(0, y, 128, 8);
        }
    }

    OLED_ShowString(0, 56, "UP/DN SEL ENT IN", OLED_6X8);
    OLED_Update();
}

static void DrvMenu_DrawPid(void)
{
    u8 first;
    u8 i;

    OLED_Clear();

    OLED_ShowString(0, 0, "PID MENU", OLED_6X8);
    OLED_ShowString(84, 0, menu_editing ? "EDIT" : "SEL ", OLED_6X8);

    OLED_ShowString(0, 8, "AX:", OLED_6X8);
    OLED_ShowString(18, 8, menu_axis_name[menu_axis], OLED_6X8);
    OLED_ShowString(54, 8, "BACK=RET", OLED_6X8);

    first = DrvMenu_GetFirstVisiblePidItem();
    for(i = 0; i < DRV_MENU_VISIBLE_ITEMS; i++)
    {
        DrvMenu_DrawPidItem((u8)(first + i), i);
    }

    OLED_ShowString(0, 56, menu_editing ? "UP/DN +/-  ENT OK" : "UP/DN SEL ENT EDIT", OLED_6X8);
    OLED_Update();
}

static void DrvMenu_DrawRadar(void)
{
    DrvMenuRadarSnapshot_t snapshot;

    DrvMenu_GetRadarSnapshot(&snapshot);

    OLED_Clear();
    OLED_ShowString(0, 0, "RADAR POS", OLED_6X8);
    OLED_ShowString(90, 0, "BACK", OLED_6X8);

    OLED_ShowString(0, 10, "X:", OLED_6X8);
    DrvMenu_ShowSignedValue(18, 10, snapshot.pos.x_x100, 5);

    OLED_ShowString(0, 20, "Y:", OLED_6X8);
    DrvMenu_ShowSignedValue(18, 20, snapshot.pos.y_x100, 5);

    OLED_ShowString(0, 30, "Z:", OLED_6X8);
    DrvMenu_ShowSignedValue(18, 30, snapshot.display_z_x100, 5);
    OLED_ShowString(54, 30, (snapshot.display_alt_source == 1U) ? "OF" : "RD", OLED_6X8);

    OLED_ShowString(0, 40, "YAW:", OLED_6X8);
    DrvMenu_ShowFixed2(30, 40, (float)snapshot.yaw_x100 * 0.01f);

    OLED_ShowString(0, 52, "P", OLED_6X8);
    OLED_ShowNum(8, 52, snapshot.pos_update_cnt, 3, OLED_6X8);
    OLED_ShowString(32, 52, "Q", OLED_6X8);
    OLED_ShowNum(40, 52, snapshot.qua_update_cnt, 3, OLED_6X8);
    OLED_ShowString(64, 52, "Y", OLED_6X8);
    OLED_ShowNum(72, 52, snapshot.yaw_update_cnt, 3, OLED_6X8);

    OLED_Update();
}

static void DrvMenu_Draw(void)
{
    if(menu_page == MENU_PAGE_MAIN)
    {
        DrvMenu_DrawMain();
    }
    else if(menu_page == MENU_PAGE_PID)
    {
        DrvMenu_DrawPid();
    }
    else
    {
        DrvMenu_DrawRadar();
    }
}

void DrvMenu_RequestRedraw(void)
{
    menu_redraw = 1;
}

void DrvMenu_Init(void)
{
    menu_page = MENU_PAGE_MAIN;
    menu_main_item = 0;
    menu_axis = PID_X;
    menu_pid_item = MENU_PID_ITEM_AXIS;
    menu_editing = 0;
    menu_redraw = 1;
    menu_last_draw_ms = 0;
    menu_last_key = GetKeyStatus();
}

void DrvMenu_Task(void)
{
    u32 now_ms = HAL_GetTick();

    DrvMenu_HandleKeys();

    if(menu_redraw || (now_ms - menu_last_draw_ms) >= DRV_MENU_REFRESH_MS)
    {
        menu_last_draw_ms = now_ms;
        menu_redraw = 0;
        DrvMenu_Draw();
    }
}
