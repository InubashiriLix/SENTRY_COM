/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H

#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"
#include "referee.h"


//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1
//云台模式使用的开关通道

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_CAN_ADD_VALUE    5000.0f

//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_X
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_Z

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15
//鼠标长按判断
#define PRESS_LONG_TIME             250
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18
//拨弹速度
#define TRIGGER_SPEED               4.0f
#define CONTINUE_TRIGGER_SPEED      4.0f
#define READY_TRIGGER_SPEED         5.0f

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  700
#define REVERSE_TIME                500
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f
#define PI_EIGHT                    0.39269908169872415480783042290994f

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP        800.0f
#define TRIGGER_ANGLE_PID_KI        0.5f
#define TRIGGER_ANGLE_PID_KD        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f

//摩擦轮PID
#define FRIC_ANGLE_PID_KP 10.0f
#define FRIC_ANGLE_PID_KI 0.15f
#define FRIC_ANGLE_PID_KD 0.0f

#define FRIC_PID_MAX_OUT 15000.0f
#define FRIC_PID_MAX_IOUT 5020.0f


#define SHOOT_HEAT_REMAIN_VALUE     80

typedef enum {
    SHOOT_STOP = 0,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,
} shoot_mode_e;

typedef enum {
    RIGHT = 0,
    LEFT,
} dual_mode_e;
typedef enum {
    HIGH_SHOOT,
    LOW_SHOOT,
} trigger_mode_e;
typedef struct {
    shoot_mode_e shoot_mode;
    trigger_mode_e trigger_mode;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
    const motor_measure_t *fric_motor_measure[2];
    ramp_function_source_t fric1_ramp;
    uint16_t fric_can1;
    ramp_function_source_t fric2_ramp;
    uint16_t fric_can2;
    pid_type_def trigger_motor_pid;
    pid_type_def fric_motor_pid[2];
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current[2];
    int8_t ecd_count;

    bool_t press_l;
    bool_t last_press_l;
    uint16_t press_l_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;

    bool_t key;
    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat_limit2;
    uint16_t heat;
    uint16_t heat2;
    dual_mode_e dual_mode;
    ext_game_robot_state_t *shoot_state;
} shoot_control_t;

//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern void shoot_init(void);

extern int16_t *shoot_control_loop(void);

shoot_control_t *get_shoot_point(void);

uint8_t get_fric_state();

uint8_t get_trigger_state();

uint8_t get_shooter_mode();

#endif