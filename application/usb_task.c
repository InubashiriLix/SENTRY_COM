/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb����������?1?7
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"
#include "remote_control.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "Can_receive.h"
#include "crc8_crc16.h"
#include "detect_task.h"
#include "voltage_task.h"
#include "vision.h"

extern vision_control_t vision_control;

static void usb_printf(const char *fmt, ...);
static void get_INS_quat_data(uint8_t array);
uint8_t usb_txbuf[256];
vision_t vision_data;
extern USBD_HandleTypeDef hUsbDeviceFS;
int32_t usb_count=0;
static void get_buller_speed(uint8_t array);
static void get_HP_data(uint8_t array);
void vision_init();
static void vision_data_update();
static void get_vision_data(void);
static void get_yaw_data(uint8_t array);
static void get_pitch_data(uint8_t array);

vision_rx_t vision_rx;
MotionRx_t motion_rx;
ProjectileTx_slow_t projectile_tx_slow;
ProjectileTx_fast_t projectile_tx_fast;

// border line of two sides
const uint8_t borderline_side = 100;

extern RC_ctrl_t rc_ctrl;
void usb_task_(void const *argument)
{
    
    MX_USB_DEVICE_Init();
		vision_init();
    osDelay(1000);

    while (1)
    {
			vision_data_update();
			get_vision_data();
        vTaskDelay(10);
    }
}

void usb_task(void const *argument)
{
    MX_USB_DEVICE_Init();
    projectile_tx_struct_init();
    osDelay(1000);

    while(1){
        for (int i = 0; i < 9; i++) {
            projectile_tx_fast_update();
            send_projectile_tx_fast();
            vTaskDelay(10);
        }
        projectile_tx_slow_update();
        send_projectile_tx_slow();
        vTaskDelay(10);
    }
}

uint32_t system_time;
uint16_t a = 20;
static void get_vision_data(void)
{
    usb_txbuf[0] = vision_data.SOF;
		get_INS_quat_data(1);
	  usb_txbuf[17] = vision_control.vision_mode;
		usb_txbuf[18] = 0xFF;
		if(vision_data.robot_state_v->robot_id>100)
		{
			usb_txbuf[19] = 0;
		}
		else
			usb_txbuf[19] = 1;
		get_buller_speed(20);
		get_HP_data(24);
		uint32_t system_time = xTaskGetTickCount();
    usb_txbuf[39] = system_time >> 24;
    usb_txbuf[38] = system_time >> 16;
    usb_txbuf[37] = system_time >> 8;
    usb_txbuf[36] = system_time;
		get_yaw_data(40);
		get_pitch_data(44);
		append_CRC8_check_sum(usb_txbuf,sizeof(usb_txbuf));
    CDC_Transmit_FS(usb_txbuf, sizeof(usb_txbuf));
}

unsigned long size_tx_slow = sizeof(ProjectileTx_slow_t);
unsigned long size_tx_fast = sizeof(ProjectileTx_fast_t);
static void send_projectile_tx_slow()
{
    memcpy(usb_txbuf, &projectile_tx_slow, size_tx_slow);
    append_CRC8_check_sum(usb_txbuf, size_tx_slow);
    CDC_Transmit_FS(usb_txbuf, size_tx_slow);
}

static void send_projectile_tx_fast()
{
    memcpy(usb_txbuf, &projectile_tx_fast, size_tx_fast);
    append_CRC8_check_sum(usb_txbuf, size_tx_fast);
    CDC_Transmit_FS(usb_txbuf, size_tx_fast);
}

void vision_init()
{
	vision_data.INS_quat_vision = get_INS_quat_point();
	vision_data.robot_state_v = get_robot_status_point();
	vision_data.vision_gimbal = get_gimbal_point();
	vision_data.SOF = 0XA5;
	vision_data.vision_mode = CLASSIC;
}

void projectile_tx_struct_init()
{
    projectile_tx_fast_update();
    projectile_tx_slow_update();
}

static void get_INS_quat_data(uint8_t array)
{
    union refree_4_byte_t INS_data[4];
    INS_data[0].f = *vision_data.INS_quat_vision;
    INS_data[1].f = *(vision_data.INS_quat_vision + 1);
    INS_data[2].f = *(vision_data.INS_quat_vision + 2);
    INS_data[3].f = *(vision_data.INS_quat_vision + 3);

    usb_txbuf[array + 1] = INS_data[0].buf[1];
    usb_txbuf[array + 2] = INS_data[0].buf[2];
    usb_txbuf[array + 3] = INS_data[0].buf[3];

    usb_txbuf[array + 4] = INS_data[1].buf[0];
    usb_txbuf[array + 5] = INS_data[1].buf[1];
    usb_txbuf[array + 6] = INS_data[1].buf[2];
    usb_txbuf[array + 7] = INS_data[1].buf[3];

    usb_txbuf[array + 8] = INS_data[2].buf[0];
    usb_txbuf[array + 9] = INS_data[2].buf[1];
    usb_txbuf[array + 10] = INS_data[2].buf[2];
    usb_txbuf[array + 11] = INS_data[2].buf[3];

		usb_txbuf[array + 12] = INS_data[3].buf[0];
    usb_txbuf[array + 13] = INS_data[3].buf[1];
    usb_txbuf[array + 14] = INS_data[3].buf[2];
    usb_txbuf[array + 15] = INS_data[3].buf[3];
}

static void get_pitch_data(uint8_t array)
{
    union refree_4_byte_t pitch_data;
    pitch_data.f = vision_data.vision_gimbal->gimbal_pitch_motor.absolute_angle;
	
    usb_txbuf[array] = pitch_data.buf[0];
    usb_txbuf[array + 1] = pitch_data.buf[1];
    usb_txbuf[array + 2] = pitch_data.buf[2];
    usb_txbuf[array + 3] = pitch_data.buf[3];
}

static void get_yaw_data(uint8_t array)
{
    union refree_4_byte_t yaw_data;
    yaw_data.f = vision_data.vision_gimbal->gimbal_yaw_motor.absolute_angle;
	
    usb_txbuf[array] = yaw_data.buf[0];
    usb_txbuf[array + 1] = yaw_data.buf[1];
    usb_txbuf[array + 2] = yaw_data.buf[2];
    usb_txbuf[array + 3] = yaw_data.buf[3];
}
static void vision_data_update()
{
	vision_data.bullet_speed = get_bullet_speed();
}

static void projectile_tx_slow_update(){
    projectile_tx_slow.header = 0xA5;

    projectile_tx_slow.timestamp = xTaskGetTickCount();

    projectile_tx_slow.vision_mode = CLASSIC;

    projectile_tx_slow.shoot_remote = rc_ctrl.mouse.press_r; // 我瞎写的, 记得弄清后改
    // FIXME: what are the one things above?
    

    // due to the id of robot is
    //  1：红方英雄机器人
    //  2：红方工程机器人
    //  3/4/5：红方步兵机器人（与机器人ID 3~5对应）
    //  6：红方空中机器人
    //  7：红方哨兵机器人
    //  8：红方飞镖
    //  9：红方雷达
    //  10：红方前哨站
    //  11：红方基地
    //  101：蓝方英雄机器人
    //  102：蓝方工程机器人
    //  103/104/105：蓝方步兵机器人（与机器人ID 3~5对应）
    //  106：蓝方空中机器人
    //  107：蓝方哨兵机器人
    //  108：蓝方飞镖
    //  109：蓝方雷达
    //  110：蓝方前哨站
    //  111：蓝方基地
    // 屎山三元计算
    projectile_tx_slow.current_side_color = get_robot_id() == 0 ? 0 : 
                                            get_robot_id() < 100 ? 1 : 2;
    // 0 -> unknown  1 -> red  2 -> blue
    uint16_t* enemy_hp_ptr = (uint16_t*) get_game_robot_HP_point();
    for (int i = 0; i < 7; i ++) {
        int j;
        if (projectile_tx_slow.current_side_color == 1) j = i + 7;
        else if (projectile_tx_slow.current_side_color == 2) j = i;
        
        projectile_tx_slow.enemy_hp[i] = *(enemy_hp_ptr + j * sizeof(uint16_t));
    }
    
    projectile_tx_slow.bullet_remaining_info = get_bullet_remaining_point()->bullet_remaining_num;
    // // [0] for the 17mm bullet

    uint8_t *fieldevents_ptr = (uint8_t*) get_field_event_point();
    for (int i = 0; i < 10; i++) {
        projectile_tx_slow.field_events[i] = *(fieldevents_ptr + i * sizeof(uint8_t));
    }

    projectile_tx_slow.game_type = get_game_state_point()->game_type;
    projectile_tx_slow.game_progress = get_game_state_point()->game_progress;
    projectile_tx_slow.state_remain_time = get_game_state_point()->stage_remain_time;
    // FIXME: the sync_time_stamp is not defined in the referee, but has been exists in the upper machine
    // projectile_tx_slow.sync_time_stamp = get_game_state_point()->sync_time_stamp;

    projectile_tx_slow.winner = get_game_result_point()->winner;

    // uint8_t *robot_buffs_ptr = (uint8_t*) get_game_robot_state_point();
    // FIXME: the robot buffers (robot_replenish_blood, shooter_cooling_acceleration, 
    // robot_defense_bonus, robot_attack_bonus) are not defined in the referee, 
    // but has been exists in the upper machine    
    // temporary solution
    memset(projectile_tx_slow.robot_buffs, 0, sizeof(projectile_tx_slow.robot_buffs));

    update_pos_position(projectile_tx_slow.robot_position);

    projectile_tx_slow.robot_status_0[0] = get_robot_status_point()->robot_id;
    projectile_tx_slow.robot_status_0[1] = get_robot_status_point()->robot_level;
    projectile_tx_slow.robot_status_1[0] = get_robot_status_point()->remain_HP;
    projectile_tx_slow.robot_status_1[1] = get_robot_status_point()->max_HP;
    projectile_tx_slow.robot_status_1[2] = get_robot_status_point()->shooter_id1_17mm_cooling_rate;
    projectile_tx_slow.robot_status_1[3] = get_robot_status_point()->shooter_id1_17mm_cooling_limit;
    projectile_tx_slow.robot_status_1[4] = get_robot_status_point()->shooter_id1_17mm_speed_limit;
    projectile_tx_slow.robot_status_1[5] = get_robot_status_point()->shooter_id2_17mm_cooling_rate;
    projectile_tx_slow.robot_status_1[6] = get_robot_status_point()->shooter_id2_17mm_cooling_limit;
    projectile_tx_slow.robot_status_1[7] = get_robot_status_point()->shooter_id2_17mm_speed_limit;
    projectile_tx_slow.robot_status_1[8] = get_robot_status_point()->shooter_id1_42mm_cooling_rate;
    projectile_tx_slow.robot_status_1[9] = get_robot_status_point()->shooter_id1_42mm_cooling_limit;
    projectile_tx_slow.robot_status_1[10] = get_robot_status_point()->shooter_id1_42mm_speed_limit;
    projectile_tx_slow.robot_status_1[11] = get_robot_status_point()->chassis_power_limit;
    projectile_tx_slow.robot_status_1[12] = get_robot_status_point()->mains_power_gimbal_output;
    projectile_tx_slow.robot_status_1[13] = get_robot_status_point()->mains_power_chassis_output;
    projectile_tx_slow.robot_status_1[14] = get_robot_status_point()->mains_power_shooter_output;

    // // FIXME: the client command is not defined in the referee, but has been exists in the upper machine
    // // NOTE: temporarily set the command_target_position to (0, 0, 0)
    projectile_tx_slow.command_target_position[0] = 0;
    projectile_tx_slow.command_target_position[1] = 0;
    projectile_tx_slow.command_target_position[2] = 0;

    // // FIXME: IS there any need for the keyboard_key_pressed? We are dealing with SENTRY
    projectile_tx_slow.keyboard_key_pressed = get_remote_control_point()->key.v;

    // // FIXME: where is the command_target_robot_id?
    projectile_tx_slow.command_target_robot_id = 0;

    // // FIXME: where is the receive_target_position?
    projectile_tx_slow.receive_target_position[0] = 0;
    projectile_tx_slow.receive_target_position[1] = 0;

    // // FIXME: where is the receive_target_robot_id?
    projectile_tx_slow.receive_target_robot_id = 0;
}

static void projectile_tx_fast_update (void) 
{

    projectile_tx_fast.header = 0xF8;

    const float *q_ = get_INS_quat_point();
    for (int i = 0; i < 4; i++){
        projectile_tx_fast.q[i] = *(q_ + i * sizeof(float));
    }

    projectile_tx_fast.yaw = get_gimbal_point()->gimbal_yaw_motor.absolute_angle;
    projectile_tx_fast.pitch = get_gimbal_point()->gimbal_pitch_motor.absolute_angle;

    projectile_tx_fast.lever_mode = rc_ctrl.rc.s[0];

    projectile_tx_fast.bullet_speed = get_bullet_speed();
}

static void get_HP_data(uint8_t array)
{
	for(int i = 0;i<=11;i++)
	{
		usb_txbuf[array+1] = 0;
	}
}
static void get_buller_speed(uint8_t array)
{
	  union refree_4_byte_t INS_data;
    INS_data.f = vision_data.bullet_speed;
	
    usb_txbuf[array] = INS_data.buf[0];
    usb_txbuf[array + 1] = INS_data.buf[1];
    usb_txbuf[array + 2] = INS_data.buf[2];
    usb_txbuf[array + 3] = INS_data.buf[3];
}

static void usb_printf(const char *fmt, ...)
{
    static uint8_t usb_buf[256];
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);
    len = vsnprintf((char *)usb_buf, sizeof(usb_buf), fmt, ap);
    if (len > sizeof(usb_buf))
    {
        len = sizeof(usb_buf); // ��ֹ���
    }
    va_end(ap);

    CDC_Transmit_FS(usb_buf, len);
}

// ==============================  for PARITY CHECK  ==============================
// ====== ODD ======
// ???
uint8_t calculate_parity(const uint8_t *data, size_t length)
{
    uint8_t parity = 0;
    for (size_t i = 0; i < length; i++)
    {
        parity ^= data[i]; // ???????
    }
    return parity & 1; // ??????0 ??????1 ??????
}

uint8_t vertify_parity(const uint8_t *data, size_t length)
{
    // ?????????????????????
    uint8_t calculated_parity_code = calculate_parity(data, length - 2);

    // ???????????????
    uint8_t stored_parity_code = data[length - 1];

    // ???????
    if (calculated_parity_code == stored_parity_code)
    {
        return 1; // ????
    }
    else
    {
        return 0; // ????
    }
}
