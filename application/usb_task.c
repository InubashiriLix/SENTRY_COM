/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usbï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ï?1?7
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
// #include "remote_control.h" // FIXME: Duplicate include
#include "crc8_crc16.h"
#include "detect_task.h"
#include "voltage_task.h"
#include "vision.h"

extern vision_control_t vision_control;

static void usb_printf(const char *fmt, ...);
static void get_INS_quat_data(uint8_t array);
// uint8_t usb_txbuf[49];
uint8_t usb_txbuf[512];
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
// ProjectileTx_t projectile_tx;
ProjectileTx_slow_t projectile_tx_slow;
ProjectileTx_fast_t projectile_tx_fast;
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

uint8_t tx_sending_fast_result = 4;
uint8_t tx_sending_slow_result = 4;
// TODO: remember to delete the result after debugging

unsigned long size_tx_slow = sizeof(ProjectileTx_slow_t);
unsigned long size_tx_fast = sizeof(ProjectileTx_fast_t);
static void send_projectile_tx_slow()
{
    memcpy(usb_txbuf, &projectile_tx_slow, size_tx_slow);
    projectile_tx_slow.checksum = Crc8Append(usb_txbuf, size_tx_slow);
    // CDC_Transmit_FS(usb_txbuf, sizeof(ProjectileTx_t));
    tx_sending_slow_result = CDC_Transmit_FS(usb_txbuf, size_tx_slow);
}

static void send_projectile_tx_fast()
{
    memcpy(usb_txbuf, &projectile_tx_fast, size_tx_fast);
    projectile_tx_fast.checksum = Crc8Append(usb_txbuf, size_tx_fast);
    // CDC_Transmit_FS(usb_txbuf, sizeof(ProjectileTx_fast_t));
    tx_sending_fast_result = CDC_Transmit_FS(usb_txbuf, size_tx_fast);
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

    projectile_tx_slow.shoot_remote = rc_ctrl.mouse.press_r;
    projectile_tx_slow.armor_color = 0;
    // FIXME: what are the two things above?
    

    // // FIXME: since the current side is unkwown, who is the enermy is also unknown
    // // NOTE: the following code is a temporary solution on the assumption that the id of sentry is in the robot_id_t enum
    
    projectile_tx_slow.current_side_color = get_robot_id() <= 7 ? 0 : 1;
    uint16_t* enemy_hp_ptr = (uint16_t*) get_game_robot_HP_point();
    for (int i = 0; i < 7; i ++) {
        int j = projectile_tx_slow.current_side_color == 0? i + 7 : i;
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

void usb_printf(const char *fmt, ...)
{
    static uint8_t usb_buf[256];
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);
    len = vsnprintf((char *)usb_buf, sizeof(usb_buf), fmt, ap);
    if (len > sizeof(usb_buf))
    {
        len = sizeof(usb_buf); // ·ÀÖ¹Òç³ö
    }
    va_end(ap);

    CDC_Transmit_FS(usb_buf, len);
}

// TODO: test for the received data
uint8_t received_data[64];

const char CRC8_INIT_ = 0xff;
const char CRC8_TABLE[] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};

const uint16_t wCRC16_INIT = 0xffff;
const uint16_t wCRC16_Table[256] =
    {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

// validation functions first

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

// ================================  for CRC CHECK  ================================
// ========= CRC16 ===========
/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t calculate_crc16(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == 0)
    {
        return 0xFFFF;
    }
    while (dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC16_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint8_t Crc16Verify(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = calculate_crc16(pchMessage, dwLength - 2, wCRC16_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

// ========= CRC8 ===========
uint8_t calculate_crc8(const uint8_t *data, uint16_t length, uint8_t init_value)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < length; i++)
    {
        crc = CRC8_TABLE[crc ^ data[i]];
    }
    return crc;
}

// the data should be an array of uint8_t
// the length should be the number of bytes in the data array (sizeof(data)?)
uint8_t Crc8Vertify(uint8_t *data, uint32_t length)
{
    if (data == NULL || length <= 1)
    {
        return 0;
    }
    uint8_t crc8_calculated = calculate_crc8(data, length - 1, CRC8_INIT_);
    return ((crc8_calculated & 0xff) == data[length - 1]) ? 1 : 0;
}

uint8_t Crc8Append(unsigned char* pchMessage, unsigned int dwLength)
{
    uint8_t ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 0)) {
        return 0x00;
    }
    ucCRC = calculate_crc8(pchMessage, dwLength - 1, CRC8_INIT_);
    pchMessage[dwLength - 1] = ucCRC;
    return ucCRC;
}