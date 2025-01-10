/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      no action.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef USB_TASK_H
#define USB_TASK_H
#include "struct_typedef.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "referee.h"
#include "gimbal_task.h"
typedef enum
{
	CLASSIC =0,
	WIND = 1,
}vision_mode_e;
typedef __packed struct
{
	uint8_t SOF;
	uint8_t target_found;
	fp32 pitch_angle;
	fp32 yaw_angle;
	uint8_t checksum;
}vision_rx_t;

typedef  struct
{
	uint8_t SOF;
	const fp32 *INS_quat_vision;
	vision_mode_e vision_mode;
	gimbal_control_t *vision_gimbal;
	uint8_t shoot_remote;
	uint8_t enery_color;
	ext_game_robot_state_t *robot_state_v;
	fp32 bullet_speed;
}vision_t;

#define PROJECTILE_TX_SLOW_HEADER 0xA5
#define PROJECTILE_TX_FAST_HEADER 0xF8


typedef struct ProjectileTx_slow
{
    uint8_t header;

    // TODO: WHAT IS THIS?
    uint32_t timestamp;

    // NOTE: the mode can be set as any value you want
    uint8_t vision_mode;

    // the gurgement part (coming from c borad)
    uint8_t shoot_remote;
    uint8_t armor_color;
    // TODO: WTF is the two things above

    // the current side color (which team we are)
    // 0 -> red
    // 1 -> blue
    uint8_t current_side_color;
    // the enemt_hp
    uint16_t enemy_hp[6];

    // the bullet_remaining
    uint16_t bullet_remaining_info[3];
    // 0 -> uint16_t remaining_17mm_num;
    // 1 -> uint16_t remaining_42mm_num;
    // 2 -> uint16_t remaining_coin_num;

    // field events
    uint8_t field_events[11];
    // 0 -> uint8_t supplier_1_occupation;
    // 1 -> uint8_t supplier_2_occupation;
    // 2 -> uint8_t supplier_3_occupation;
    // 3 -> uint8_t power_rune_activation_point_occupation;
    // 4 -> uint8_t small_power_rune_activation_status;
    // 5 -> uint8_t big_power_rune_activation_status;
    // 6 -> uint8_t r2b2_ground_occupation;
    // 7 -> uint8_t r3b3_ground_occupation;
    // 8 -> uint8_t r4b4_ground_occupation;
    // 9 -> uint8_t base_has_shield;
    // 10 -> uint8_t outpost_alive;

    // game status
    uint8_t game_type;
    uint8_t game_progress;
    uint16_t state_remain_time;
    uint64_t sync_time_stamp;

    // game result
    uint8_t winner;

    // robot buffs
    uint8_t robot_buffs[4];
    // 0 -> uint8_t robot_replenishing_blood;
    // 1 -> uint8_t shooter_cooling_acceleration;
    // 2 -> uint8_t robot_defense_bonus;
    // 3 -> uint8_t robot_attack_bonus;

    // robot position
    float robot_position[4];
    // 0 -> x = 0.0f;
    // 1 -> y = 0.0f;
    // 2 -> z = 0.0f;
    // 3 -> yaw = 0.0f;

    // robot status
    uint8_t robot_status_0[2];
    // 0 -> uint8_t robot_id;
    // 1 -> uint8_t robot_level;
    uint16_t robot_status_1[15];
    // 0 -> uint16_t remain_hp;
    // 1 -> uint16_t max_hp;
    // 2 -> uint16_t shooter_17mm_id1_cooling_rate;
    // 3 -> uint16_t shooter_17mm_id1_cooling_limit;
    // 4 -> uint16_t shooter_17mm_id1_speed_limit;
    // 5 -> uint16_t shooter_17mm_id2_cooling_rate;
    // 6 -> uint16_t shooter_17mm_id2_cooling_limit;
    // 7 -> uint16_t shooter_17mm_id2_speed_limit;
    // 8 -> uint16_t shooter_42mm_id1_cooling_rate;
    // 9 -> uint16_t shooter_42mm_id1_cooling_limit;
    // 10 -> uint16_t shooter_42mm_id1_speed;
    // 11 -> uint16_t chassis_power_limit;
    // 12 -> uint16_t gimbal_power_output;
    // 13 -> uint16_t chassis_power_output;
    // 14 -> uint16_t shooter_power_output;

    // client command
    float command_target_position[3];
    // float target_position_x;
    // float target_position_y;
    // float target_position_z;
    uint16_t keyboard_key_pressed;
    uint8_t command_target_robot_id;

    // client receive
    float receive_target_position[2];
    // float target_position_x;
    // float target_position_y;
    uint8_t receive_target_robot_id;

    uint8_t checksum;
} ProjectileTx_slow_t;

typedef struct __attribute__((packed)){
  uint8_t header;
  float q[4];

  float yaw;
  float pitch;

  float bullet_speed;

  uint8_t checksum;
} ProjectileTx_fast_t;

#define MOTION_RX_HEADER 0x5A
typedef struct MotionRx
{
    // the type of the message
    uint8_t header;          // 第 0-3 位
    float yaw;               // 第 4-7 位
    float pitch;             // 第 4-11 位
    float actual1;           // 第 8-15 位
    uint8_t raw1[4];         // 第 12-19 位
    float actual2;           // 第 12-23 位  
    uint8_t raw2[4];         // 第 19～27位
    float linear_x;          // 第 24～31 位
    float linear_y;          // 第 28～35 位
    float angular_z;         // 第 32～39 位
    uint8_t target_found;    // 第 41 位
    uint8_t placeholder2;    // 第 42 位
    uint8_t placeholder3;    // 第 43 位
    uint8_t checksum;        // 第 44 位 (校验位)
} MotionRx_t;
// extern size_t MotionRxSize = sizeof(MotionRx_t);  

// crc8, crc16 and parity vertification
extern const char CRC8_INIT_;
extern const char CRC8_TABLE[256];
extern const uint16_t wCRC16_INIT;
extern const uint16_t wCRC16_Table[256];

void usb_task_(void const *argument);
static void projectile_tx_struct_init(void);
static void projectile_tx_slow_update(void);
static void projectile_tx_fast_update(void);
static void send_projectile_tx_slow(void);
static void send_projectile_tx_fast(void);

uint8_t calculate_parity(const uint8_t *data, size_t length);
uint8_t vertify_parity(const uint8_t *data, size_t length);
uint16_t calculate_crc16(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint8_t Crc16Verify(uint8_t *pchMessage, uint32_t dwLength);
uint8_t calculate_crc8(const uint8_t *data, uint16_t length, uint8_t init_value);
uint8_t Crc8Vertify(uint8_t *data, uint32_t length);
uint8_t Crc8Append(unsigned char* pchMessage, unsigned int dwLength);

union refree_4_byte_t
{
		float f;
		unsigned char buf[4];
};
extern void usb_task(void const * argument);

#endif
