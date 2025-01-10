#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "user_task.h"
#include "bsp_fric.h"

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_supply_projectile_booking_t supply_projectile_booking_t;
ext_referee_warning_t referee_warning_t;

ext_game_robot_state_t robot_state;
ext_power_heat_data_t power_heat_data_t;
ext_game_robot_pos_t game_robot_pos_t;
ext_buff_musk_t buff_musk_t;
aerial_robot_energy_t robot_energy_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_bullet_remaining_t bullet_remaining_t;
ext_student_interactive_data_t student_interactive_data_t;

extern fp32 fric_30, fric_18, fric_15;
extern int8_t low_speed_flag;

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));

    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));

    memset(&robot_state, 0, sizeof(ext_game_robot_state_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(ext_buff_musk_t));
    memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));

    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));
}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
    case GAME_STATE_CMD_ID:
    {
        memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
    }
    break;
    case GAME_RESULT_CMD_ID:
    {
        memcpy(&game_result, frame + index, sizeof(game_result));
    }
    break;
    case GAME_ROBOT_HP_CMD_ID:
    {
        memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
    }
    break;
    case FIELD_EVENTS_CMD_ID:
    {
        memcpy(&field_event, frame + index, sizeof(field_event));
    }
    break;
    case SUPPLY_PROJECTILE_ACTION_CMD_ID:
    {
        memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
    }
    break;
    case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
    {
        memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
    }
    break;
    case REFEREE_WARNING_CMD_ID:
    {
        memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
    }
    break;

    case ROBOT_STATE_CMD_ID:
    {
        memcpy(&robot_state, frame + index, sizeof(robot_state));
    }
    break;
    case POWER_HEAT_DATA_CMD_ID:
    {
        memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
    }
    break;
    case ROBOT_POS_CMD_ID:
    {
        memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
    }
    break;
    case BUFF_MUSK_CMD_ID:
    {
        memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
    }
    break;
    case AERIAL_ROBOT_ENERGY_CMD_ID:
    {
        memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
    }
    break;
    case ROBOT_HURT_CMD_ID:
    {
        memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
        if (robot_hurt_t.hurt_type == 0)
        {
            if (robot_hurt_t.armor_type == 0)
            {
                ui_reset_car_front_armor_attacked_timer();
            }
            if (robot_hurt_t.armor_type == 1)
            {
                ui_reset_car_left_armor_attacked_timer();
            }
            if (robot_hurt_t.armor_type == 2)
            { // 这是后
                ui_reset_car_back_armor_attacked_timer();
            }
            if (robot_hurt_t.armor_type == 3)
            { // 这是左
                ui_reset_car_right_armor_attacked_timer();
            }
        }
        if (robot_hurt_t.hurt_type == 0x2)
        {
            fric_30 -= 200;
        }
    }
    break;
    case SHOOT_DATA_CMD_ID:
    {
        memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
        bullet_speed_change();
    }
    break;
    case BULLET_REMAINING_CMD_ID:
    {
        memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
    }
    break;
    case STUDENT_INTERACTIVE_DATA_CMD_ID:
    {
        memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
    }
    break;
    default:
    {
        break;
    }
    }
}

ext_event_data_t *get_field_event_point(void)
{
    return &field_event;
}

ext_referee_warning_t *get_referee_warning_point(void)
{
    return &referee_warning_t;
}

ext_power_heat_data_t *get_power_heat_data_point(void)
{
    return &power_heat_data_t;
}

ext_game_robot_pos_t *get_game_robot_pos_point(void)
{
    return &game_robot_pos_t;
}
void update_pos_position(float* pos){
    pos[0] = game_robot_pos_t.x;
    pos[1] = game_robot_pos_t.y;
    pos[2] = game_robot_pos_t.z;
    pos[3] = game_robot_pos_t.yaw;
}

ext_shoot_data_t *get_shoot_data_point(void)
{
    return &shoot_data_t;
}

ext_bullet_remaining_t *get_bullet_remaining_point(void)
{
    return &bullet_remaining_t;
}

ext_game_robot_HP_t *get_game_robot_HP_point(void)
{
    return &game_robot_HP_t;
}
ext_game_state_t *get_game_state_point(void)
{
    return &game_state;
}

ext_game_result_t *get_game_result_point(void)
{
    return &game_result;
}

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.chassis_power_buffer;
}

void get_chassis_max_power(uint16_t *max_power)
{
    *max_power = robot_state.chassis_power_limit;
}

uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}

void get_shoot_heat1_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    *heat0_limit = robot_state.shooter_id1_17mm_cooling_limit;
    *heat0 = power_heat_data_t.shooter_id1_17mm_cooling_heat;
}

void get_shoot_heat2_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    *heat1_limit = robot_state.shooter_id2_17mm_cooling_limit;
    *heat1 = power_heat_data_t.shooter_id2_17mm_cooling_heat;
}

ext_robot_hurt_t *get_hurt_point(void)
{
    return &robot_hurt_t;
}

ext_game_robot_state_t *get_robot_status_point(void)
{
    return &robot_state;
}

fp32 get_bullet_speed(void)
{
    return shoot_data_t.bullet_speed;
}

void bullet_speed_change(void)
{
    if (robot_state.shooter_id1_17mm_speed_limit == 15)
    {
        if (shoot_data_t.bullet_speed > (TAR_FRIC_15 + 0.5) || shoot_data_t.bullet_speed > (15 - 0.2))
        {
            fric_15 -= 100;
            low_speed_flag = 0;
        }
        else if (shoot_data_t.bullet_speed < (TAR_FRIC_15 - 0.5))
        {
            low_speed_flag++;
            if (low_speed_flag >= 3)
            {
                fric_15 += 50;
                low_speed_flag = 0;
            }
        }
        else
            low_speed_flag = 0;
    }
    if (robot_state.shooter_id1_17mm_speed_limit == 18)
    {
        if (shoot_data_t.bullet_speed > (TAR_FRIC_18 + 0.5) || shoot_data_t.bullet_speed > (18 - 0.2))
        {
            fric_18 -= 100;
            low_speed_flag = 0;
        }
        else if (shoot_data_t.bullet_speed < (TAR_FRIC_18 - 0.5))
        {
            low_speed_flag++;
            if (low_speed_flag >= 3)
            {
                fric_18 += 50;
                low_speed_flag = 0;
            }
        }
        else
            low_speed_flag = 0;
    }
    if (robot_state.shooter_id1_17mm_speed_limit == 30)
    {
        if (shoot_data_t.bullet_speed > (TAR_FRIC_30 + 0.5) || shoot_data_t.bullet_speed > (30 - 0.2))
        {
            fric_30 -= 100;
            low_speed_flag = 0;
        }
        else if (shoot_data_t.bullet_speed < (TAR_FRIC_30 - 0.5))
        {
            low_speed_flag++;
            if (low_speed_flag >= 3)
            {
                fric_30 += 50;
                low_speed_flag = 0;
            }
        }
        else
            low_speed_flag = 0;
    }
}

uint8_t get_shooter_speed_limit()
{
    return robot_state.shooter_id1_17mm_speed_limit;
}

uint8_t get_left_amour_attacked()
{
    if (robot_hurt_t.hurt_type == 0)
    {
        if (robot_hurt_t.armor_type == 1)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

uint8_t get_front_amour_attacked()
{
    if (robot_hurt_t.hurt_type == 0)
    {
        if (robot_hurt_t.armor_type == 0)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

uint8_t get_back_amour_attacked()
{
    if (robot_hurt_t.hurt_type == 0)
    {
        if (robot_hurt_t.armor_type == 2)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

uint8_t get_right_amour_attacked()
{
    if (robot_hurt_t.hurt_type == 0)
    {
        if (robot_hurt_t.armor_type == 3)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}
