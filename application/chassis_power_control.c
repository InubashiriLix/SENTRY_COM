/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             只控制80w功率，主要通过控制电机电流设定值,如果限制功率是40w，减少
  *             JUDGE_TOTAL_CURRENT_LIMIT和POWER_CURRENT_LIMIT的值，还有底盘最大速度
  *             (包括max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "remote_control.h"
#include "pid.h"
#include "voltage_task.h"

#define POWER_LIMIT 80.0f
#define WARNING_POWER 40.0f
#define WARNING_POWER_BUFF 50.0f

#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f //16000 * 4,

extern cap_measure_t cap_measure;
extern RC_ctrl_t rc_ctrl;
extern fp32 battery_voltage;
/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */
    uint16_t max_power_limit = 40;
		uint8_t cap_state=0;
		fp32 total_current_limit = 0.0f;
		fp32 chassis_max_power;
		float capacitor_power;
		float initial_give_power[4];
		float initial_total_power;
		fp32 scale_give_power[4];
void chassis_power_control(chassis_move_t *chassis_power_control)
{
    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    //fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;

		fp32 toque_coefficient = 1.99688994e-6f;
		fp32 a =  1.23e-07; //k2
		fp32 k1 = 1.453e-07; //k1
		fp32 c;
		get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
		PID_calc(&chassis_power_control->buffer_pid,chassis_power_buffer,30);
		get_chassis_max_power(&max_power_limit);
		capacitor_power= max_power_limit - chassis_power_control->buffer_pid.out;
        if(capacitor_power>=120) capacitor_power = 120;
		CAN_CMD_CAP(capacitor_power);
		
        //设置功率  
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)
		{
			cap_state = 0;
		}
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)
		{
			cap_state = 1;
		}
        
        if(!toe_is_error(REFEREE_TOE))
        {      
            if(!toe_is_error(CAP_TOE))
            {  
                if(cap_state == 0)
                {
                    chassis_max_power = capacitor_power;
                }else
                {
                    if(cap_measure.cap_percent>10)
                    {
                        chassis_max_power = capacitor_power + 300;
                    }else
                    {
                        chassis_max_power = capacitor_power;
                        cap_state = 0;
                    }
                }
            }else
              {
                  chassis_max_power = capacitor_power;
              }
        }else
         {
             chassis_max_power = 45;
         }
		

        //计算功率
            
		initial_total_power = 0;
		for (uint8_t i = 0; i < 4; i++) //first get all the initial motor power and total motor power
		{
			initial_give_power[i] =  chassis_power_control->motor_speed_pid[i].out * toque_coefficient * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm + \
			k1 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +\
			a * chassis_power_control->motor_speed_pid[i].out * chassis_power_control->motor_speed_pid[i].out + 4.081f;
			
			if(initial_give_power<0) continue;
			initial_total_power +=initial_give_power[i];
		}
	
		//判断及限制功率
		
		if(initial_total_power > chassis_max_power) //larger than max power
		{
			fp32 power_scale = chassis_max_power/initial_total_power;
			//fp32 power_scale = chassis_max_power/4;
			for (uint8_t i = 0; i < 4; i++)
			{
				scale_give_power[i] = initial_give_power[i] * power_scale;  //get scaled power
				if(scale_give_power[i]<0||initial_give_power[i]<scale_give_power[i])
				{
					continue;
				}
				//scale_give_power[i] = chassis_max_power/4;
				fp32 b = toque_coefficient * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm;
				c = k1 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm - scale_give_power[i] + 4.081f;
				
					if(chassis_power_control->motor_speed_pid[i].out > 0)
					{	
							fp32 temp = (-b + sqrt(b*b-4*a*c)) / (2*a);
							if(temp > 16000)
							{
								chassis_power_control->motor_speed_pid[i].out = 16000;
							}
							else
								chassis_power_control->motor_speed_pid[i].out =  temp;
					}
					else
					{
							fp32 temp = (-b - sqrt(b*b-4*a*c)) / (2*a);
							if(temp < -16000)
							{
								chassis_power_control->motor_speed_pid[i].out = - 16000;
							}
							else
								chassis_power_control->motor_speed_pid[i].out =   temp;
					}
			}
		}
}


