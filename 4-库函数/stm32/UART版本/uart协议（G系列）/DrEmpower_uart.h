#ifndef __B07_HI_H__
#define __B07_HI_H__

#include <stdint.h>
#include <stdio.h>

struct servo_state
{
    float angle;
    float speed;
};

struct servo_volcur
{
    float vol;
    float cur;
};

void format_data( float *value_data, int *type_data,int length, char * str);
void reply_state(uint8_t id_num);
void preset_angle(uint8_t id_num, float angle, float t, float param, int mode);
void preset_speed(uint8_t id_num, float speed, float param, int mode);
void preset_torque(uint8_t id_num, float torque, float param, int mode);
void set_angle(uint8_t id_num, float angle, float speed, float param, int mode);
void set_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n);
void step_angle(uint8_t id_num, float angle, float speed, float param, int mode);
void step_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n);
void set_speed(uint8_t id_num, float speed, float param, int mode);
void set_speeds(uint8_t *id_list, float *speed_list, float param, float mode, size_t n);
void set_torque(uint8_t id_num, float torque, float param, int mode);
void set_torques(uint8_t *id_list, float *torque_list, float param, int mode, size_t n);
void impedance_control(uint8_t id_num, float pos, float vel, float tff, float kp, float kd);
void estop(uint8_t id_num);
void set_id(uint8_t id_num,int new_id);
void set_uart_baud_rate(uint8_t id_num, int baud_rate);
void set_can_baud_rate(uint8_t id_num, int baud_rate);
void set_mode(uint8_t id_num, int mode);
void set_zero_position(uint8_t id_num);
void set_GPIO_mode(uint8_t id_num, uint8_t mode, uint32_t param);
int8_t set_angle_range(uint8_t id_num, float angle_min, float angle_max);
void set_traj_mode(uint8_t id_num,int mode);
void write_property(uint8_t id_num,unsigned short param_address,int8_t param_type,float value);
uint8_t get_id(uint8_t id_num);
struct servo_state get_state(uint8_t id_num);
struct servo_volcur get_volcur(uint8_t id_num);
int8_t get_GPIO_mode(uint8_t id_num, uint8_t *enable_uart, uint8_t *enable_step_dir, uint32_t *n);
float read_property(uint8_t id_num,int param_address, int param_type);
void clear_error(uint8_t id_num);
int8_t dump_error(uint8_t id_num);
void save_config(uint8_t id_num);
void reboot(uint8_t id_num);
void position_done(uint8_t id_num);
void positions_done(uint8_t *id_list,size_t n);
#endif /* __B07_HI_H__ */
