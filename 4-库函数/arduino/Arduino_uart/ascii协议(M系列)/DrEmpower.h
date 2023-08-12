#ifndef __DREMPOWER_H__
#define __DREMPOWER_H__

#include <stdint.h>
#include <stdio.h>
#include <Arduino.h>

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

extern int8_t READ_FLAG;
extern int8_t TRAJ_MODE; //速度轨迹模式选择控制，1表示梯形轨迹模式，2表示S形轨迹模式（指的是速度曲线形状，适用于位置控制-梯形轨迹模式）

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
void write_property(uint8_t id_num, char *property, float value);
uint8_t get_id(uint8_t id_num);
struct servo_state get_state(uint8_t id_num);
struct servo_volcur get_volcur(uint8_t id_num);
int8_t get_GPIO_mode(uint8_t id_num, uint8_t *enable_uart, uint8_t *enable_step_dir, uint32_t *n);
float read_property(uint8_t id_num, char *property);
void clear_error(uint8_t id_num);
int8_t dump_error(uint8_t id_num, char *error);
void save_config(uint8_t id_num);
void reboot(uint8_t id_num);
void position_done(uint8_t id_num);
void positions_done(uint8_t *id_list,size_t n);

#endif /* __DREMPOWER_H__ */
