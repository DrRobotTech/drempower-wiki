#include "DrEmpower.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stm32F1xx.h"
#include "usart.h"

#define INPUT_MODE_PASSTHROUGH 1
#define INPUT_MODE_VEL_RAMP 2
#define AXIS_STATE_IDLE 1
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8

#define SERVO_USART huart1              /* 驱动电机使用的串口 */
#define SERVO_RECEIVE_TIMEOUT 1000        /* 串口接收超时(ms) */
#define ENABLE_INPUT_VALIDITY_CHECK 1   /* 输入合法性检查。为 0 时不编译空指针判断等检查，可减小程序体积、加快处理速度。 */

#define SERVO_MALLOC(size) malloc(size) /* 如果使用了操作系统，需要将这个函数换成操作系统对应的函数 */
#define SERVO_FREE(ptr) free(ptr)       /* 如果使用了操作系统，需要将这个函数换成操作系统对应的函数 */
#define SERVO_DELAY(n) HAL_Delay(n)     /* 如果使用了操作系统，需要将这个函数换成操作系统对应的函数 */
#define SERVO_SPRINTF sprintf           /* 使用这个函数需要在 Keil 里勾选 Use MicroLIB 或在 STM32CubeIDE 里勾选 Use float with printf from newlib-nano */
#define PRE_READ() __HAL_UART_FLUSH_DRREGISTER(&SERVO_USART)

char command[64];       // 命令发送缓冲区
uint8_t rx_buffer[128]; // 接收缓冲区
int8_t READ_FLAG = 0;

int8_t TRAJ_MODE = 1;  //速度轨迹模式选择控制，1表示梯形轨迹模式，2表示S形轨迹模式（指的是速度曲线形状，适用于位置控制-梯形轨迹模式）

// 内部辅助函数，用户无需使用

/**
 * @brief 使用串口发送命令
 * 
 * @param strLength 要发送的命令的长度
 */
void send_command(uint16_t strLength)
{
	
    HAL_UART_Transmit(&SERVO_USART, (uint8_t *)&command, strLength, 1000);
}

/**
 * @brief 从串口读取数据
 * 当 mode 为 0 时，收到'\n'返回 1，当 mode 为 1 时，超时返回最后接收到的字符，当 mode 为 2 时，收到'\0'返回 1。所有情况如遇出错返回 0。
 * 
 * @param mode 模式
 * @return int8_t
 */
uint8_t read_data(uint8_t mode)
{
    rx_buffer[0] = '\0';
    HAL_StatusTypeDef status;
    for (uint16_t i = 0; i < sizeof(rx_buffer) / sizeof(rx_buffer[0]); i++)
    {
        status = HAL_UART_Receive(&SERVO_USART, rx_buffer + i, 1, SERVO_RECEIVE_TIMEOUT);
        if (mode == 0)
        {
            if (rx_buffer[i] == '\n') return 1;
            if (status != HAL_OK) return 0;
        }
        else if (mode == 1)
        {
            if (status == HAL_TIMEOUT) return rx_buffer[i];
            else if (status != HAL_OK) return 0;
        }
        else if (mode == 2){
            if (rx_buffer[i] == '\0') return 1;
            if (status != HAL_OK) return 0;
        }
    }
    return 0;
}

/**
 * @brief 单个电机角度控制函数。
 * 控制指定电机编号的电机按照指定的速度转动到指定的角度（绝对角度，相对于电机零点位置）。
 * 
 * @param id_num 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 * @param angle 电机角度（-360~360）*n，支持大角度转动
 * @param t mode=0,无作用，直接给0即可; mode=1, 运动时间（s）; mode =2, 前馈速度（r/min)
 * @param param mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加速度（(r/min)/s）; mode =2, 前馈扭矩（Nm)
 * @param mode 角度控制模式选择，电机支持三种角度控制模式，
 *             mode = 0: 多个电机轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
 *             mode = 1: 多个电机梯形轨迹模式，此时speed用运动时间t（s）表示，param为目标加速度（(r/min)/s）。
 *             mode = 2: 前馈控制模式，这种模式下的t为前馈速度，param为前馈扭矩。前馈控制在原有PID控制基础上加入速度和扭矩前馈，提高系统的响应特性和减少静态误差。
 * @note 在mode=1,梯形轨迹模式中，speed和accel都需要大于0.如果speed=0会导致电机报motor error 并退出闭环控制模式，所以在这种模式下如果speed=0,会被用0.01代替。
         另外如果这种模式下accel=0，电机以最快速度运动到angle,speed参数不再其作用。
 */
void preset_angle(uint8_t id_num, float angle, float t, float param, int mode)
{
    if (mode == 0) send_command(SERVO_SPRINTF(command, "ms %d %f %f %f\n", id_num, angle, fabs(t), fabs(param)));
    else if (mode == 1) send_command(SERVO_SPRINTF(command, "ms %d %f %f %f\n", id_num, angle, fabs(t), fabs(param)));
    else if (mode == 2) send_command(SERVO_SPRINTF(command, "ms %d %f %f %f\n", id_num, angle, t, fabs(param)));
}

/**
 * @brief 单个电机速度预设函数。
 * 预设指定电机编号的电机的目标速度，之后需要用mv指令启动转动。
 * 
 * @param id_num 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 * @param speed 目标速度（r/min）
 * @param param mode=1, 前馈扭矩（Nm); mode!=1,或目标加速度（(r/min)/s）
 * @param mode 控制模式选择
 *             mode=1, 速度前馈控制模式，电机将目标速度直接设为speed
 *             mode!=1,速度爬升控制模式，电机将按照目标加速度axis0.controller.config_.vel_ramp_rate变化到speed。
 */
void preset_speed(uint8_t id_num, float speed, float param, int mode)
{
    if (mode == 1) send_command(SERVO_SPRINTF(command, "ms %d %f %f %d \n", id_num, speed, param, INPUT_MODE_PASSTHROUGH));
    else send_command(SERVO_SPRINTF(command, "ms %d %f %f %d \n", id_num, speed, param, INPUT_MODE_VEL_RAMP));
}

/**
 * @brief 单个电机力矩预设函数。
 * 预设指定电机编号的电机目标扭矩（Nm）
 * 
 * @param id_num 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 * @param torque 电机输出（Nm)
 * @param param mode=1,改参数无意义；mode!=1,扭矩上升速度axis0.controller.config.torque_ramp_rate（Nm/s）
 * @param mode 控制模式选择
 *             mode=1, 扭矩直接控制模式，电机将目标扭矩直接设为torque
 *             mode!=1,扭矩爬升控制模式，电机将按照扭矩上升速率axis0.controller.config.torque_ramp_rate（Nm/s）变化到torque。
 */
void preset_torque(uint8_t id_num, float torque, float param, int mode)
{
    if (mode == 1) send_command(SERVO_SPRINTF(command, "ms %d %f 0 %d \n", id_num, torque, INPUT_MODE_PASSTHROUGH));
    else send_command(SERVO_SPRINTF(command, "ms %d %f %f %d \n", id_num, torque, param, INPUT_MODE_VEL_RAMP));
}

// 功能函数，用户使用

// 运动控制功能

/**
 * @brief 单个电机角度控制函数。
 * 控制指定电机编号的电机按照指定的速度转动到指定的角度（绝对角度，相对于电机零点位置）。
 * 
 * @param id_num 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 * @param angle 电机角度（-360~360）*n，支持大角度转动
 * @param speed 最大速度限制或前馈速度（r/min）
 * @param param mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加速度（(r/min)/s）; mode =2, 前馈扭矩（Nm)
 * @param mode 角度控制模式选择，电机支持三种角度控制模式，
 *             mode = 0: 轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
 *             mode = 1: 梯形轨迹模式，这种模式下可以指定运动过程中的速度（speed）和启停加速度（accel）。
 *             mode = 2: 前馈控制模式，这种模式下的speed和torque分别为前馈控制量.前馈控制在原有PID控制基础上加入速度和扭矩前馈，提高系统的响应特性和减少静态误差。
 * @note 在mode=1,梯形轨迹模式中，speed和accel都需要大于0.如果speed=0会导致电机报motor error 并退出闭环控制模式，所以在这种模式下如果speed=0,会被用0.01代替。
 *       另外如果这种模式下accel=0，电机以最快速度运动到angle,speed参数不再其作用。
 */
void set_angle(uint8_t id_num, float angle, float speed, float param, int mode)
{
    if (mode == 0) send_command(SERVO_SPRINTF(command, "t %d %f %f %f\n", id_num, angle, fabs(speed), fabs(param)));
    else if (mode == 1) send_command(SERVO_SPRINTF(command, "q %d %f %f %f\n", id_num, angle, fabs(speed), fabs(param)));
    else if (mode == 2) send_command(SERVO_SPRINTF(command, "p %d %f %f %f\n", id_num, angle, fabs(speed), fabs(param)));
}

/**
 * @brief 多个电机控制函数。
 * 控制指定电机编号的电机按照指定的速度转动到指定的角度，保证多个电机同时到达目标角度。
 * 
 * @param id_list 电机编号组成的数组
 * @param angle_list 电机角度组成的数组
 * @param speed 最大的电机转动的速度（r/min）或前馈速度
 * @param param mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加速度（(r/min)/s）; mode =2, 前馈速度（r/min)
 * @param mode 角度控制模式选择，电机支持三种角度控制模式，
 *             mode = 0: 多个电机轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
 *             mode = 1: 多个电机梯形轨迹模式，此时speed为多个电机中的最快速度（r/min），param为目标加速度（(r/min)/s）。
 *             mode = 2: 前馈控制模式，这种模式下的speed和torque分别为前馈控制量.前馈控制在原有PID控制基础上加入速度和扭矩前馈，提高系统的响应特性和减少静态误差。
 * @param n 数组长度
 */
void set_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n)
{
#if ENABLE_INPUT_VALIDITY_CHECK
    if (id_list == NULL || angle_list == NULL) return;
#endif
    static int last_count = 0;
    static uint8_t *last_id_list = NULL;
    static float *current_angle_list = NULL;

	uint8_t state = 0;
	for (size_t i = 0; i < n; i++)
	{
		if (last_id_list[i] != id_list[i])
		{
				state = 1;
				break;
		}
	}

    if (last_count != n || state)
    {
        last_count = n;
        SERVO_FREE(last_id_list);
        SERVO_FREE(current_angle_list);
        last_id_list = SERVO_MALLOC(n * sizeof(uint8_t));
        current_angle_list = SERVO_MALLOC(n * sizeof(float));
        memcpy(last_id_list, id_list, n * sizeof(uint8_t));
        for (size_t i = 0; i < n; i++)
            current_angle_list[i] = get_state(id_list[i]).angle; 
    }
    if (mode == 0)
    {
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i], speed, param, mode);
        send_command(strlen(strcpy(command, "mt 0 0\n")));
    }
    else if (mode == 1)
    {
        if (speed < 0 || param < 0) return;
        float delta_angle = 0;
        float t = 0;
        float fabs_temp;
        for (size_t i = 0; i < n; i++)
        {
            if (delta_angle < (fabs_temp = fabs(angle_list[i] - current_angle_list[i])))
                delta_angle = fabs_temp;
        }
        if (TRAJ_MODE == 2)
        {
            if(delta_angle <= (12 * speed * speed / fabs(param)))
            {
                speed = sqrt((fabs(param) * delta_angle) / 12);
                t = 4 * speed / fabs(param);
            }
            else
            {
                t = delta_angle / (6 * speed) + 2 * speed / fabs(param);
            }
        }
        else
        {
            if (delta_angle <= (6 * speed * speed / fabs(param)))
                t = 2 * sqrt(delta_angle / (6 * fabs(param)));
            else
                t = speed / fabs(param) + delta_angle / (6 * speed);
        }
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i], t, param, mode);
        send_command(strlen(strcpy(command, "mq 0 0\n")));
    }
    else if (mode == 2)
    {
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i], speed, param, mode);
        send_command(strlen(strcpy(command, "mp 0 0\n")));
    }
    memcpy(current_angle_list, angle_list, n * sizeof(float));
}

/**
 * @brief 单个电机相对角度控制函数。
 * 控制指定电机编号的电机按照指定的速度相对转动指定的角度（相对角度，相对于电机当前位置）。
 * 
 * @param id_num 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 * @param angle 电机相对角度（-360~360）*n，支持大角度转动
 * @param speed 最大速度限制或前馈速度（r/min）
 * @param param mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加速度（(r/min)/s）; mode =2, 前馈扭矩（Nm)
 * @param mode 角度控制模式选择，电机支持三种角度控制模式，
 *             mode = 0: 轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
 *             mode = 1: 梯形轨迹模式，这种模式下可以指定运动过程中的速度（speed）和启停加速度（accel）。
 *             mode = 2: 前馈控制模式，这种模式下的speed和torque分别为前馈控制量.前馈控制在原有PID控制基础上加入速度和扭矩前馈，提高系统的响应特性和减少静态误差。
 * @note 在mode=1,梯形轨迹模式中，speed和accel都需要大于0.如果speed=0会导致电机报motor error 并退出闭环控制模式，所以在这种模式下如果speed=0,会被用0.01代替。
 *       另外如果这种模式下accel=0，电机以最快速度运动到angle,speed参数不再其作用。
 */
void step_angle(uint8_t id_num, float angle, float speed, float param, int mode)
{
    step_angles(&id_num, &angle, speed, param, mode, 1);
}

/**
 * @brief 多个电机相对角度控制函数。
 * 控制指定电机编号的电机按照指定的时间相对转动给定角度。
 * 
 * @param id_list 电机编号组成的列表
 * @param angle_list 电机角度组成的列表
 * @param speed 最大的电机转动的速度（r/min）或前馈速度
 * @param param mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加速度（(r/min)/s）; mode =2, 前馈速度（r/min)
 * @param mode 角度控制模式选择，电机支持三种角度控制模式，
 *             mode = 0: 多个电机轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
 *             mode = 1: 多个电机梯形轨迹模式，此时speed为多个电机中的最快速度（r/min），param为目标加速度（(r/min)/s）。
 *             mode = 2: 前馈控制模式，这种模式下的speed和torque分别为前馈控制量.前馈控制在原有PID控制基础上加入速度和扭矩前馈，提高系统的响应特性和减少静态误差。
 * @param n 数组长度
 */
void step_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n)
{
#if ENABLE_INPUT_VALIDITY_CHECK
    if (id_list == NULL || angle_list == NULL) return;
#endif
    if (mode == 0)
	{
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i], speed, param, mode);
        send_command(SERVO_SPRINTF(command, "mt 0 1\n"));
    }
    else if (mode == 1)
    {
        if (speed <= 0 || param <= 0) return;
        float delta_angle = 0;
        float t = 0;
        float fabs_temp;
        for (size_t i = 0; i < n; i++)
        {
            if(delta_angle < (fabs_temp = fabs(angle_list[i])))
                delta_angle = fabs_temp;
        }
        if(TRAJ_MODE == 2)
        {
            if(delta_angle <= (12 * speed * speed / fabs(param)))
            {
                speed = sqrt((fabs(param) * delta_angle) / 12);
                t = 4 * speed / fabs(param);
            }
            else
            {
                t = delta_angle / (6 * speed) + 2 * speed / fabs(param);
            }
        }
        else
        {
            if(delta_angle <= (6 * speed * speed / fabs(param)))
                t = 2 * sqrt(delta_angle / (6 * fabs(param)));
            else
                t = speed / fabs(param) + delta_angle / (6 * speed);
        }
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i], t, param, mode);
        send_command(SERVO_SPRINTF(command, "mq 0 1\n"));
    }
    else if (mode == 2)
    {
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i], speed, param, mode);
        send_command(SERVO_SPRINTF(command, "mp 0 1\n"));
    }
}

/**
 * @brief 单个电机速度控制函数。
 * 控制指定电机编号的电机按照指定的速度连续整周转动。
 * 
 * @param id_num 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 * @param speed 目标速度（r/min）
 * @param param mode=1, 前馈扭矩（Nm); mode!=1,或目标加速度（(r/min)/s）
 * @param mode 控制模式选择
 *             mode=1, 速度前馈控制模式，电机将目标速度直接设为speed
 *             mode!=1,速度爬升控制模式，电机将按照目标加速度axis0.controller.config_.vel_ramp_rate变化到speed。
 * @note 在速度爬升模式下，如果目标加速度axis0.controller.config_.vel_ramp_rate设置为0，则电机速度将保持当前值不变。
 */
void set_speed(uint8_t id_num, float speed, float param, int mode)
{
    if (mode == 1) send_command(SERVO_SPRINTF(command, "v %d %f %f %d \n", id_num, speed, param, INPUT_MODE_PASSTHROUGH));
    else send_command(SERVO_SPRINTF(command, "v %d %f %f %d \n", id_num, speed, param, INPUT_MODE_VEL_RAMP));
}

/**
 * @brief 多个电机速度控制函数。
 * 控制指定多个电机编号的电机按照指定的速度连续整周转动。
 * 
 * @param id_list 电机编号组成的列表
 * @param speed_list 电机目标速度（r/min）组成的列表
 * @param param mode=1, 前馈扭矩（Nm); mode!=1,或目标加速度（(r/min)/s）
 * @param mode 控制模式选择
 *             mode=1, 速度前馈控制模式，电机将目标速度直接设为speed
 *             mode!=1,速度爬升控制模式，电机将按照目标加速度axis0.controller.config_.vel_ramp_rate变化到speed。
 * @param n 数组长度
 */
void set_speeds(uint8_t *id_list, float *speed_list, float param, float mode, size_t n)
{
    if (id_list == NULL || speed_list == NULL) return;
    for (size_t i = 0; i < n; i++)
        preset_speed(id_list[i], speed_list[i], param, mode);
    send_command(SERVO_SPRINTF(command, "mv 0\n"));
}

/**
 * @brief 单个电机力矩（电流）闭环控制函数。
 * 控制指定电机编号的电机输出指定的扭矩（Nm）
 * 
 * @param id_num 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 * @param torque 电机输出（Nm)
 * @param param mode=1,改参数无意义；mode!=1,扭矩上升速度axis0.controller.config.torque_ramp_rate（Nm/s）
 * @param mode 控制模式选择
 *             mode=1, 扭矩直接控制模式，电机将目标扭矩直接设为torque
 *             mode!=1,扭矩爬升控制模式，电机将按照扭矩上升速率axis0.controller.config.torque_ramp_rate（Nm/s）变化到torque。
 * @note 如果电机转速超过您设置的 vel_limit ，电机输出的力矩将会减小。
 *       可以设置 axis0.controller.config.enable_current_mode_vel_limit = False 来禁止力矩减小。
 *       另外在扭矩爬升控制模式下，如果点击扭矩上升速率axis0.controller.config.torque_ramp_rate为0，则点击扭矩将在当前值保持不变。
 */
void set_torque(uint8_t id_num, float torque, float param, int mode)
{
    if (mode == 1) send_command(SERVO_SPRINTF(command, "c %d %f 0 %d \n", id_num, torque, INPUT_MODE_PASSTHROUGH));
    else send_command(SERVO_SPRINTF(command, "c %d %f %f %d \n", id_num, torque, param, INPUT_MODE_VEL_RAMP));
}

/**
 * @brief 多个个电机力矩控制函数。
 * 同时控制多个电机编号的电机目标扭矩（Nm）
 * 
 * @param id_list 电机编号组成的列表
 * @param torque_list 电机目标扭矩（Nm)组成的列表
 * @param param mode=1,改参数无意义；mode!=1,扭矩上升速度axis0.controller.config.torque_ramp_rate（Nm/s）
 * @param mode 控制模式选择
 *             mode=1, 扭矩直接控制模式，电机将目标扭矩直接设为torque
 *             mode!=1,扭矩爬升控制模式，电机将按照扭矩上升速率axis0.controller.config.torque_ramp_rate（Nm/s）变化到torque。
 * @param n 数组长度
 */
void set_torques(uint8_t *id_list, float *torque_list, float param, int mode, size_t n)
{
    if (id_list == NULL || torque_list == NULL) return;
    for (size_t i = 0; i < n; i++)
        preset_torque(id_list[i], torque_list[i], param, mode);
    send_command(SERVO_SPRINTF(command, "mc 0\n"));
}

/**
 * @brief 单个电机阻抗控制函数。
 * 对指定电机编号的电机进行阻抗控制。
 * 
 * @param id_num 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 * @param pos 电机目标角度（度）
 * @param vel 电机目标速度（r/min）
 * @param tff 前馈扭矩（Nm)
 * @param kp 刚度系数(rad/Nm)
 * @param kd 阻尼系数(rad/s/Nm)
 * @note 阻抗控制为MIT开源方案中的控制模式，其目标输出扭矩计算公式如下：
         torque = kp*( pos – pos_) + t_ff + kd*(vel – vel_)
         其中pos_和vel_分别为输出轴当前实际位置（degree）和当前实际速度（r/min）, kp和kd为刚度系数和阻尼系数，系数比例与MIT等效
 */
void impedance_control(uint8_t id_num, float pos, float vel, float tff, float kp, float kd)
{
    send_command(SERVO_SPRINTF(command, "d %d %f %f %f %f %f\n", id_num, pos, vel, tff, kp, kd));
}

/**
 * @brief 急停函数
 * 控制电机紧急停止。电机急停后将切换到IDLE待机模式，电机卸载并生成ERROR_ESTOP_REQUESTED错误标志，不再响应set_angle/speed/torque指令。
 * 如果要恢复正常控制模式，需要首先用clear_error清除错误标志后,然后用set_mode函数将模式设置为2（闭环控制模式）。
 * 
 * @param id_num 需要急停的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 */
void estop(uint8_t id_num)
{
    send_command(SERVO_SPRINTF(command, "st %d\n", id_num));
}

// 参数设置功能

/**
 * @brief 设置电机ID号。
 * 改变电机ID号（掉电保存）
 * 
 * @param id_num 需要重新设置编号的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则多个电机会被设置成相同编号
 * @param new_id 新电机编号，电机ID号范围为1~63
 */
void set_id(uint8_t id_num, int new_id)
{
    write_property(id_num, "axis0.config.can_node_id", new_id);
    save_config(new_id);
}

/**
 * @brief 设置电机串口波特率。
 * 设置UART串口波特率（掉电保存）
 * 
 * @param id_num 需要重新设置波特率的电机编号,如果不知道当前电机编号，可以用0广播。
 * @param baud_rate uart串口波特率，支持9600,19200,57600,115200中任意一种，修改成功后需手动将主控UART波特率也修改为相同值
 * @note 这个串口波特率只对UART总线（TX/RX）接口有效，USB接口中的串口为虚拟串口，波特率不用设置，可以自动适应上位机的波特率。
 */
void set_uart_baud_rate(uint8_t id_num, int baud_rate)
{
    write_property(id_num, "config.uart_baudrate", baud_rate);
    save_config(id_num);
}

/**
 * @brief 设置电机CAN波特率。
 * 设置CAN波特率（掉电保存）
 * 
 * @param id_num 需要重新设置波特率的电机编号,如果不知道当前电机编号，可以用0广播。
 * @param baud_rate CAN波特率，支持125k,250k,500k,1M中任意一种,修改成功后需手动将主控CAN波特率也修改为相同值。
 */
void set_can_baud_rate(uint8_t id_num, int baud_rate)
{
    write_property(id_num, "can.config.baud_rate", baud_rate);
    save_config(id_num);
}


/**
 * @brief 设置电机模式。
 * 设置电机进入不同的控制模式。
 * 
 * @param id_num 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 * @param mode 电机模式编号
 *             mode = 1: IDLE待机模式，电机将关掉PWM输出，电机卸载
 *             mode = 2: 闭环控制模式，set_angle， set_speed, set_torque函数必须在闭环控制模式下才能进行控制。（电机上电后的默认模式）
 * @note 模式3和模式4是用来校准电机和编码器参数，出厂前已完成校准，正常情况下不要使用。
 */
void set_mode(uint8_t id_num, int mode)
{
    if (mode == 1) write_property(id_num, "axis.requested_state", AXIS_STATE_IDLE);
    else if (mode == 2) write_property(id_num, "axis.requested_state", AXIS_STATE_CLOSED_LOOP_CONTROL);
}

/**
 * @brief 设置电机零点位置函数
 * 设置当前位置为电机输出轴零点，设置完后当前位置为0度
 * 
 * @param id_num 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 */
void set_zero_position(uint8_t id_num)
{
    send_command(SERVO_SPRINTF(command, "sz %d\n", id_num));
}

/**
 * @brief 设置GPIO控制接口模式
 * 设置电机预留的两个引脚模式，支持的模式有UART串口和Step/Dir接口，通过调用该函数进行切换并设置基本参数
 * 
 * @param id_num 需要设置的电机编号,如果不知道当前电机编号，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 * @param mode 需要选择的模式，mode == 0 表示选择uart串口模式，mode == 1表示选择Step/Dir接口控制模式
 * @param param 当mode='tx_rx'时，param表示串口的波特率，支持9600、19200、57600、115200其中一种；当mode='step_dir'时，param表示电机转一圈对应的脉冲数，支持1-1024(必须是整数)
 */
void set_GPIO_mode(uint8_t id_num, uint8_t mode, uint32_t param)
{
#if ENABLE_INPUT_VALIDITY_CHECK
    if (mode != 0 && mode != 1) return;
#endif
    const uint32_t sleep_time = 100;
    uint8_t enable_uart, enable_step_dir;
    get_GPIO_mode(id_num, &enable_uart, &enable_step_dir, NULL);

    if (enable_uart != !mode)
    {
        write_property(id_num, "config.enable_uart", !mode);
        SERVO_DELAY(sleep_time);
    }
    if (enable_step_dir != mode)
    {
        write_property(id_num, "axis0.config.enable_step_dir", mode);
        SERVO_DELAY(sleep_time);
    }
    if (mode == 0) write_property(id_num, "config.uart_baudrate", param);
    else if (mode == 1) write_property(id_num, "axis0.output_shaft.steps_per_turn", param);
    SERVO_DELAY(sleep_time);
    save_config(id_num);
    SERVO_DELAY(sleep_time);
    reboot(id_num);
}

/**
 * @brief 设置电机软件限位极限位置
 * 设置电机预输出轴软件限位极限位置值，设置成功后电机在位置、速度及扭矩控制模式电机输出轴将被限制在[angle_min, angle_max]范围内
*  （注意：当前输出轴位置必须在[angle_min, angle_max]范围内，否则将设置失败）
 * 
 * @param id_num 需要设置的电机编号,如果不知道当前电机编号，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 * @param angle_min 软件限位最小角度（该参数与axis0.output_shaft.circular_setpoint_min对应）
 * @param angle_max 软件限位最大角度（该参数与axis0.output_shaft.circular_setpoint_max对应）
 * @return 是否设置成功
 */
int8_t set_angle_range(uint8_t id_num, float angle_min, float angle_max)
{
    float cur_angle = get_state(id_num).angle;
    if (cur_angle < angle_min || cur_angle > angle_max || READ_FLAG != 1) return -1;
    write_property(id_num, "axis0.output_shaft.circular_setpoint_min", angle_min);
    write_property(id_num, "axis0.output_shaft.circular_setpoint_max", angle_max);
    write_property(id_num, "axis0.config.extra_setting.enable_circular_setpoint_limit", 1);
    save_config(id_num);
    return 1;
}

/**
 * @brief设置速度轨迹模式下速度曲线类型
 * 设置速度轨迹模式，1表示梯形轨迹模式，2表示S形轨迹模式（指的是速度曲线形状，适用于位置控制-梯形轨迹模式）
 *
 * @param id_num: 需要设置的电机编号,如果不知道当前电机编号，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 * @param   mode: 1表示梯形轨迹，2表示S形轨迹（指的是速度曲线形状，适用于位置控制-梯形轨迹模式）
 * @return 无
*/
void set_traj_mode(uint8_t id_num,int mode)
{
    write_property(id_num, "axis0.trap_traj.config.traj_mode", mode);
    TRAJ_MODE = mode;
}

/**
 * @brief 修改电机属性参数
 * 修改电机属性参数，这里的属性参数为电机控制参数
 * 
 * @param id_num 需要修改的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 * @param property 需要读取的属性参数名称，例如"vbus_voltage"，"axis0.config.can_node_id"等，具体参数名称见enums.py文件里property_address字典里的键值。
 * @param value 对应参数的目标值。
 */
void write_property(uint8_t id_num, char *property, float value)
{
#if ENABLE_INPUT_VALIDITY_CHECK
    if (property == NULL) return;
#endif
    if (strstr(property, "odrv") != NULL)
        property = strstr(property, ".") + 1;
    if (strstr(property, "axis") != NULL)
    {
        property = strstr(property, ".") + 1;
        if (round(value) == value) send_command(SERVO_SPRINTF(command, "w axis%d.%s %d \n", id_num, property, (int)value));
        else send_command(SERVO_SPRINTF(command, "w axis%d.%s %f \n", id_num, property, value));
    }
    else
    {
        if (round(value) == value) send_command(SERVO_SPRINTF(command, "w %s %d %d \n", property, (int)value, id_num));
        else send_command(SERVO_SPRINTF(command, "w %s %f %d \n", property, value, id_num));
    }
}

// 参数回读功能

/**
 * @brief 读取电机ID。
 * 
 * @param id_num 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。
 * @return 电机的 ID 号
 */
uint8_t get_id(uint8_t id_num)
{
    return read_property(id_num, "axis0.config.can_node_id");
}

/**
 * @brief 读取电机的当前位置和速度
 * 读取电机输出轴当前位置和速度列表，单位分别为度（°）和转每分钟(r/min)
 * 
 * @param id_num 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。
 * @return struct servo_state 储存电机位置和速度的结构体
 */
struct servo_state get_state(uint8_t id_num)
{
    PRE_READ();
    send_command(SERVO_SPRINTF(command, "f %d \n", id_num));
    struct servo_state state = {0, 0};
    if (read_data(0))
    {
        READ_FLAG = 1;
        state.angle = (float)atof(strtok((char *)rx_buffer, " "));
        state.speed = (float)atof(strtok(NULL, " "));
    }
    else READ_FLAG = -1;
    return state;
}

/**
 * @brief 读取电机的当前电压和电流
 * 读取电机当前电压和q轴电流列表，单位分别为伏（V）和安(A)
 * 
 * @param id_num 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。
 * @return struct servo_volcur 储存电机电压和电流的结构体
 */
struct servo_volcur get_volcur(uint8_t id_num)
{
    PRE_READ();
    send_command(SERVO_SPRINTF(command, "g %d \n", id_num));
    struct servo_volcur volcur = {0, 0};
    if (read_data(0))
    {
        READ_FLAG = 1;
        volcur.vol = (float)atof(strtok((char *)rx_buffer, " "));
        volcur.cur = (float)atof(strtok(NULL, " "));
    }
    READ_FLAG = -1;
    return volcur;
}

/**
 * @brief 读取GPIO控制接口模式
 * 读取电机预留的两个引脚当前模式及参数，支持的模式有UART串口和Step/Dir接口
 * 
 * @param id_num 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。
 * @param enable_uart 是否为 uart 模式
 * @param enable_step_dir 是否为 step dir 模式
 * @param n 串口波特率或脉冲数/每圈
 * @return 是否读取成功 
 */
int8_t get_GPIO_mode(uint8_t id_num, uint8_t *enable_uart, uint8_t *enable_step_dir, uint32_t *n)
{
    if (enable_uart != NULL) *enable_uart = read_property(id_num, "config.enable_uart") == 0 ? 0 : 1;
    if (enable_step_dir != NULL) *enable_step_dir = read_property(id_num, "axis0.config.enable_step_dir") == 0 ? 0 : 1;
    if (*enable_uart && !*enable_step_dir)
    {
        if (n != NULL)
            *n = read_property(id_num, "config.uart_baudrate");
        return 1;
    }
    else if (!*enable_uart && *enable_step_dir)
    {
        if (n != NULL)
            *n = read_property(id_num, "axis0.output_shaft.steps_per_turn");
        return 1;
    }
    return -1;
}

/**
 * @brief 读取电机属性参数
 * 读取电机属性参数，这里的属性参数包括电机状态量及电机控制参数
 * 
 * @param id_num 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。
 * @param property 需要读取的属性参数名称，例如"vbus_voltage"，"axis0.config.can_node_id"等，具体参数名称见enums.py文件里property_address字典里的键值。
 * @return 对应属性参数的值
 */
float read_property(uint8_t id_num, char *property)
{
#if ENABLE_INPUT_VALIDITY_CHECK
    if (property == NULL) return 0;
#endif
    if (strstr(property, "odrv") != NULL)
        property = strstr(property, ".") + 1;
    PRE_READ();
    if (strstr(property, "axis") != NULL)
    {
        property = strstr(property, ".") + 1;
        send_command(SERVO_SPRINTF(command, "r axis%d.%s \n", id_num, property));
    }
    else
        send_command(SERVO_SPRINTF(command, "r %s %d \n", property, id_num));
    if (!read_data(0))
    {
        READ_FLAG = -1;
        return 0;
    }
    READ_FLAG = 1;
    return atof((char *)rx_buffer);
}

// 辅助功能

/**
 * @brief 清除错误标志函数
 * 一旦电机运行过程中出现任何错误，电机将进入IDLE模式，如果要恢复正常控制模式，需要首先用clear_error清除错误标志后,然后用set_mode函数将模式设置为2（闭环控制模式）。
 * 
 * @param id_num 需要清除错误标志的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 */
void clear_error(uint8_t id_num)
{
    send_command(SERVO_SPRINTF(command, "sc %d\n", id_num));
}

/**
 * @brief 打印电机错误编号
 * 读取电机错误信息编码，如果错误编码为0，表示无异常。如果错误编码不为0，则表示存在故障。
 * 
 * @param id_num 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。
 * @param error 输出的错误信息
 * @return 是否读取成功
 */
int8_t dump_error(uint8_t id_num, char *error)
{
    int8_t x = 1;
    PRE_READ();
    send_command(SERVO_SPRINTF(command, "e %d \n", id_num));
    if (!read_data(2)) x = -1;
    if (error != NULL) strcpy(error, (char *)rx_buffer);
    return x;
}

/**
 * @brief 保存配置函数
 * 正常情况下，通过write_property修改的属性电机上电重启之后，会恢复为修改前的直，如果想永久保存，则需要用save_config函数将相关参数保存到flash中，掉电不丢失。
 * 
 * @param id_num 需要保存配置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 */
void save_config(uint8_t id_num)
{
    send_command(SERVO_SPRINTF(command, "ss %d\n", id_num));
}

/**
 * @brief 电机重启函数
 * 电机软件重启，效果与重新上电类似。
 * 
 * @param id_num 需要重启的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 */
void reboot(uint8_t id_num)
{
    send_command(SERVO_SPRINTF(command, "sr %d\n", id_num));
}

/**
 * @brief 单个电机等待函数
 * 延时等待直到给定电机到达目标位置(只对角度控制指令有效)
 * 
 * @param id_num 需要重启的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 ** @return 无
 */
void position_done(uint8_t id_num)
{
	int traj_done = 0;
	while(traj_done == 0 || READ_FLAG == -1)
	{
      traj_done = read_property(id_num, "axis0.controller.trajectory_done");
	}
}
/**
 * @brief 多个电机等待函数
 * 程序等待（阻塞）直到所有电机都到达目标位置(只对角度控制指令有效)
 * 
 * @param id_list: 电机编号组成的列表
 ** @return 无
 */
void positions_done(uint8_t *id_list,size_t n)
{
    for (size_t i = 0; i < n; i++)
	{
        position_done(id_list[i]);
	}
}