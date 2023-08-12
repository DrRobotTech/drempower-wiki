/**************************************************
大然机器人-外转子盘式无刷伺服驱动器库

适用平台：windows或linux平台
库版本号：v2.0
测试主控版本：windows 10 python 3.7
测试人员：唐昭
测试时间：2021.10.27
*************************************************/

#include <string.h>
#include <stdlib.h>
#include <math.h>
//#include <SPI.h>
//#include <mcp2515.h>

//#include <stdio.h>
#include "DrEmpower_can.h"
//#include "DrEmpower.h"

#define INPUT_MODE_PASSTHROUGH 1
#define INPUT_MODE_VEL_RAMP 2
#define AXIS_STATE_IDLE 1
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8
#define INPUT_MODE_TORQUE_RAMP  6

//#define SERVO_USART huart1              /* 驱动电机使用的串口 */
#define SERVO_RECEIVE_TIMEOUT 1000        /* 串口接收超时(ms) */
#define ENABLE_INPUT_VALIDITY_CHECK 1   /* 输入合法性检查。为 0 时不编译空指针判断等检查，可减小程序体积、加快处理速度。 */
#define DTOSTRF_PREC 5


#define SERVO_MALLOC(size) malloc(size) /* 如果使用了操作系统，需要将这个函数换成操作系统对应的函数 */
#define SERVO_FREE(ptr) free(ptr)       /* 如果使用了操作系统，需要将这个函数换成操作系统对应的函数 */
#define SERVO_DELAY(n) delay(n)        /* 如果使用了操作系统，需要将这个函数换成操作系统对应的函数 */
#define SERVO_SPRINTF servo_sprintf     /* 如果无法使用 sprintf 输出浮点数，则要用精简版的 servo_sprintf 代替 */
//#define PRE_READ() ({while (Serial.available()) Serial.read();})

char command[64];       // 命令发送缓冲区
char DaTemp[4];
uint8_t i;
int8_t READ_FLAG=0;  // 读取结果标志位
uint8_t rx_buffer[8];   //读取缓冲区
uint16_t can_id = 0x00;

int8_t TRAJ_MODE = 1; //速度轨迹模式选择控制，1表示梯形轨迹模式，2表示S形轨迹模式（指的是速度曲线形状，适用于位置控制-梯形轨迹模式）

int8_t enable_replay_state = 0;  //如需要打开运动控制指令实时状态返回功能，请将该变量改为1，并将下面的MOTOR_NUM设置为总线上的最大电机ID号
#define MOTOR_NUM  16             
float motor_state[MOTOR_NUM][5];   //电机状态二维数组，通过motor_state[id_num-1]获取电机id_num的实时返回状态[angle,speed,torque,traj_done,axis_error]，单位分别为degree，r/min,Nm，三个变量值均指的是电机输出轴
// # 其中motor_state[id_num-1][0]表示id_num号电机的角度，motor_state[id_num-1][1]表示id_num号电机的速度，motor_state[id_num-1][2]表示id_num号电机的输出扭矩
uint8_t reply_state_error = 0;   // reply_state错误次数累积标志

struct can_frame canMsg;
MCP2515 mcp2515(10);

//CAN MCP2515 初始化函数
void MCP2515_CAN_Init(void)
{
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS,MCP_8MHZ);
  mcp2515.setNormalMode();
}

//CAN数据接收函数
void MCP2515_CAN_readMessage(void)
{
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
   {
       READ_FLAG =1;
       for (int i = 0; i<canMsg.can_dlc; i++) 
        {  
           rx_buffer[i]=canMsg.data[i];
        } 
        can_id = canMsg.can_id;
    }  
}

//cur_angle_list = [];  //当前角度列表

//"""
//内部辅助函数，用户无需使用
//"""

// CAN发送函数
void send_command(uint8_t id_num, char cmd, unsigned char *data,uint8_t rt )
{

     canMsg.can_id=(id_num << 5) + cmd;
     canMsg.can_dlc = 8;
     for(i=0;i<8;i++)
     {
        canMsg.data[i]=data[i];
     }
    mcp2515.sendMessage(&canMsg);
}


//CAN接收函数
void receive_data(void)
{
    uint8_t OutTime,OutTime_mark;
	OutTime_mark=0;
 do{
      MCP2515_CAN_readMessage();
	  if( READ_FLAG ==1) OutTime_mark=1;
	   SERVO_DELAY(100);
	   OutTime++;
	   if( OutTime ==100) OutTime_mark=1;
	}
	while(OutTime_mark==0);
   
}

// 数据格式转换函数，decode是将二进制(bytes)转化成人看的懂得数据，encode反之
/*
float，short，unsigned short，int，unsigned int五种数据类型与byte类型的转换
五种数据对应名：0,1,2,3,4;
使用方法，在函数中调用时：
    首先要在调用前对结构体内的数据进行赋值
    value_data[3]：若将五种类型的数据转换为byte，则赋值
    byte_data[8]：若将byte转换为五种数据的类型，则赋值
    type_data[3]类型名赋值（必要）
    length：数据个数赋值（必要）

若将五种类型的数据转换为byte，调用format_data(data_struct data_list , char * str)，
    参数为：结构体指针，要做的操作（输入“encode”）
若将byte转换为五种数据的类型，调用format_data(data_struct data_list , char * str)，
    参数为：结构体指针，要做的操作（输入“decode”）

下面两个函数在上述两个函数中自动调用，不需要使用者主动调用
    byte2value()
    value2byte()

type类型：
type_data=0, 浮点数（float）,数据长度32位,符号‘f’;
type_data=1, 无符号短整数（unsigned short int）,数据长度16位,符号‘u16’;
type_data=2, 有符号短整数（short int）,数据长度16位,符号‘s16’;
type_data=3, 无符号短整数（unsigned int）,数据长度32位,符号‘u32’;
type_data=4, 有符号短整数（int）,数据长度32位,符号‘s32’;

*/
struct format_data_struct
{
    float value_data[3];//若将五种类型的数据转换为byte，则赋值
    unsigned char byte_data[8];//若将byte转换为五种数据的类型，则赋值
    int type_data[3];//类型名赋值（必要）
    int length;//数据个数赋值（必要）
}*data_struct,data_list;  //定义全局变量 data_list，用于进行CAN数据encode和decode过程中存储参数输入及结果输出

// note: the bit order is in accordance with intel little endian
static inline void uint16_to_data(uint16_t val, uint8_t *data)
{
    data[1] = (uint8_t)(val >> 8);
    data[0] = (uint8_t)(val);
}
static inline uint16_t data_to_uint16(uint8_t *data)
{
    uint16_t tmp_uint16;
    tmp_uint16 = (((uint32_t)data[1] << 8)  + ((uint32_t)data[0]));
    return tmp_uint16;
}
static inline void uint_to_data(uint32_t val, uint8_t *data)
{
    data[3] = (uint8_t)(val >> 24);
    data[2] = (uint8_t)(val >> 16);
    data[1] = (uint8_t)(val >> 8);
    data[0] = (uint8_t)(val);
}
static inline uint32_t data_to_uint(uint8_t *data)
{
    uint32_t tmp_uint;
    tmp_uint = (((uint32_t)data[3] << 24) + ((uint32_t)data[2] << 16) + ((uint32_t)data[1] << 8)  + ((uint32_t)data[0]));
    return tmp_uint;
}
static inline void int16_to_data(int16_t val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
}
static inline int16_t data_to_int16(uint8_t *data)
{
    int16_t tmp_int16;
    *(((uint8_t*)(&tmp_int16)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int16)) + 1) = data[1];
    return tmp_int16;
}
static inline void int_to_data(int val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}
static inline int data_to_int(uint8_t *data)
{
    int tmp_int;
    *(((uint8_t*)(&tmp_int)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int)) + 1) = data[1];
    *(((uint8_t*)(&tmp_int)) + 2) = data[2];
    *(((uint8_t*)(&tmp_int)) + 3) = data[3];
    return tmp_int;
}
static inline void float_to_data(float val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}
static inline float data_to_float(uint8_t *data)
{
    float tmp_float;
    *(((uint8_t*)(&tmp_float)) + 0) = data[0];
    *(((uint8_t*)(&tmp_float)) + 1) = data[1];
    *(((uint8_t*)(&tmp_float)) + 2) = data[2];
    *(((uint8_t*)(&tmp_float)) + 3) = data[3];
    return tmp_float;
}

//不需主动调用
void byte2value()
{
    int value_index = 0;
    int byte_index = 0;
    while (1)
    {
        switch(data_list.type_data[value_index])
        {
        case 0:
        {
            data_list.value_data[value_index] = (float)data_to_float(&data_list.byte_data[byte_index]);
            byte_index = 4+byte_index;
            value_index++;
        }
        break;
        case 1:
        {
            data_list.value_data[value_index] = (float)data_to_uint16(&data_list.byte_data[byte_index]);
            byte_index = 2+byte_index;
            value_index++;
        }
        break;
        case 2:
        {
            data_list.value_data[value_index] = (float)data_to_int16(&data_list.byte_data[byte_index]);
            byte_index = 2+byte_index;
            value_index++;
        }
        break;
        case 3:
        {
            data_list.value_data[value_index] = (float)data_to_uint(&data_list.byte_data[byte_index]);
            byte_index = 4+byte_index;
            value_index++;
        }
        break;
        case 4:
        {
            data_list.value_data[value_index] = (float)data_to_int(&data_list.byte_data[byte_index]);
            byte_index = 4+byte_index;
            value_index++;
        }
        break;
        default:
            value_index++;
            break;
        }
        if((byte_index>=8)||(value_index>=data_list.length))
        {
            return;
        }
    }
}
//不需主动调用
void value2byte()
{
    int byte_index=0;
    int value_index=0;
    while(1)
    {
        if (data_list.type_data[value_index]==0)
        {
            float_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        switch(data_list.type_data[value_index])
        {
        case 0:
        {
            float_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        break;
        case 1:
        {
            uint16_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 2;
            value_index += 1;
        }
        break;
        case 2:
        {
            int16_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 2;
            value_index += 1;
        }
        break;
        case 3:
        {
            uint_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        break;
        case 4:
        {
            int_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        break;
        default:
            value_index++;
            break;
        }
        if((byte_index >=8)||(value_index>=data_list.length))
        {
            return;
        }
    }
}

//参数结构体指针，要做的操作（五种type转byte输入“encode”，byte转五种type输入“decode”）
void format_data( float *value_data, int *type_data,int length, char * str)
{
    data_list.length=length;
    for (int i = 0; i < length; i++)
    {
        data_list.value_data[i]= value_data[i];
        data_list.type_data[i] = type_data[i];
    }
    if (strcmp(str,"encode")==0)
    {
        value2byte();
    }
    if (strcmp(str,"decode")==0)
    {
        for (int i = 0; i < 8; i++)
        {
            data_list.byte_data[i]=rx_buffer[i];
        }
        byte2value();
    }
}

/**
 * @brief 电机运动控制指令状态实时返回参数
 * 通过该函数读取电机运动控制指令实时返回的电机状态参数[angle,speed,torque]，单位分别为degree，r/min,Nm，三个变量值均指的是电机输出轴
 * 其中motor_state[id_num-1][0]表示id_num号电机的角度，motor_state[id_num-1][1]表示id_num号电机的速度，motor_state[id_num-1][2]表示id_num号电机的输出扭矩
 * @param id_num 需要读取的电机编号  注意该指令id_num不能为0

 */
void reply_state(uint8_t id_num)
{
    if(enable_replay_state&&id_num<=MOTOR_NUM)   //id_num不能为0
    {
        READ_FLAG=0;
        receive_data();
        if (READ_FLAG == 1)
        {
            if(id_num==0)  //如果ID号为0，则通过返回数据帧的ID信息更新ID号
            {
                id_num = (uint8_t)((can_id & 0x07E0) >> 5)&0xFF;
            }
            float factor = 0.01f;
            float value_data[3]= {0,0,0};
            int type_data[3]= {0,2,2};
            format_data(value_data,type_data,3,"decode");
            motor_state[id_num-1][0]=data_list.value_data[0];
            motor_state[id_num-1][1]=data_list.value_data[1]*factor;
            motor_state[id_num-1][2]=data_list.value_data[2]*factor;
            motor_state[id_num -1][3] =(int)((can_id & 0x02) >> 1);
            motor_state[id_num -1][4] = (int)((can_id & 0x04) >> 2);
        }
        else
        {
            READ_FLAG=-1;
            reply_state_error += 1;
        }
    }
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

    float factor = 0.01;
    if (mode == 0)
    {
        float f_angle = angle;
        int s16_time = (int)(abs(t) / factor);
        if (param > 300)
            //print("input_filter_width = " + str(param) + ", which is too big and resized to 300")
            param = 300;
        int  s16_width = (int)(abs(param / factor));
        
        float value_data[3]={f_angle, s16_time, s16_width};
        int type_data[3]={0,2,2};
        
        format_data(value_data,type_data,3,"encode");
        
        send_command(id_num,0x0C,data_list.byte_data,0);
    }
    else if(mode == 1)
    {
        float f_angle = angle;
        int s16_time = (int)(abs(t) / factor);
        int s16_accel = (int)((abs(param)) / factor);
        
        float value_data[3]={f_angle,s16_time,s16_accel};
        int type_data[3]={0,2,2};
        
        format_data(value_data,type_data,3,"encode");
        send_command(id_num,0x0C,data_list.byte_data,0);
    }
    else if(mode == 2)
    {
        float f_angle = angle;
        int s16_speed_ff = (int)((t) / factor);
        int s16_torque_ff = (int)((param) / factor);
        
        float value_data[3]={f_angle,s16_speed_ff,s16_torque_ff};
        int type_data[3]={0,2,2};
        
        format_data(value_data,type_data,3,"encode");
        send_command(id_num,0x0C,data_list.byte_data,0);
    }
    reply_state(id_num);
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

    float factor = 0.01;
        float f_speed = speed;
        if (mode == 1)
        {
           int  s16_torque = (int)((param) / factor);
            if (f_speed == 0)
                s16_torque = 0;
            int s16_input_mode = (int)(INPUT_MODE_PASSTHROUGH / factor);
           
            float value_data[3]={f_speed, s16_torque, s16_input_mode};
            int type_data[3]={0,2,2};
            
            format_data(value_data,type_data,3,"encode");
        }
        else
        {
            int s16_ramp_rate = (int)((param) / factor);
            int s16_input_mode = (int)(INPUT_MODE_VEL_RAMP / factor);
            
            float value_data[3]={f_speed, s16_ramp_rate, s16_input_mode};
            int type_data[3]={0,2,2};
            
            format_data(value_data,type_data,3,"encode");
        }
        
        send_command(id_num, 0x0C,data_list.byte_data, 0);
        reply_state(id_num);
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
    
    float factor = 0.01;
        float f_torque = torque;
				int s16_ramp_rate, s16_input_mode;
        if (mode == 1)
        {
            s16_input_mode = (int)(INPUT_MODE_PASSTHROUGH / factor);
            s16_ramp_rate = 0;
        }
        else
        {
             s16_input_mode = (int)(INPUT_MODE_TORQUE_RAMP / factor);
             s16_ramp_rate =  (int)((param) / factor);
        }
        
        float value_data[3]={f_torque, s16_ramp_rate, s16_input_mode};
        int type_data[3]={0,2,2};
        
        format_data(value_data,type_data,3,"encode");
        
        send_command(id_num,0x0C,data_list.byte_data,0);
        reply_state(id_num);
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

    float factor = 0.01;
        if (mode == 0)
        {
            float f_angle = angle;
            int s16_speed = (int)((abs(speed)) / factor);
            if (param > 300)
                param = 300;
            int s16_width = (int)(abs(param / factor));
            
            float value_data[3]={f_angle,s16_speed,s16_width};
            int type_data[3]={0,2,2};
            
            format_data(value_data,type_data,3,"encode");
            
            send_command(id_num,0x19,data_list.byte_data,0);
        }
        else if (mode == 1)
            {
            if ((speed > 0)&&(param > 0))
            {
                float f_angle = angle;
                int s16_speed = (int)((abs(speed)) / factor);
                int s16_accel = (int)((abs(param)) / factor);
                
                float value_data[3]={f_angle,s16_speed,s16_accel};
                int type_data[3]={0,2,2};
                
                format_data(value_data,type_data,3,"encode");
                
                send_command(id_num,0x1A,data_list.byte_data,0);
            }
            }
        else if (mode == 2)
           {
            float f_angle = angle;
            int s16_speed_ff = (int)((speed) / factor);
            int s16_torque_ff = (int)((param) / factor);
            
            float value_data[3]={f_angle,s16_speed_ff,s16_torque_ff};
            int type_data[3]={0,2,2};
            
            format_data(value_data,type_data,3,"encode");
            
            send_command(id_num, 0x1B, data_list.byte_data, 0);
           }
           reply_state(id_num);
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
                preset_angle(id_list[i],angle_list[i],speed,param,mode);
            unsigned char order_num = 0x10;
            
            float value_data[3]={order_num,0,0};
            int type_data[3]={3,1,1};
            
            format_data(value_data,type_data,3,"encode");
            
            send_command(0,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
        }
        else if (mode == 1)
            {
             if ((speed < 0) ||( param < 0)) return;
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
                    preset_angle(id_list[i],angle_list[i],t,param,mode);
                unsigned char order_num = 0x11;
				
                float value_data[3]={order_num,0,0};
                int type_data[3]={3,1,1};
                format_data(value_data,type_data,3,"encode");

                send_command(0,0x08,data_list.byte_data,0); // 需要用标准帧（数据帧）进行发送，不能用远程帧
            }
        else if( mode == 2)
        {
            for (size_t i = 0; i < n; i++)
                preset_angle(id_list[i], angle_list[i],speed,param,mode);
            unsigned char order_num = 0x12;
            
            float value_data[3]={order_num,0,0};
            int type_data[3]={3,1,1};
            format_data(value_data,type_data,3,"encode");
            
            send_command(0,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
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

    step_angles(&id_num,&angle,speed,param,mode,1);

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
            unsigned char order_num = 0x10;
            
            float value_data[3]={order_num,0,0};
            int type_data[3]={3,1,1};
            format_data(value_data,type_data,3,"encode");
            
            send_command(0,0x08,data_list.byte_data,0); //需要用标准帧（数据帧）进行发送，不能用远程帧
        }
        else if( mode == 1)
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
                unsigned char order_num = 0x11;
                
                float value_data[3]={order_num,0,0};
                int type_data[3]={3,1,1};
                format_data(value_data,type_data,3,"encode");
                
                send_command(0, 0x08, data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
        }
        else if (mode == 2)
        {
            for (size_t i = 0; i < n; i++)
                preset_angle(id_list[i], angle_list[i], speed, param, mode);
            unsigned char order_num = 0x12;
            
            float value_data[3]={order_num,0,0};
            int type_data[3]={3,1,1};
            format_data(value_data,type_data,3,"encode");
            
            send_command(0,0x08,data_list.byte_data,0);  //需要用标准帧（数据帧）进行发送，不能用远程帧
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

    float factor = 0.01;
        float f_speed = speed;
        if (mode == 1)
        {
            int s16_torque = (int)((param) / factor);
            if( f_speed == 0)
                s16_torque = 0;
            unsigned short u16_input_mode = INPUT_MODE_PASSTHROUGH;
            
            float value_data[3]={f_speed,s16_torque,u16_input_mode};
            int type_data[3]={0,2,1};
            format_data(value_data,type_data,3,"encode");
        }
        else
        {
            int s16_ramp_rate = (int)((param) / factor);
            unsigned short u16_input_mode = INPUT_MODE_VEL_RAMP;
             
            float value_data[3]={f_speed,s16_ramp_rate,u16_input_mode};
            int type_data[3]={0,2,1};
            format_data(value_data,type_data,3,"encode");
        }
        
        send_command(id_num,0x1c,data_list.byte_data,0);
        reply_state(id_num);

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
            unsigned char order_num = 0x13;
            
            float value_data[3]={order_num,0,0};
            int type_data[3]={3,1,1};
            format_data(value_data,type_data,3,"encode");
            
            send_command(0, 0x08, data_list.byte_data, 0); //需要用标准帧（数据帧）进行发送，不能用远程帧
    
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

    float factor = 0.01;
    int u16_input_mode,s16_ramp_rate;
        float f_torque = torque;
        if (mode == 1)
        {
             u16_input_mode = INPUT_MODE_PASSTHROUGH;
             s16_ramp_rate = 0;
        }
        else
        {
             u16_input_mode = INPUT_MODE_TORQUE_RAMP;
             s16_ramp_rate = (int)((param) / factor);
        }
        
        float value_data[3]={f_torque, s16_ramp_rate,u16_input_mode};
        int type_data[3]={0,2,1};
        format_data(value_data,type_data,3,"encode");
        
        send_command(id_num,0x1d,data_list.byte_data,0);
        reply_state(id_num);
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
            unsigned char order_num = 0x14;
            
            float value_data[3]={order_num,0,0};
            int type_data[3]={3,1,1};
            format_data(value_data,type_data,3,"encode");
            
            send_command(0, 0x08, data_list.byte_data, 0);  //需要用标准帧（数据帧）进行发送，不能用远程帧

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

    float factor = 0.01;
    
        preset_angle(id_num,pos,vel, tff, 2);
        unsigned char order_num = 0x15;
        
        float value_data[3]={order_num,(int)(kp / factor),(int)(kd / factor)};
        int type_data[3]={3,2,2};
        format_data(value_data,type_data,3,"encode");
        
        send_command(0, 0x08,data_list.byte_data,0);//需要用标准帧（数据帧）进行发送，不能用远程帧
    
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
    
        unsigned char order_num = 0x06;
        
        float value_data[3]={order_num,0,0};
        int type_data[3]={3,1,1};
        format_data(value_data,type_data,3,"encode");
        
        send_command(id_num,0x08,data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
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

    write_property(id_num,31001,3,new_id);
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

    write_property(id_num,10001,3,baud_rate);
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

    write_property(id_num,21001,3,baud_rate);
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

    if (mode == 1)
        write_property(id_num,30003,3,AXIS_STATE_IDLE);
    else if (mode == 2)
        write_property(id_num,30003,3, AXIS_STATE_CLOSED_LOOP_CONTROL);

}
/**
 * @brief 设置电机零点位置函数
 * 设置当前位置为电机输出轴零点，设置完后当前位置为0度
 * 
 * @param id_num 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 */
void  set_zero_position(uint8_t id_num)
{

        unsigned char order_num = 0x05;
       
        float value_data[3]={order_num,0,0};
        int type_data[3]={3,1,1};
        format_data(value_data,type_data,3,"encode");
        
        send_command(id_num, 0x08, data_list.byte_data, 0); //需要用标准帧（数据帧）进行发送，不能用远程帧
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
        write_property(id_num, 10008, 1, !mode);
        SERVO_DELAY(sleep_time);
     }
    if (enable_step_dir != mode)
    {
        write_property(id_num, 31006, 1, mode);
        SERVO_DELAY(sleep_time);
    }
    if (mode == 0) write_property(id_num, 10001, 3, param);
    else if (mode == 1) write_property(id_num, 38019, 0, param);
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

    float pos_vel = get_state(id_num).angle;
    if (READ_FLAG == 1)
    {
        if (pos_vel < angle_min || pos_vel > angle_max || READ_FLAG != 1) return -1;
            write_property(id_num, 38010, 0, angle_min);
            write_property(id_num, 38011, 0, angle_max);
            write_property(id_num, 31202, 1, 1);
            save_config(id_num);
            return 1;
       
    }    
    else
    {
        return 0;
    }

}
/**
 * @brief设置速度轨迹模式下速度曲线类型
 * 设置梯形轨迹模式，1表示梯形轨迹模式，2表示S形轨迹模式（指的是速度曲线形状，适用于位置控制-梯形轨迹模式）
 *
 * @param id_num: 需要设置的电机编号,如果不知道当前电机编号，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 * @param   mode: 1表示梯形轨迹，2表示S形轨迹（指的是速度曲线形状，适用于位置控制-梯形轨迹模式）
 * @return 无
*/
void set_traj_mode(uint8_t id_num,int mode)
{
    write_property(id_num, 35104,3,mode);
    TRAJ_MODE = mode;
}

/**
 * @brief 修改电机属性参数
 * 修改电机属性参数，这里的属性参数为电机控制参数
 * 
 * @param id_num 需要修改的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 * @param param_address 需要读取的属性参数地址，例如"vbus_voltage"，"axis0.config.can_node_id"等，具体参数名称见enums.py文件里property_address字典里的键值。
 * @param param_type 需要读取的属性参数数据类型。
 * @param value 对应参数的目标值。
 */
void write_property(uint8_t id_num,unsigned short param_address,int8_t param_type,float value)
{

        float value_data[3]={param_address,param_type,value};
        int type_data[3]={1,1,param_type};
        format_data(value_data,type_data,3,"encode");
        
        send_command(id_num, 0x1F,data_list.byte_data,0);  //需要用标准帧（数据帧）进行发送，不能用远程帧
   
}
/**
 * @brief 读取电机ID。
 * 
 * @param id_num 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。
 * @return 电机的 ID 号
 */
uint8_t get_id(uint8_t id_num)
{

    return read_property(id_num,31001,3);

}
/**
 * @brief 读取电机的当前位置和速度
 * 读取电机输出轴当前位置和速度列表，单位分别为度（°）和转每分钟(r/min)
 *
 *同时作为实时状态（实时位置、实时速度、实时扭矩、是否到达目标位置、是否报错）快速读取接口，进行实时控制时可采用该函数进行快速读取电机状态
 *注：1. 但是需要将MOTOR_NUM变量根据最大的电机ID号进行调整，保证MOTOR_NUM大于或等于最大的电机ID号；
      2. enable_reply_state值不影响快速读取接口，只影响发送控制指令时是否实时返回电机状态；

 * @return 电机的位置和速度
 * @param id_num 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。
 * @return struct servo_state 储存电机位置和速度的结构体
 */
struct servo_state get_state(uint8_t id_num)
{
    struct servo_state state = {0, 0};
    float value_data[3]= {0x00,0x00,0};
    int type_data[3]= {1,1,3};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x1E,data_list.byte_data,0);// 需要用标准帧（数据帧）进行发送，不能用远程帧
	READ_FLAG=0;
    receive_data();
    if(id_num<=MOTOR_NUM)//id_num不能为0
    {
			if(READ_FLAG==1)
			{
					if(id_num==0)  //如果ID号为0，则通过返回数据帧的ID信息更新ID号
					{
						id_num = (uint8_t)((can_id & 0x07E0) >> 5)&0xFF;
					}
					float factor = 0.01;
					float value_data[3]= {0,0,0};
					int type_data[3]= {0,2,2};
					format_data(value_data,type_data,3,"decode");
					motor_state[id_num-1][0]=data_list.value_data[0];
					motor_state[id_num-1][1]=data_list.value_data[1]*factor;
					motor_state[id_num-1][2]=data_list.value_data[2]*factor;
					motor_state[id_num-1][3] =(int)((can_id & 0x02) >> 1);
					motor_state[id_num-1][4] = (int)((can_id & 0x04) >> 2);

					state.angle = data_list.value_data[0];
					state.speed = data_list.value_data[1]*factor;
			}
			else
			{
					READ_FLAG=-1;
					reply_state_error++;
			}
    }
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
  struct servo_volcur volcur = {0, 0};
  volcur.vol  = read_property(id_num, 1, 0);
  if(READ_FLAG==1){ 
  volcur.cur  = read_property(id_num, 33206,0);
  }
  else READ_FLAG=-1;
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

    if (enable_uart != NULL) *enable_uart = read_property(id_num,10008,1) == 0 ? 0 : 1;
    if (enable_step_dir != NULL) *enable_step_dir = read_property(id_num,31006,1) == 0 ? 0 : 1;
    if (READ_FLAG == 1)
    {
        if (*enable_uart && !*enable_step_dir)
        {
            if (n != NULL)
                *n = read_property(id_num,10001,3);
            return 1;
        }
        else if (!*enable_uart && *enable_step_dir)
        {
            if (n != NULL)
                *n = read_property(id_num,38019,0);
            return 1;
        }
        return -1;
			}
    return 0;
}
/**
 * @brief 读取电机属性参数
 * 读取电机属性参数，这里的属性参数包括电机状态量及电机控制参数
 * 
 * @param id_num 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。
 * @param param_address  需要读取的属性参数的地址，例如"vbus_voltage"，"axis0.config.can_node_id"等，具体参数名称见enums.py文件里property_address字典里的键值。
 * @param param_type 需要读取的属性参数的数据类型。
 * @return 对应属性参数的值
 */
float read_property(uint8_t id_num,int param_address,int param_type)
{

        float value_data[3]={param_address,param_type,0};
        int type_data[3]={1,1,3};
        format_data(value_data,type_data,3,"encode");
        READ_FLAG=0;
        send_command(id_num,0x1E,data_list.byte_data,0);// 需要用标准帧（数据帧）进行发送，不能用远程帧

        receive_data();
         
        if (READ_FLAG == 1)
        {
		    float value_data[3]={0,0,0};			 
            int type_data[3]={1,1,param_type};
            format_data(value_data,type_data,3,"decode");
           
            float value=data_list.value_data[2];
            return value;
        }
        else
        {
            READ_FLAG=-1;
            return 0;
        }
}
/*
其他系统辅助函数，一般情况下无需使用
*/


/**
 * @brief 清除错误标志函数
 * 一旦电机运行过程中出现任何错误，电机将进入IDLE模式，如果要恢复正常控制模式，需要首先用clear_error清除错误标志后,然后用set_mode函数将模式设置为2（闭环控制模式）。
 * 
 * @param id_num 需要清除错误标志的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 */
void clear_error(uint8_t id_num)
{
    
        unsigned char order_num = 0x04;
        
        float value_data[3]={order_num,0,0};
        int type_data[3]={3,1,1};
        format_data(value_data,type_data,3,"encode");
        
        send_command(id_num,0x08, data_list.byte_data, 0);  //需要用标准帧（数据帧）进行发送，不能用远程帧
}


/**
 * @brief 打印电机错误编号
 * 读取电机错误信息编码，如果错误编码为0，表示无异常。如果错误编码不为0，则表示存在故障。
 * 
 * @param id_num 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。
 * @return 电机抛出的错误信息编号，如果为负数表示在读取对应错误信息时通信失败，未成功读取到数据
 *         0 无任何错误
 *         1 axis_error
 *         2 motor_error
 *         3 controller_error
 *         4 encoder_error
 *         5 can_error
 *         6 fet_thermistor_error
 *         7 motor_thermistor_error
 */
int8_t dump_error(uint8_t id_num)
{
	unsigned int axis_error = read_property(id_num, 30001,3);
	if(READ_FLAG==1&&axis_error!=0)
	    return 1;
	else if(READ_FLAG!=1)
	    return -1;
	unsigned int motor_error = read_property(id_num, 33001,3);
	if(READ_FLAG==1&&motor_error!=0)
	    return 2;
    else if(READ_FLAG!=1)
        return -2;
	unsigned int controller_error = read_property(id_num, 32001,3);
	if(READ_FLAG==1&&controller_error!=0)
	    return 3;
    else if(READ_FLAG!=1)
        return -3;
	unsigned int encoder_error = read_property(id_num, 34001,3);
	if(READ_FLAG==1&&encoder_error!=0)
	    return 4;
    else if(READ_FLAG!=1)
        return -4;
	unsigned int can_error = read_property(id_num, 20001,3);
	if(READ_FLAG==1&&can_error!=0)
	    return 5;
    else if(READ_FLAG!=1)
        return -5;
	unsigned int fet_thermistor_error = read_property(id_num, 36001,3);
	if(READ_FLAG==1&&fet_thermistor_error!=0)
	    return 6;
    else if(READ_FLAG!=1)
        return -6;
	unsigned int motor_thermistor_error = read_property(id_num, 37001,3);
	if(READ_FLAG==1&&motor_thermistor_error!=0)
	    return 7;
    else if(READ_FLAG!=1)
        return -7;
    else
        return 0;
}

/**
 * @brief 保存配置函数
 * 正常情况下，通过write_property修改的属性电机上电重启之后，会恢复为修改前的直，如果想永久保存，则需要用save_config函数将相关参数保存到flash中，掉电不丢失。
 * 
 * @param id_num 需要保存配置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 */
void save_config(uint8_t id_num)
{
    
        unsigned char order_num = 0x01;
        
        float value_data[3]={order_num,0,0};
        int type_data[3]={3,1,1};
        format_data(value_data,type_data,3,"encode");
        
        send_command(id_num,0x08, data_list.byte_data,0);  //需要用标准帧（数据帧）进行发送，不能用远程帧
    

}
/**
 * @brief 电机重启函数
 * 电机软件重启，效果与重新上电类似。
 * 
 * @param id_num 需要重启的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
 */
void reboot(uint8_t id_num)
{
    
        unsigned char order_num = 0x03;
        
        float value_data[3]={order_num,0,0};
        int type_data[3]={3,1,1};
        format_data(value_data,type_data,3,"encode");
        
        send_command(id_num,0x08, data_list.byte_data,0);  // 需要用标准帧（数据帧）进行发送，不能用远程帧
 
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
        traj_done = read_property(id_num, 32008,3);
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