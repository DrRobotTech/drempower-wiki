//例程1：单个电机在正负180度之间来回转动
#include "DrEmpower_uart.h"

extern float motor_state[1][5];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}
uint8_t id_num = 1;
void loop() {  
    // put your main code here, to run repeatedly:
    set_angle(id_num,180,30,60,1);
    position_done(id_num);
    //用实时状态读取接口函数get_state实现position_done功能
    // motor_state[id_num-1][3] = 0;       //发送新指令后，先把是否已到达目标位置置0,
  //   while(motor_state[id_num-1][3]!=1)  //为id_num号关节电机实时状态保存在motor_state[id_num-1]数组中，包含位置、速度、扭矩、是否到达目标位置、是否报错5个状态量
    // {
    //     get_state(id_num);  //采用实时状态快速读取接口函数
    // }
    set_angle(id_num,-180,30,60,1);
    position_done(id_num);
    //用实时状态读取接口函数get_state实现position_done功能
    // motor_state[id_num-1][3] = 0;
    // while(motor_state[id_num-1][3]!=1)
    // {
    //     get_state(id_num);  //采用实时状态快速读取接口函数
    // }
}


/*
//例程2：多个电机在正负360度之间来回转动
#include "DrEmpower.h"

extern float motor_state[1][5];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}
uint8_t id_list[]={1,2,3};  //三个电机的ID号
float angle_list[]={360,360,360}; //三个电机的正转角度
float F_angle_list[]={-360,-360,-360};  //三个电机的反转角度
void loop() {  
    // put your main code here, to run repeatedly:
     set_angles(id_list,angle_list,30,60,1,3);  //多个电机角度控制函数
     positions_done(id_list,3); //多个电机等待函数
//    //用实时状态读取接口函数get_state实现position_done功能
//    for(int i=0;i<3;i++)
//    {
//        uint8_t id_num = id_list[i];
//        motor_state[id_num-1][3] = 0;
//        while(motor_state[id_num-1][3]!=1)
//        {
//            get_state(id_num);  //采用实时状态快速读取接口函数
//        }
//    }
      set_angles(id_list,F_angle_list,30,60,1,3);  //多个电机角度控制函数
      positions_done(id_list,3); //多个电机等待函数
//    //用实时状态读取接口函数get_state实现position_done功能
//    for(int i=0;i<3;i++)
//    {
//        uint8_t id_num = id_list[i];
//        motor_state[id_num-1][3] = 0;
//        while(motor_state[id_num-1][3]!=1)
//        {
//            get_state(id_num);  //采用实时状态快速读取接口函数
//        }
//    }
}
*/
