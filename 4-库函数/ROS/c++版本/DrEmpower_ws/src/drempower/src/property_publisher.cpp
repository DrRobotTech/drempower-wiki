#include "ros/ros.h"
#include "drempower/property_msg.h"
#include <sstream>

#define MOTOR_NUM 1
#define MOTOR_ID 1 // 根据电机ID号进行修改
#define TP_MODE    // 通过修改这一项测试不同模式

int main(int argc, char *argv[])
{
        setlocale(LC_ALL, "");
        ROS_INFO("property_publisher_node");
        ros::init(argc, argv, "property_publisher_node");
        ros::NodeHandle nh;
        ros::Publisher read_property_pub = nh.advertise<drempower::property_msg>("read_property", 10);   // 属性读取
        ros::Publisher write_property_pub = nh.advertise<drempower::property_msg>("write_property", 10); // 属性修改
        drempower::property_msg p_message;
        p_message.id = MOTOR_ID;
        ros::Rate rate(100);
        int32_t step = 0;
        // p_message.address = 38013; // 修改axis0.output_shaft.torque_lim属性，用于限制电机最大输出扭矩（对所有控制模式都有效）
        // p_message.data_type = 0;
        // p_message.value= 5;                 //将axis0.output_shaft.torque_lim设置为5，即将电机最大输出扭矩限制在5Nm以内
        // write_property_pub.publish(p_message);
        p_message.address = 31214; // 修改axis0.config.extra_setting.enable_reply_state属性，用于使能实时状态返回功能（对所有控制模式都有效）
        p_message.data_type = 3;
#ifdef TP_MODE
        p_message.value = 1; // 轨迹跟踪(插补)模式下将axis0.config.extra_setting.enable_reply_state设置为1，即使能实时状态返回功能
#else
        p_message.value = 0; // 其他控制模式将axis0.config.extra_setting.enable_reply_state设置为0，即关闭电机实时状态返回功能
#endif
        ros::Rate rate_delay(1); // 延时1s等待CAN接口初始化之后再进行reply_state配置，否则可能会出现配置不成功的情况
        rate_delay.sleep();
        write_property_pub.publish(p_message);
        while (ros::ok())
        {
#ifdef PP_MODE
                // if (step % 10 == 0) // 以更低频率读取axis0.controller.trajectory_done属性
                // {
                //         p_message.address = 32008; // 读取axis0.controller.trajectory_done属性，用于判断是否是否到达目标角度（仅对角度控制模式有效）
                //         p_message.data_type = 3;
                //         read_property_pub.publish(p_message);
                // }
                // p_message.address = 38007; // 读取axis0.output_shaft.pos_estimate属性，用于绘制实时位置曲线
                p_message.address = 0x00; //  最新版本直接使用实时状态快速读取接口（address=0x00)，一条指令同时读取位置、速度、扭矩、是否到达目标位置、是否报错5个状态量
                p_message.data_type = 0;
                read_property_pub.publish(p_message);
                step++;
#endif
#ifdef PV_MODE
                p_message.address = 38008; // 读取axis0.output_shaft.vel_estimate属性，用于绘制实时速度曲线
                p_message.data_type = 0;
                read_property_pub.publish(p_message);
#endif
#ifdef PT_MODE
                p_message.address = 38009; // 读取axis0.output_shaft.torque_estimate属性，用于绘制实时扭矩曲线
                p_message.data_type = 0;
                read_property_pub.publish(p_message);
#endif
#ifdef TP_MODE

#endif
#ifdef IP_MODE
                p_message.address = 38007; // 读取axis0.output_shaft.pos_estimate属性，用于绘制实时位置曲线
                p_message.data_type = 0;
                read_property_pub.publish(p_message);
#endif
#ifdef FP_MODE
                p_message.address = 38007; // 读取axis0.output_shaft.pos_estimate属性，用于绘制实时位置曲线
                p_message.data_type = 0;
                read_property_pub.publish(p_message);
#endif
#ifdef FV_MODE
                p_message.address = 38008; // 读取axis0.output_shaft.vel_estimate属性，用于绘制实时速度曲线
                p_message.data_type = 0;
                read_property_pub.publish(p_message);
#endif
#ifdef FT_MODE
                p_message.address = 38009; // 读取axis0.output_shaft.torque_estimate属性，用于绘制实时扭矩曲线
                p_message.data_type = 0;
                read_property_pub.publish(p_message);
#endif
                rate.sleep();
                ros::spinOnce();
        }
        return 0;
}
