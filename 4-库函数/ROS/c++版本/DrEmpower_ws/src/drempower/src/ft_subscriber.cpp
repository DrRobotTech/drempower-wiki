#include <ros/ros.h>
#include <drempower/ft_msg.h>
#include "can_msgs/Frame.h"
#include "format_data.h"


ros::Publisher can_msg_pub;

void msg_callback(const drempower::ft_msg &msg)
{
    uint8_t data[8];
    uint16_t cmd, node_id;
    can_msgs::Frame t_message;
    t_message.dlc = 8;
    for (uint8_t i = 0; i < msg.id_list.size(); i++)
    {
        ROS_INFO("ft_node:[%d,%f]", msg.id_list[i], msg.input_torque_list[i]);
        float_to_data(msg.input_torque_list[i], &data[0]);
        int16_to_data(int16_t(0x00 / 0.01f), &data[4]);
        int16_to_data(int16_t(0x01 / 0.01f), &data[6]); // 直接控制模式mode = 0x01
        // t_message.id = (msg.id_list[i] << 5) + 0x1D; // 单个电机控制指令（单条指令，无需再发送起始指令）
        t_message.id = (msg.id_list[i] << 5) + 0x0C; //  多个电机角度预设指令
        for (uint8_t j = 0; j < 8; j++)
            t_message.data[j] = data[j];
        can_msg_pub.publish(t_message);
    }
    // 起始指令
    uint_to_data(0x14, &data[0]);
    int16_to_data(int16_t(0x00 / 0.01f), &data[4]);
    int16_to_data(int16_t(0x00 / 0.01f), &data[6]);
    t_message.id = (0x00 << 5) + 0x08; // 起始指令默认通过广播ID-0进行发送
    for (uint8_t j = 0; j < 8; j++)
        t_message.data[j] = data[j];
    can_msg_pub.publish(t_message);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ft_subscriber_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("ft_mode", 10, msg_callback);
    can_msg_pub = n.advertise<can_msgs::Frame>("sent_messages", 10);
    ros::Rate rate(1);
    rate.sleep(); // 延时等待CAN通信初始化
    ros::spin();
    return 0;
}
