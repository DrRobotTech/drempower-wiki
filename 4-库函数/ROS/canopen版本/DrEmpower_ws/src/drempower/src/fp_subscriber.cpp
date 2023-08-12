#include <ros/ros.h>
#include <drempower/fp_msg.h>
#include "can_msgs/Frame.h"
#include "format_data.h"
#include "canopen.h"

ros::Publisher can_msg_pub;

void msg_callback(const drempower::fp_msg &msg)
{
    ROS_INFO("fp_node:[%d,%f,%f,%f]", msg.id_list[0], msg.input_pos_list[0], msg.input_vel_list[0], msg.input_torque_list[0]);
    uint8_t data[8];
    uint16_t cmd, node_id;
    can_msgs::Frame t_message;
    t_message.dlc = 8;
    for (uint8_t i = 0; i < msg.id_list.size(); i++)
    {
        float_to_data(msg.input_pos_list[i], &data[0]);
        int16_to_data(int16_t(msg.input_vel_list[i] / 0.01f), &data[4]);
        int16_to_data(int16_t(msg.input_torque_list[i] / 0.01f), &data[6]);
        t_message.id = 0x200 + msg.id_list[i];
        for (uint8_t j = 0; j < 8; j++)
            t_message.data[j] = data[j];
        can_msg_pub.publish(t_message);
    }
    t_message.id = 0x080;
    can_msg_pub.publish(t_message);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fp_subscriber_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("fp_mode", 10, msg_callback);
    can_msg_pub = n.advertise<can_msgs::Frame>("sent_messages", 10);
    ros::Rate rate(1);
    rate.sleep(); // 延时等待CAN通信初始化
    set_op_mode(can_msg_pub, 1, OP_FP_MODE);
    ros::spin();
    return 0;
}
