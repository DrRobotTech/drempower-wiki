#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include <sstream>
#include "format_data.h"
#include "drempower/property_msg.h"

ros::Publisher can_msg_pub;
ros::Publisher property_msg_pub1;
ros::Publisher property_msg_pub2;
ros::Publisher property_msg_pub3;

void ReadProperyCallback(const drempower::property_msg &msg)
{
     uint8_t data[8];
     uint16_t cmd, node_id;
     can_msgs::Frame t_message;
     t_message.dlc = 8;
     ROS_INFO("read_property_node:[%d,%d,%d]", msg.id, msg.address, msg.data_type);
     uint16_to_data(msg.address, &data[0]);
     uint16_to_data(msg.data_type, &data[2]);
     t_message.id = (msg.id << 5) + 0x1E;
     for (uint8_t j = 0; j < 8; j++)
          t_message.data[j] = data[j];
     can_msg_pub.publish(t_message);
}

void WriteProperyCallback(const drempower::property_msg &msg)
{
     uint8_t data[8];
     uint16_t cmd, node_id;
     can_msgs::Frame t_message;
     t_message.dlc = 8;
     ROS_INFO("write_property_node:[%d,%d,%d,%f]", msg.id, msg.address, msg.data_type, msg.value);
     uint16_to_data(msg.address, &data[0]);
     uint16_to_data(msg.data_type, &data[2]);
     switch (msg.data_type)
     {
     case 0:
          float_to_data(msg.value, &data[4]);
          break;
     case 1:
          uint16_to_data(uint16_t(msg.value), &data[4]);
          break;
     case 2:
          int16_to_data(int16_t(msg.value), &data[4]);
          break;
     case 3:
          uint_to_data(uint32_t(msg.value), &data[4]);
          break;
     case 4:
          int_to_data(int32_t(msg.value), &data[4]);
          break;
     default:
          break;
     }
     t_message.id = (msg.id << 5) + 0x1F;
     for (uint8_t j = 0; j < 8; j++)
          t_message.data[j] = data[j];
     can_msg_pub.publish(t_message);
}

void ResolveProperyCallback(const drempower::property_msg &msg)
{
     // ROS_INFO("resolve_property_node:[%d,%d,%d,%f]", msg.id, msg.address, msg.data_type, msg.value);
     if (msg.address == 32008) //  && node_id == 3，如需只接收特定关节电机数据，则将ID号判断加进去即可
     {
          ros::param::set("trajectory_done", msg.value);
     }
     else if (msg.address == 38007) //  && node_id == 3，如需只接收特定关节电机数据，则将ID号判断加进去即可
     {
          property_msg_pub1.publish(msg); // 这里直接转发的作用是对消息"motor_property"进行一次筛选，将消息名称改为具体的"pos_estimate"，便于后续使用（比如使用rqt_plot绘图）
     }
     else if (msg.address == 38008) //  && node_id == 3，如需只接收特定关节电机数据，则将ID号判断加进去即可
     {
          property_msg_pub2.publish(msg); // 这里直接转发的作用是对消息"motor_property"进行一次筛选，将消息名称改为具体的"vel_estimate"，便于后续使用（比如使用rqt_plot绘图）
     }
     else if (msg.address == 38009) //  && node_id == 3，如需只接收特定关节电机数据，则将ID号判断加进去即可
     {
          property_msg_pub3.publish(msg); // 这里直接转发的作用是对消息"motor_property"进行一次筛选，将消息名称改为具体的"torque_estimate"，便于后续使用（比如使用rqt_plot绘图）
     }
}

int main(int argc, char **argv)
{
     ROS_INFO("property_subscriber_node");
     ros::init(argc, argv, "property_subscriber_node");
     ros::NodeHandle n;
     ros::Subscriber read_sub = n.subscribe("read_property", 1000, ReadProperyCallback);
     ros::Subscriber write_sub = n.subscribe("write_property", 1000, WriteProperyCallback);
     ros::Subscriber resolve_sub = n.subscribe("motor_property", 1000, ResolveProperyCallback);
     can_msg_pub = n.advertise<can_msgs::Frame>("sent_messages", 10);
     property_msg_pub1 = n.advertise<drempower::property_msg>("pos_estimate", 10);
     property_msg_pub2 = n.advertise<drempower::property_msg>("vel_estimate", 10);
     property_msg_pub3 = n.advertise<drempower::property_msg>("torque_estimate", 10);
     ros::Rate rate(1);
     rate.sleep(); // 延时等待CAN通信初始化
     ros::spin();
     return 0;
}
