#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include <sstream>
#include "format_data.h"
#include "drempower/state_msg.h"
#include "drempower/property_msg.h"

ros::Publisher state_msg_pub;
ros::Publisher property_msg_pub;

void chatterCallback(const can_msgs::Frame::ConstPtr &msg)
{
     uint16_t cmd, node_id;
     uint8_t data[8];
     node_id = msg->id >> 5;
     cmd = msg->id & 0x1F;
     for (uint8_t i = 0; i < 8; i++)
     {
          data[i] = msg->data[i];
     }
     // 电机实时状态返回接口
     if (cmd != 0x1E) //  && node_id == 3，如需只接收特定关节电机数据，则将ID号判断加进去即可
     {
          drempower::state_msg t_message;
          t_message.id = node_id;
          t_message.pos_estimate = data_to_float(&data[0]);
          t_message.vel_estimate = data_to_int16(&data[4]) * 0.01f;
          t_message.torque_estimate = data_to_int16(&data[6]) * 0.01f;
          t_message.traj_done =  bool((cmd&0x02)>>1);
          ros::param::set("trajectory_done",  t_message.traj_done);
          t_message.axis_error = bool((cmd&0x04)>>2);
          state_msg_pub.publish(t_message);
          ROS_INFO("motor state received data: [%d,%f, %f, %f,%d,%d]", t_message.id, t_message.pos_estimate, t_message.vel_estimate, t_message.torque_estimate,t_message.traj_done,t_message.axis_error);
     }
     // 参数读取指令接口
     else
     {
          drempower::property_msg t_message;
          t_message.id = node_id;
          t_message.address = data_to_uint16(&data[0]);
          t_message.data_type = data_to_uint16(&data[2]);
          switch (t_message.data_type)
          {
          case 0:
               t_message.value = data_to_float(&data[4]);
               break;
          case 1:
               t_message.value = data_to_uint16(&data[4]);
               break;
          case 2:
               t_message.value = data_to_int16(&data[4]);
               break;
          case 3:
               t_message.value = data_to_uint(&data[4]);
               break;
          case 4:
               t_message.value = data_to_int(&data[4]);
               break;
          default:
               break;
          }
          property_msg_pub.publish(t_message);
          ROS_INFO("motor property received data: [%d,%d, %d, %f]", t_message.id, t_message.address, t_message.data_type, t_message.value);
     }
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "recvmsg_subscriber_node");
     ros::NodeHandle n;
     ros::Subscriber sub = n.subscribe("received_messages", 1000, chatterCallback);
     state_msg_pub = n.advertise<drempower::state_msg>("motor_state", 10);
     property_msg_pub = n.advertise<drempower::property_msg>("motor_property", 10);
     ros::spin();
     return 0;
}
