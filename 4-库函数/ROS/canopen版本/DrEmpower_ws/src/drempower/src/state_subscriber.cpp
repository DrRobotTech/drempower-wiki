#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include <sstream>
#include "format_data.h"
#include "drempower/state_msg.h"

ros::Publisher state_msg_pub;

void chatterCallback(const can_msgs::Frame::ConstPtr &msg)
{
     uint16_t cmd, node_id;
     uint8_t data[8];
     node_id = msg->id & 0x7F;
     cmd = msg->id & 0x0780;
     for (uint8_t i = 0; i < 8; i++)
     {
          data[i] = msg->data[i];
     }
     if (cmd == 0x580)
     {
          float value = data_to_float(&data[4]);
          ROS_INFO("SDO received data= %f", value);
     }
     else if (cmd == 0x180)
     {
          drempower::state_msg t_message;
          t_message.id = node_id;
          t_message.pos_estimate = data_to_float(&data[0]);
          t_message.vel_estimate = data_to_int16(&data[4]) * 0.01f;
          t_message.torque_estimate = data_to_int16(&data[6]) * 0.01f;
          state_msg_pub.publish(t_message);
          ROS_INFO("TPDO1 received data: [%d,%f, %f, %f]", t_message.id, t_message.pos_estimate, t_message.vel_estimate, t_message.torque_estimate);
     }
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "state_subscriber_node");
     ros::NodeHandle n;
     ros::Subscriber sub = n.subscribe("received_messages", 1000, chatterCallback);
     state_msg_pub = n.advertise<drempower::state_msg>("motor_state", 10);
     ros::spin();
     return 0;
}
