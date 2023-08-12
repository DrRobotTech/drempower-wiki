#ifndef drempower_CANOPEN_H
#define drempower_CANOPEN_H

#include <ros/ros.h>
#include "can_msgs/Frame.h"
#include "format_data.h"

enum
{
    OP_PP_MODE = 1, // INPUT_MODE_TRAP_TRAJ (Profile Position Mode)
    OP_PV_MODE = 3, // INPUT_MODE_VELOCITY_RAMP (Profile Velocity Mode)
    OP_PT_MODE = 4, // INPUT_MODE_TORQUE_RAMP (Profile Torque  Mode)
    OP_HZ_MODE = 6, // SET_ZERO_POSITION (Homing Mode)
    OP_TP_MODE = 7, // INPUT_MODE_POS_FILTER (Interpolated Position Mode)

    OP_FP_MODE = -1, // INPUT_MODE_FF_CONTROL(feedforward Position Mode)
    OP_FV_MODE = -3, // INPUT_MODE_FF_CONTROL(feedforward  Velocity Mode)
    OP_FT_MODE = -4, // INPUT_MODE_FF_CONTROL(feedforward  Torque  Mode)
    OP_IP_MODE = -7, // INPUT_MODE_IMPEDANCE_CONTROL(Impedance Control)
};

void set_nmt_state(ros::Publisher pub, uint16_t id_num, uint8_t state)
{
    uint8_t nmt_code;
    can_msgs::Frame t_message;
    switch (state)
    {
    case 0x7F:
        nmt_code = 0x80;
        break; //"Pre-operational"
    case 0x05:
        nmt_code = 0x01;
        break; // "Operational"
    case 0x04:
        nmt_code = 0x02;
        break; // "Stopped"
    default:
        break;
    }
    t_message.id = 0x0000;
    t_message.dlc = 8;
    t_message.data[0] = nmt_code;
    t_message.data[1] = id_num;
    pub.publish(t_message);
    ROS_INFO("send nmt");
    ros::Rate rate(100);
    rate.sleep();
}

void send_rpdo(ros::Publisher pub, uint16_t id_num, uint8_t pdo_num, uint8_t *data, uint8_t len)
{
    can_msgs::Frame t_message;
    uint16_t cob_id_list[] = {0x200, 0x300, 0x400, 0x500};
    t_message.id = cob_id_list[pdo_num - 1] + id_num;
    t_message.dlc = 8;
    for (uint8_t i = 0; i < len; i++)
    {
        t_message.data[i] = data[i];
    }
    pub.publish(t_message);
    ROS_INFO("send  rpdo!");
    ros::Rate rate(100);
    rate.sleep();
}

void send_sdo(ros::Publisher pub, uint16_t id_num, uint16_t index, uint8_t subindex, uint8_t *data, uint8_t len)
{
    uint8_t cs_code;
    can_msgs::Frame t_message;
    switch (len)
    {
    case 1:
        cs_code = 0x2F;
        break;
    case 2:
        cs_code = 0x2B;
        break;
    case 3:
        cs_code = 0x27;
        break;
    case 4:
        cs_code = 0x23;
        break;
    default:
        break;
    }
    t_message.id = 0x600 + id_num;
    t_message.dlc = 8;
    t_message.data[0] = cs_code;
    t_message.data[1] = index & 0xFF;
    t_message.data[2] = index >> 8;
    t_message.data[3] = subindex;
    for (uint8_t i = 0; i < len; i++)
        t_message.data[4 + i] = data[i];
    pub.publish(t_message);
    ROS_INFO("send sdo!");
    ros::Rate rate(100);
    rate.sleep();
}

void config_tpdo(ros::Publisher pub, uint16_t id_num, uint8_t pdo_num, uint32_t *map_list, uint8_t len, uint8_t trans_type, uint16_t event_timer)
{
    set_nmt_state(pub, id_num, 0x7F); // 进入预操作模式-//"Pre-operational"
    uint8_t data[] = {uint8_t(id_num + 0x80), pdo_num, 0x00, 0x80};
    send_sdo(pub, id_num, 0x1800 + pdo_num - 1, 0x01, data, 4); // 失能PDO
    data[0] = 0x00;
    send_sdo(pub, id_num, 0x1A00 + pdo_num - 1, 0x00, data, 1); // 清空PDO映射
    for (uint8_t i = 0; i < len; i++)
    {
        for (uint8_t j = 0; j < 4; j++)
        {
            data[j] = (map_list[i] >> (8 * j)) & 0xFF;
        }
        send_sdo(pub, id_num, 0x1A00 + pdo_num - 1, i + 1, data, 4); // 设置PDO映射
    }
    data[0] = trans_type;
    send_sdo(pub, id_num, 0x1800 + pdo_num - 1, 0x02, data, 1); // 设置PDO传输类型
    data[0] = uint8_t(event_timer & 0xFF);
    data[1] = uint8_t(event_timer >> 8);
    send_sdo(pub, id_num, 0x1800 + pdo_num - 1, 0x05, data, 2); // 设置PDO事件定时器触发的时间(单位 ms)
    data[0] = len;
    send_sdo(pub, id_num, 0x1A00 + pdo_num - 1, 0x00, data, 1); // 使能PDO映射
    data[0] = uint8_t(id_num + 0x80);
    data[1] = pdo_num;
    data[2] = 0x00;
    data[3] = 0x00;
    send_sdo(pub, id_num, 0x1800 + pdo_num - 1, 0x01, data, 4); // 使能PDO
    set_nmt_state(pub, id_num, 0x05);                           // 进入操作模式- // "Operational"
}

void config_rpdo(ros::Publisher pub, uint16_t id_num, uint8_t pdo_num, uint32_t *map_list, uint8_t len, uint8_t trans_type)
{
    set_nmt_state(pub, id_num, 0x7F);                               // 进入预操作模式-//"Pre-operational"
    uint8_t data[] = {uint8_t(id_num), pdo_num + 1, 0x00, 0x80};
    send_sdo(pub, id_num, 0x1400 + pdo_num - 1, 0x01, data, 4);     // 失能PDO
    data[0] = 0x00;
    send_sdo(pub, id_num, 0x1600 + pdo_num - 1, 0x00, data, 1);     // 清空PDO映射
    for (uint8_t i = 0; i < len; i++)
    {
        for (uint8_t j = 0; j < 4; j++)
        {
            data[j] = (map_list[i] >> (8 * j)) & 0xFF;
        }
        send_sdo(pub, id_num, 0x1600 + pdo_num - 1, i + 1, data, 4); // 设置PDO映射
    }
    data[0] = trans_type;
    send_sdo(pub, id_num, 0x1400 + pdo_num - 1, 0x02, data, 1);      // 设置PDO传输类型
    data[0] = len;
    send_sdo(pub, id_num, 0x1600 + pdo_num - 1, 0x00, data, 1);      // 使能PDO映射
    data[0] = uint8_t(id_num);
    data[1] = pdo_num + 1;
    data[2] = 0x00;
    data[3] = 0x00;
    send_sdo(pub, id_num, 0x1400 + pdo_num - 1, 0x01, data, 4);      // 使能PDO
    set_nmt_state(pub, id_num, 0x05);                                // 进入操作模式- // "Operational"
}

void set_op_mode(ros::Publisher pub, uint16_t id_num, int8_t mode)
{
    float factor = 0.01;
    uint32_t map_list[3];
    switch (mode)
    {
    case OP_PP_MODE:
    {
        map_list[0] = 0x21420010;
        map_list[1] = 0x21430010;
        config_rpdo(pub, id_num, 2, map_list, 2, 0xFF);
        map_list[0] = 0x21010020;
        map_list[1] = 0x21410010;
        config_rpdo(pub, id_num, 1, map_list, 2, 0x01);
    }
    break;
    case OP_IP_MODE:
    {
        map_list[0] = 0x21310010;
        map_list[1] = 0x21320010;
        config_rpdo(pub, id_num, 3, map_list, 2, 0xFF);
        map_list[0] = 0x21010020;
        map_list[1] = 0x21020010;
        map_list[2] = 0x21030010;
        config_rpdo(pub, id_num, 1, map_list, 3, 0x01);
    }
    break;
    case OP_FP_MODE:
    {
        map_list[0] = 0x21010020;
        map_list[1] = 0x21020010;
        map_list[2] = 0x21030010;
        config_rpdo(pub, id_num, 1, map_list, 3, 0x01);
    }
    break;
    case OP_TP_MODE:
    {
        map_list[0] = 0x21010020;
        map_list[1] = 0x21610010;
        config_rpdo(pub, id_num, 1, map_list, 2, 0x01);
    }
    break;
    case OP_PV_MODE:
    {
        map_list[0] = 0x21020010;
        map_list[1] = 0x21510010;
        config_rpdo(pub, id_num, 1, map_list, 2, 0x01);
    }
    break;
    case OP_FV_MODE:
    {
        map_list[0] = 0x21020010;
        map_list[1] = 0x21030010;
        config_rpdo(pub, id_num, 1, map_list, 2, 0x01);
    }
    break;
    case OP_PT_MODE:
    {
        map_list[0] = 0x21030010;
        map_list[1] = 0x21520010;
        config_rpdo(pub, id_num, 1, map_list, 2, 0x01);
    }
    break;
    case OP_FT_MODE:
    {
        map_list[0] = 0x21030010;
        config_rpdo(pub, id_num, 1, map_list, 1, 0x01);
    }
    break;
    default:
        break;
    }
    map_list[0] = 0x60600008;
    config_rpdo(pub, id_num, 4, map_list, 1, 0xFF);
    if (mode == OP_TP_MODE)
    {
        map_list[0] = 0x21210020;
        map_list[1] = 0x21220010;
        map_list[2] = 0x21230010;
        config_tpdo(pub, id_num, 1, map_list, 3, 0x01, 0x0000); // 配置TPDO1为循环同步模式，每接收到一个SYNC发送一次TPDO1
    }
    else
    {
        map_list[0] = 0x21210020;
        map_list[1] = 0x21220010;
        map_list[2] = 0x21230010;
        config_tpdo(pub, id_num, 1, map_list, 3, 0xFF, 0x000A); // 配置TPDO1为异步模式，每10ms发送一次TPDO1(由于计算时间限制，发送频率最高300HZ左右，这里的event_timer必不大于3)
    }
    uint8_t udata[1];
    int8_to_data(mode, udata);
    send_rpdo(pub, id_num, 4, udata, 1);
}

#endif // drempower_CANOPEN_H