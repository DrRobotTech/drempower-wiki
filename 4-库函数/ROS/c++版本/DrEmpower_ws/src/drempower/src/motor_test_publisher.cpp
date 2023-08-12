#include "ros/ros.h"
#include "drempower/pp_msg.h"
#include "drempower/pv_msg.h"
#include "drempower/pt_msg.h"
#include "drempower/tp_msg.h"
#include "drempower/ip_msg.h"
#include "drempower/fp_msg.h"
#include "drempower/fv_msg.h"
#include "drempower/ft_msg.h"
#include "drempower/property_msg.h"
#include <sstream>

#define MOTOR_NUM 1
#define MOTOR_ID 1 // 根据电机ID号进行修改
#define TP_MODE    // 通过修改这一项测试不同模式

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "motor_test_node");
    ros::NodeHandle nh;
#ifdef PP_MODE
    ros::Publisher pub = nh.advertise<drempower::pp_msg>("pp_mode", 10);
    drempower::pp_msg t_message;
    t_message.id_list.resize(MOTOR_NUM);
    t_message.input_pos_list.resize(MOTOR_NUM);
    t_message.tt_vel_limit_list.resize(MOTOR_NUM);
    t_message.tt_accel_limit_list.resize(MOTOR_NUM);
    t_message.tt_decel_limit_list.resize(MOTOR_NUM);
    t_message.id_list[0] = MOTOR_ID;
    t_message.input_pos_list[0] = 180.0f;
    t_message.tt_vel_limit_list[0] = 100.0f;
    t_message.tt_accel_limit_list[0] = 100.0f;
    t_message.tt_decel_limit_list[0] = 100.0f;
    ros::Rate rate(100);
#endif
#ifdef PV_MODE
    ros::Publisher pub = nh.advertise<drempower::pv_msg>("pv_mode", 10);
    drempower::pv_msg t_message;
    t_message.id_list.resize(MOTOR_NUM);
    t_message.input_vel_list.resize(MOTOR_NUM);
    t_message.vel_ramp_rate_list.resize(MOTOR_NUM);
    t_message.id_list[0] = MOTOR_ID;
    t_message.input_vel_list[0] = 30.0f;
    t_message.vel_ramp_rate_list[0] = 30.0f;
    ros::Rate rate(0.5);
#endif
#ifdef PT_MODE
    ros::Publisher pub = nh.advertise<drempower::pt_msg>("pt_mode", 10);
    drempower::pt_msg t_message;
    t_message.id_list.resize(MOTOR_NUM);
    t_message.input_torque_list.resize(MOTOR_NUM);
    t_message.torque_ramp_rate_list.resize(MOTOR_NUM);
    t_message.id_list[0] = MOTOR_ID;
    t_message.input_torque_list[0] = 5.0f;
    t_message.torque_ramp_rate_list[0] = 5.0f;
    ros::Rate rate(0.5);
#endif
#ifdef TP_MODE
    ros::Publisher pub = nh.advertise<drempower::tp_msg>("tp_mode", 10);
    drempower::tp_msg t_message;
    t_message.id_list.resize(MOTOR_NUM);
    t_message.input_pos_list.resize(MOTOR_NUM);
    t_message.interpolation_freq_list.resize(MOTOR_NUM);
    t_message.id_list[0] = MOTOR_ID;
    t_message.input_pos_list[0] = 0.0f;
    t_message.interpolation_freq_list[0] = 100;
    ros::Rate rate(100); // 轨迹插补模式下这个频率必须与interpolation_freq_list[0]参数保持一致
    uint32_t step = 0;
#endif
#ifdef IP_MODE
    ros::Publisher pub = nh.advertise<drempower::ip_msg>("ip_mode", 10);
    drempower::ip_msg t_message;
    t_message.id_list.resize(MOTOR_NUM);
    t_message.input_pos_list.resize(MOTOR_NUM);
    t_message.input_vel_list.resize(MOTOR_NUM);
    t_message.input_torque_list.resize(MOTOR_NUM);
    t_message.impedance_kp_list.resize(MOTOR_NUM);
    t_message.impedance_kd_list.resize(MOTOR_NUM);
    t_message.id_list[0] = MOTOR_ID;
    t_message.input_pos_list[0] = 180.0f;
    t_message.input_vel_list[0] = 0.0f;
    t_message.input_torque_list[0] = 0.0f;
    t_message.impedance_kp_list[0] = 0.1f;
    t_message.impedance_kd_list[0] = 0.02f;
    ros::Rate rate(0.5);
#endif
#ifdef FP_MODE
    ros::Publisher pub = nh.advertise<drempower::fp_msg>("fp_mode", 10);
    drempower::fp_msg t_message;
    t_message.id_list.resize(MOTOR_NUM);
    t_message.input_pos_list.resize(MOTOR_NUM);
    t_message.input_vel_list.resize(MOTOR_NUM);
    t_message.input_torque_list.resize(MOTOR_NUM);
    t_message.id_list[0] = MOTOR_ID;
    t_message.input_pos_list[0] = 180.0f;
    t_message.input_vel_list[0] = 0.0f;
    t_message.input_torque_list[0] = 0.0f;
    ros::Rate rate(0.5);
#endif
#ifdef FV_MODE
    ros::Publisher pub = nh.advertise<drempower::fv_msg>("fv_mode", 10);
    drempower::fv_msg t_message;
    t_message.id_list.resize(MOTOR_NUM);
    t_message.input_vel_list.resize(MOTOR_NUM);
    t_message.input_torque_list.resize(MOTOR_NUM);
    t_message.id_list[0] = MOTOR_ID;
    t_message.input_vel_list[0] = 30.0f;
    t_message.input_torque_list[0] = 0.0f;
    ros::Rate rate(0.5);
#endif
#ifdef FT_MODE
    ros::Publisher pub = nh.advertise<drempower::ft_msg>("ft_mode", 10);
    drempower::ft_msg t_message;
    t_message.id_list.resize(MOTOR_NUM);
    t_message.input_torque_list.resize(MOTOR_NUM);
    t_message.id_list[0] = MOTOR_ID;
    t_message.input_torque_list[0] = 3.0f;
    ros::Rate rate(0.5);
#endif
    while (ros::ok())
    {
#ifdef PP_MODE
        int32_t traj_done;
        ros::param::get("trajectory_done", traj_done); // 在recvmsg_subsricber_node节点中将实时状态返回的raj_done读取的值保存到变量"trajectory_done"
        if (traj_done == 1)                            // 如果等于1，表示到达目标角度
        {
            pub.publish(t_message);
            ros::param::set("trajectory_done", 0); // 发送新目标角度后，将标志位trajectory_done清0
            t_message.input_pos_list[0] *= -1;
            rate.sleep();                                                      // 延时等待新目标角度生效(生效后电机内的traj_done状态标志被清0)
        }
#else
        pub.publish(t_message);
#endif
#ifdef PV_MODE
        t_message.input_vel_list[0] *= -1;
#endif
#ifdef PT_MODE
        t_message.input_torque_list[0] *= -1;
#endif
#ifdef TP_MODE
        t_message.input_pos_list[0] = 90 * sinf32(step / 100.0f * M_PI - M_PI_2) + 90;
        step++;
#endif
#ifdef IP_MODE
        t_message.input_pos_list[0] *= -1;
#endif
#ifdef FP_MODE
        t_message.input_pos_list[0] *= -1;
#endif
#ifdef FV_MODE
        t_message.input_vel_list[0] *= -1;
#endif
#ifdef FT_MODE
        t_message.input_torque_list[0] *= -1;
#endif
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
