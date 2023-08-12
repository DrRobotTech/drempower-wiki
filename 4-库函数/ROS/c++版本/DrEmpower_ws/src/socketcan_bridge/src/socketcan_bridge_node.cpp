/*
 * Copyright (c) 2016, Ivor Wanders
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_bridge/socketcan_to_topic.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/xmlrpc_settings.h>
#include <memory>
#include <string>
#include <stdlib.h>
using namespace std;

#define CMD_RESULT_BUF_SIZE 1024

/*
 * cmd：待执行命令
 * result：命令输出结果
 * 函数返回：0 成功；-1 失败；
 */
int ExecuteCMD(const char *cmd, char *result)
{
  int iRet = -1;
  char buf_ps[CMD_RESULT_BUF_SIZE];
  char ps[CMD_RESULT_BUF_SIZE] = {0};
  FILE *ptr;

  strcpy(ps, cmd);

  if ((ptr = popen(ps, "r")) != NULL)
  {
    while (fgets(buf_ps, sizeof(buf_ps), ptr) != NULL)
    {
      strcat(result, buf_ps);
      if (strlen(result) > CMD_RESULT_BUF_SIZE)
      {
        break;
      }
    }
    pclose(ptr);
    ptr = NULL;
    iRet = 0; // 处理成功
  }
  else
  {
    printf("popen %s error\n", ps);
    iRet = -1; // 处理失败
  }

  return iRet;
}

__inline std::string SystemWithResult(const char *cmd)
{
  char cBuf[CMD_RESULT_BUF_SIZE] = {0};
  string sCmdResult;

  ExecuteCMD(cmd, cBuf);
  sCmdResult = string(cBuf); // char * 转换为 string 类型
  printf("CMD Result: \n%s\n", sCmdResult.c_str());

  return sCmdResult;
}

int main(int argc, char *argv[])
{
  // 初始化can设备
  string result_str = SystemWithResult("sudo ls -l /dev/ttyACM*");
  string flag_str = "/dev/ttyACM*";
  string::size_type idx;
  idx = result_str.find(flag_str);
  if (idx == string::npos && result_str.size() != 0)
  {
    printf("找到虚拟串口设备: can_interface  =  /dev/ttyACM*\r\n");
    string com = result_str.substr(result_str.find_last_of(" ") + 1);
    com.resize(com.size() - 1);
    string order = "sudo slcand -o -c -s8 " + com + " can0";
    system(order.c_str());
    system("sudo ifconfig can0 up");
    system("sudo ifconfig can0 txqueuelen 1000");
  }
  else
  {
    printf("未找到虚拟串口设备: can_interface = cando\r\n");
    system("sudo ip link set down can0"); // set up can0
    system("sudo ip link set can0 type can bitrate 1000000");
    system("sudo ip link set up can0"); // set up can0
    system("sudo ip -details link show can0");
  }

  ros::init(argc, argv, "socketcan_bridge_node");

  ros::NodeHandle nh(""), nh_param("~");

  std::string can_device;
  nh_param.param<std::string>("can_device", can_device, "can0");

  can::ThreadedSocketCANInterfaceSharedPtr driver = std::make_shared<can::ThreadedSocketCANInterface>();

  // initialize device at can_device, 0 for no loopback.
  if (!driver->init(can_device, 0, XmlRpcSettings::create(nh_param)))
  {
    ROS_FATAL("Failed to initialize can_device at %s", can_device.c_str());
    return 1;
  }
  else
  {
    ROS_INFO("Successfully connected to %s.", can_device.c_str());
  }

  // initialize the bridge both ways.
  socketcan_bridge::TopicToSocketCAN to_socketcan_bridge(&nh, &nh_param, driver);
  to_socketcan_bridge.setup();

  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, driver);
  to_topic_bridge.setup(nh_param);

  ros::spin();

  driver->shutdown();
  driver.reset();

  ros::waitForShutdown();
}
