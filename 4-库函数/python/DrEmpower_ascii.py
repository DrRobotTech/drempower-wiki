# -*- coding=utf-8 -*-

"""
大然机器人-外转子盘式无刷伺服驱动器库

适用平台：windows或linux平台
库版本号：v2.0
测试主控版本：windows 10 python 3.7
测试人员：唐昭
测试时间：2021.10.27
"""
import time
import serial
from interface_enums import *
import math as cm
import re

uart_baudrate = 115200

# uart = serial.Serial('/dev/ttyAMA0', 115200)  # 在树莓派（raspbian）下控制电机测试，相应的输入连接的串口和波特率
uart = serial.Serial('COM215', 115200)  # 在 windows 下控制电机测试，相应的输入连接的COM口和波特率
# uart = serial.Serial('/dev/ttyUSB0', 115200)  # 在 jetson nano（ubuntu） 下控制电机测试，相应的输入连接的串口和波特率

READ_FLAG = 0  # 读取结果标志位
CHECK_FLAG = 0  # 是否校验标志位
cur_angle_list = []  # 当前角度列表

TRAJ_MODE = 1  # 速度轨迹模式选择控制，1表示梯形轨迹模式，2表示S形轨迹模式（指的是速度曲线形状，适用于位置控制-梯形轨迹模式）

"""
内部辅助函数，用户无需使用
"""


# 串口发送函数
def write_data(text=""):
    global uart
    if uart.inWaiting() > 0:
        uart.read(uart.inWaiting())
    # print(text)
    try:
        result = uart.write(text.encode('utf-8'))  # 写数据
        # print(text.encode('utf-8'))
        return result
    except Exception as e:
        print("---error in write_data--：", e)
        print("重启串口")
        uart.close()
        uart.open()
        result = uart.write(text.encode('utf-8'))  # 写数据
        return result


# 串口接收函数
def read_data(mode=1):
    global READ_FLAG
    global received_buff
    READ_FLAG = -1
    try:
        received_buff = ""
        if mode == 1:
            start = time.perf_counter()
            while uart.inWaiting() == 0 and time.perf_counter() - start < 0.05:  # To do:
                time.sleep(0.001)
            time.sleep(0.001)
            while uart.inWaiting() > 0:
                received_buff = received_buff + chr(ord(uart.read(1)))
            if len(received_buff) > 0 and received_buff[-1] == '\n':
                READ_FLAG = 1
                # print(received_buff)
                return received_buff
            else:
                print("接收的数据有误:" + received_buff)
                READ_FLAG = -1
                return
        else:
            uart.read(uart.inWaiting())
            return
    except Exception as e:
        READ_FLAG = -1
        print("---error in read_data--：", e)
        print("重启串口")
        uart.close()
        uart.open()
        return


# 计算信息校验码
def checksum(message_str):
    try:
        message_str_int = []
        for i in range(0, len(message_str)):
            message_str_int.append(ord(message_str[i]))
        crc_value = message_str_int[0] ^ message_str_int[1]
        for i in range(2, len(message_str_int)):
            crc_value = crc_value ^ message_str_int[i]
    except Exception as e:
        print("---error in checksum---：", e)
        return False
    return '*' + str(crc_value)


# 计算信息校验码是否正确
def check_checksum(checksum_str, object_str):
    try:
        object_str_check_sum = checksum(object_str)
        if checksum_str == object_str_check_sum:
            return True
        else:
            return False
    except Exception as e:
        print("---error in check_checksum---：", e)
        return None


# 发送最后一位必须为换行符,cn判断是否发送校验字符，cn=0,不发送，cn=1发送
def send_command(order="", cn=0):
    try:
        if cn == 0:
            order = order + '\n'
        else:
            str_check = checksum(message_str=order)
            order = order + str_check
            order = order + '\n'
        # print('发送的命令为：' + order)
        write_data(order)
    except Exception as e:
        print("---error in send_command---：", e)
        return False
    return True


# 将接收到的字符串转换成数值（列表）
def to_num(data=""):
    global READ_FLAG
    try:
        if CHECK_FLAG == 1:
            data = data.split('*')[0]
        if not re.search(r'\d', data):
            print(data)
            return
        if '.' in data:
            float_flag = 1
        else:
            float_flag = 0
        num = data.split()
        if len(num) == 1:
            if float_flag == 1:
                return float(num[0])
            else:
                return int(num[0])
        else:
            if float_flag == 1:
                return [float(element) for element in num]
            else:
                return [int(element) for element in num]
    except Exception as e:
        READ_FLAG = -1
        print("---error in to_num---：", e)
        return


# 预设角度
def preset_angle(id_num=1, angle=0, t=0, param=0, mode=0):
    """单个电机角度控制函数。

    控制指定电机编号的电机按照指定的速度转动到指定的角度（绝对角度，相对于电机零点位置）。

    Args:
        id_num: 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
        angle: 电机角度（-360~360）*n，支持大角度转动
        t: mode=0,无作用，直接给0即可; mode=1, 运动时间（s）; mode =2, 前馈速度（r/min)
        param: mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加速度（(r/min)/s）; mode =2, 前馈扭矩（Nm)
        mode: 角度控制模式选择，电机支持三种角度控制模式，
              mode = 0: 多个电机轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
              mode = 1: 多个电机梯形轨迹模式，此时speed用运动时间t（s）表示，param为目标加速度（(r/min)/s）。
              mode = 2: 前馈控制模式，这种模式下的t为前馈速度，param为前馈扭矩。前馈控制在原有PID控制基础上加入速度和扭矩前馈，提高系统的响应特性和减少静态误差。

    Note: 在mode=1,梯形轨迹模式中，speed和accel都需要大于0.如果speed=0会导致电机报motor error 并退出闭环控制模式，所以在这种模式下如果speed=0,会被用0.01代替。
          另外如果这种模式下accel=0，电机以最快速度运动到angle,speed参数不再其作用。

    Returns:
        True: 运行正常
        False: 运行过程中出现异常

    Raises:
        "---error in set_angle---"

    """
    if mode == 0:
        str_command = 'ms'
        str_motor = str(id_num)
        str_space = ' '
        str_angle = str(angle)
        str_time = str(abs(t))
        str_width = str(abs(param))
        str_order = str_command + str_space + str_motor + str_space + str_angle + str_space + str_time \
                    + str_space + str_width + str_space
        send_command(order=str_order, cn=CHECK_FLAG)
    elif mode == 1:
        str_command = 'ms'
        str_motor = str(id_num)
        str_space = ' '
        str_angle = str(angle)
        str_time = str(abs(t))
        str_accel = str(abs(param))
        str_order = str_command + str_space + str_motor + str_space + str_angle + str_space + str_time \
                    + str_space + str_accel + str_space
        send_command(order=str_order, cn=CHECK_FLAG)
    elif mode == 2:
        str_command = 'ms'
        str_motor = str(id_num)
        str_space = ' '
        str_angle = str(angle)
        str_speed = str(t)
        str_torque = str(param)
        str_order = str_command + str_space + str_motor + str_space + str_angle + str_space + str_speed \
                    + str_space + str_torque + str_space
        send_command(order=str_order, cn=CHECK_FLAG)


# 预设速度
def preset_speed(id_num=0, speed=10, param=0, mode=1):
    """单个电机速度预设函数。

    预设指定电机编号的电机的目标速度，之后需要用mv指令启动转动。

    Args:
        id_num: 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
        speed:  目标速度（r/min）
        param:  mode=1, 前馈扭矩（Nm); mode!=1,或目标加速度（(r/min)/s）
        mode:   控制模式选择
                mode=1, 速度前馈控制模式，电机将目标速度直接设为speed
                mode!=1,速度爬升控制模式，电机将按照目标加速度axis0.controller.config_.vel_ramp_rate变化到speed。

    Note:
        在速度爬升模式下，如果目标加速度axis0.controller.config_.vel_ramp_rate设置为0，则电机速度将保持当前值不变。

    Returns:
        True: 运行正常
        False: 运行过程中出现异常

    Raises:
        "---error in set_speed---"

    """
    try:
        str_command = 'ms'
        str_motor = str(id_num)
        str_space = ' '
        str_speed = str(speed)
        if mode == 1:
            str_input_mode = str(INPUT_MODE_PASSTHROUGH)
            if speed == 0:
                param = 0
            str_torque = str(param)
            str_order = str_command + str_space + str_motor + str_space + str_speed + str_space + str_torque \
                        + str_space + str_input_mode + str_space
        else:
            str_input_mode = str(INPUT_MODE_VEL_RAMP)
            str_ramp_rate = str(param)
            str_order = str_command + str_space + str_motor + str_space + str_speed + str_space + str_ramp_rate \
                        + str_space + str_input_mode + str_space
        send_command(order=str_order, cn=CHECK_FLAG)
    except Exception as e:
        print("---error in preset_speed---：", e)
        return False
    return True


# 预设扭矩
def preset_torque(id_num=0, torque=0.1, param=0, mode=1):
    """单个电机力矩预设函数。

    预设指定电机编号的电机目标扭矩（Nm）

    Args:
        id_num: 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
        torque: 电机输出（Nm)
        param: mode=1,改参数无意义；mode!=1,扭矩上升速度axis0.controller.config.torque_ramp_rate（Nm/s）
        mode:   控制模式选择
                mode=1, 扭矩直接控制模式，电机将目标扭矩直接设为torque
                mode!=1,扭矩爬升控制模式，电机将按照扭矩上升速率axis0.controller.config.torque_ramp_rate（Nm/s）变化到torque。

    Note;
        如果电机转速超过您设置的 vel_limit ，电机输出的力矩将会减小。
        可以设置 axis0.controller.config.enable_current_mode_vel_limit = False 来禁止力矩减小。
        另外在扭矩爬升控制模式下，如果点击扭矩上升速率axis0.controller.config.torque_ramp_rate为0，则点击扭矩将在当前值保持不变。

    Returns:
        True: 运行正常
        False: 运行过程中出现异常

    Raises:
        "---error in set_torque---"

    """
    try:
        str_command = 'ms'
        str_motor = str(id_num)
        str_space = ' '
        str_torque = str(torque)
        if mode == 1:
            str_input_mode = str(INPUT_MODE_PASSTHROUGH)
            str_ramp_rate = str(0)  # 扭矩直接控制模式下这个参数没有作用，直接给0即可
        else:
            str_input_mode = str(INPUT_MODE_TORQUE_RAMP)
            str_ramp_rate = str(param)
        str_order = str_command + str_space + str_motor + str_space + str_torque + str_space + str_ramp_rate \
                    + str_space + str_input_mode + str_space
        send_command(order=str_order, cn=CHECK_FLAG)
    except Exception as e:
        print("---error in preset_torque---：", e)
        return False
    return True


"""
功能函数，用户使用
"""

"""
运动控制功能
"""


# 绝对角度控制
def set_angle(id_num=1, angle=0, speed=0, param=0, mode=0):
    """单个电机角度控制函数。

    控制指定电机编号的电机按照指定的速度转动到指定的角度（绝对角度，相对于电机零点位置）。

    Args:
        id_num: 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
        angle: 电机角度（-360~360）*n，支持大角度转动
        speed: 最大速度限制或前馈速度（r/min）
        param: mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加速度（(r/min)/s）; mode =2, 前馈扭矩（Nm)
        mode: 角度控制模式选择，电机支持三种角度控制模式，
              mode = 0: 轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
              mode = 1: 梯形轨迹模式，这种模式下可以指定运动过程中的速度（speed）和启停加速度（accel）。
              mode = 2: 前馈控制模式，这种模式下的speed和torque分别为前馈控制量.前馈控制在原有PID控制基础上加入速度和扭矩前馈，提高系统的响应特性和减少静态误差。

    Note: 在mode=1,梯形轨迹模式中，speed和accel都需要大于0.如果speed=0会导致电机报motor error 并退出闭环控制模式，所以在这种模式下如果speed=0,会被用0.01代替。
          另外如果这种模式下accel=0，电机以最快速度运动到angle,speed参数不再其作用。

    Returns:
        True: 运行正常
        False: 运行过程中出现异常

    Raises:
        "---error in set_angle---"

    """
    try:
        if mode == 0:
            str_command = 't'
            str_motor = str(id_num)
            str_space = ' '
            str_angle = str(angle)
            str_speed = str(abs(speed))
            str_width = str(abs(param))
            str_order = str_command + str_space + str_motor + str_space + str_angle + str_space + str_speed \
                        + str_space + str_width + str_space
            send_command(order=str_order, cn=CHECK_FLAG)
        elif mode == 1:
            if speed > 0 and param > 0:
                str_command = 'q'
                str_motor = str(id_num)
                str_space = ' '
                str_angle = str(angle)
                if speed == 0:
                    print("speed = 0 in INPUT_MODE_TRAP_TRAJ is not allowed and has been change to speed=0.001")
                    speed = 0.001
                str_speed = str(abs(speed))
                str_accel = str(abs(param))
                str_order = str_command + str_space + str_motor + str_space + str_angle + str_space + str_speed \
                            + str_space + str_accel + str_space
                send_command(order=str_order, cn=CHECK_FLAG)
            else:
                print("speed or accel <= 0")
        elif mode == 2:
            str_command = 'p'
            str_motor = str(id_num)
            str_space = ' '
            str_angle = str(angle)
            str_speed = str(speed)
            str_torque = str(param)
            str_order = str_command + str_space + str_motor + str_space + str_angle + str_space + str_speed \
                        + str_space + str_torque + str_space
            send_command(order=str_order, cn=CHECK_FLAG)
    except Exception as e:
        print("---error in set_angle---：", e)
        return False
    return True


# 多个电机绝对角度同步控制
def set_angles(id_list=[1, 2, 3], angle_list=[150.0, 150.0, 150.0], speed=10, param=10, mode=1):
    """多个电机控制函数。

    控制指定电机编号的电机按照指定的速度转动到指定的角度，保证多个电机同时到达目标角度。

    Args:
        id_list: 电机编号组成的列表
        angle_list: 电机角度组成的列表
        speed: 最大的电机转动的速度（r/min）或前馈速度
        param: mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加速度（(r/min)/s）; mode =2, 前馈速度（r/min)
        mode: 角度控制模式选择，电机支持三种角度控制模式，
              mode = 0: 多个电机轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
              mode = 1: 多个电机梯形轨迹模式，此时speed为多个电机中的最快速度（r/min），param为目标加速度（(r/min)/s）。
              mode = 2: 前馈控制模式，这种模式下的speed和torque分别为前馈控制量.前馈控制在原有PID控制基础上加入速度和扭矩前馈，提高系统的响应特性和减少静态误差。

    Returns:
        无

    Raises:
        如果id_list和angle_list长度不一致时会显示Parameter errors in set_angles()!

    """
    global cur_angle_list
    if len(cur_angle_list) != len(angle_list):
        cur_angle_list = []
        for i in range(len(id_list)):
            state = get_state(id_num=id_list[i])
            if READ_FLAG == 1:
                cur_angle_list.append(state[0])
            else:
                cur_angle_list.append(0)
    if len(id_list) == len(angle_list):
        if mode == 0:
            for i in range(len(id_list)):
                preset_angle(id_num=id_list[i], angle=angle_list[i], t=speed, param=param, mode=mode)
            send_command(order="mt " + str(0) + " 0", cn=CHECK_FLAG)
        elif mode == 1:
            if speed > 0 and param > 0:
                for i in range(len(angle_list)):
                    DETA_angle_list = list(
                        map(lambda x: abs(x[0] - x[1]),
                            zip(angle_list, cur_angle_list)))
                delta_angle = max(DETA_angle_list)
                print(delta_angle)
                if TRAJ_MODE == 2:
                    if delta_angle <= (12 * speed * speed / abs(param)):
                        speed = cm.sqrt((abs(param) * delta_angle) / 12)
                        t = 4 * speed / abs(param)
                    else:
                        t = delta_angle / (6 * speed) + 2 * speed / abs(param)
                else:
                    if delta_angle <= (6 * speed * speed / abs(param)):
                        t = 2 * cm.sqrt(delta_angle / (6 * abs(param)))
                    else:
                        t = speed / abs(param) + delta_angle / (6 * speed)
                print('t=' + str(t))
                for i in range(len(id_list)):
                    preset_angle(id_num=id_list[i], angle=angle_list[i], t=t, param=param, mode=mode)
                send_command(order="mq " + str(0) + " 0", cn=CHECK_FLAG)
            else:
                print("speed or accel <= 0")
        elif mode == 2:
            for i in range(len(id_list)):
                preset_angle(id_num=id_list[i], angle=angle_list[i], t=speed, param=param, mode=mode)
            send_command(order="mp " + str(0) + " 0", cn=CHECK_FLAG)
        cur_angle_list = angle_list[:]


# 相对角度控制
def step_angle(id_num=1, angle=0, speed=0, param=0, mode=0):
    """单个电机相对角度控制函数。

    控制指定电机编号的电机按照指定的速度相对转动指定的角度（相对角度，相对于电机当前位置）。

    Args:
        id_num: 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
        angle: 电机相对角度（-360~360）*n，支持大角度转动
        speed: 最大速度限制或前馈速度（r/min）
        param: mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加速度（(r/min)/s）; mode =2, 前馈扭矩（Nm)
        mode: 角度控制模式选择，电机支持三种角度控制模式，
              mode = 0: 轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
              mode = 1: 梯形轨迹模式，这种模式下可以指定运动过程中的速度（speed）和启停加速度（accel）。
              mode = 2: 前馈控制模式，这种模式下的speed和torque分别为前馈控制量.前馈控制在原有PID控制基础上加入速度和扭矩前馈，提高系统的响应特性和减少静态误差。

    Note: 在mode=1,梯形轨迹模式中，speed和accel都需要大于0.如果speed=0会导致电机报motor error 并退出闭环控制模式，所以在这种模式下如果speed=0,会被用0.01代替。
          另外如果这种模式下accel=0，电机以最快速度运动到angle,speed参数不再其作用。

    Returns:
        True: 运行正常
        False: 运行过程中出现异常

    Raises:
        "---error in step_angle---"

    """
    step_angles(id_list=[id_num], angle_list=[angle], speed=speed, param=param, mode=mode)


# 多个电机相对角度同步控制
def step_angles(id_list=[1, 2, 3], angle_list=[150.0, 150.0, 150.0], speed=10, param=10, mode=1):
    """多个电机控制函数。

    控制指定电机编号的电机按照指定的时间先对转动给定角度。

    Args:
        id_list: 电机编号组成的列表
        angle_list: 电机角度组成的列表
        speed: 最大的电机转动的速度（r/min）或前馈速度
        param: mode=0,角度输入滤波带宽（<300），mode=1,启动和停止阶段加速度（(r/min)/s）; mode =2, 前馈速度（r/min)
        mode: 角度控制模式选择，电机支持三种角度控制模式，
              mode = 0: 多个电机轨迹跟踪模式，用于一般绕轴运动，特别适合多个轨迹点输入，角度输入带宽参数需设置为指令发送频率的一半。
              mode = 1: 多个电机梯形轨迹模式，此时speed为多个电机中的最快速度（r/min），param为目标加速度（(r/min)/s）。
              mode = 2: 前馈控制模式，这种模式下的speed和torque分别为前馈控制量.前馈控制在原有PID控制基础上加入速度和扭矩前馈，提高系统的响应特性和减少静态误差。

    Returns:
        无

    Raises:
        如果id_list和angle_list长度不一致时会显示Parameter errors in set_angles()!

    """
    global TRAJ_MODE
    if len(id_list) == len(angle_list):
        if mode == 0:
            for i in range(len(id_list)):
                preset_angle(id_num=id_list[i], angle=angle_list[i], t=speed, param=param, mode=mode)
            send_command(order="mt " + str(0) + " 1", cn=CHECK_FLAG)
        elif mode == 1:
            if speed > 0 and param > 0:
                DETA_angle_list = [abs(x) for x in angle_list]
                delta_angle = max(DETA_angle_list)
                print(delta_angle)
                if TRAJ_MODE == 2:
                    if delta_angle <= (12 * speed * speed / abs(param)):
                        speed = cm.sqrt((abs(param) * delta_angle) / 12)
                        t = 4 * speed / abs(param)
                    else:
                        t = delta_angle / (6 * speed) + 2 * speed / abs(param)
                else:
                    if delta_angle <= (6 * speed * speed / abs(param)):
                        t = 2 * cm.sqrt(delta_angle / (6 * abs(param)))
                    else:
                        t = speed / abs(param) + delta_angle / (6 * speed)
                print('t=' + str(t))
                for i in range(len(id_list)):
                    preset_angle(id_num=id_list[i], angle=angle_list[i], t=t, param=param, mode=mode)
                send_command(order="mq " + str(0) + " 1", cn=CHECK_FLAG)
            else:
                print("speed or accel <= 0")
        elif mode == 2:
            for i in range(len(id_list)):
                preset_angle(id_num=id_list[i], angle=angle_list[i], t=speed, param=param, mode=mode)
            send_command(order="mp " + str(0) + " 1", cn=CHECK_FLAG)


# 速度控制
def set_speed(id_num=0, speed=10, param=0, mode=1):
    """单个电机速度控制函数。

    控制指定电机编号的电机按照指定的速度连续整周转动。

    Args:
        id_num: 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
        speed:  目标速度（r/min）
        param:  mode=1, 前馈扭矩（Nm); mode!=1,或目标加速度（(r/min)/s）
        mode:   控制模式选择
                mode=1, 速度前馈控制模式，电机将目标速度直接设为speed
                mode!=1,速度爬升控制模式，电机将按照目标加速度axis0.controller.config_.vel_ramp_rate变化到speed。

    Note:
        在速度爬升模式下，如果目标加速度axis0.controller.config_.vel_ramp_rate设置为0，则电机速度将保持当前值不变。

    Returns:
        True: 运行正常
        False: 运行过程中出现异常

    Raises:
        "---error in set_speed---"

    """
    try:
        str_command = 'v'
        str_motor = str(id_num)
        str_space = ' '
        str_speed = str(speed)
        if mode == 1:
            if speed == 0:
                param = 0
            str_input_mode = str(INPUT_MODE_PASSTHROUGH)
            str_torque = str(param)
            str_order = str_command + str_space + str_motor + str_space + str_speed + str_space + str_torque \
                        + str_space + str_input_mode + str_space
        else:
            str_input_mode = str(INPUT_MODE_VEL_RAMP)
            str_ramp_rate = str(param)
            str_order = str_command + str_space + str_motor + str_space + str_speed + str_space + str_ramp_rate \
                        + str_space + str_input_mode + str_space
        send_command(order=str_order, cn=CHECK_FLAG)
    except Exception as e:
        print("---error in set_speed---：", e)
        return False
    return True


# 多个电机速度控制
def set_speeds(id_list=[1, 2, 3], speed_list=[10.0, 20.0, 30.0], param=0, mode=1):
    """多个电机速度控制函数。

    控制指定多个电机编号的电机按照指定的速度连续整周转动。

    Args:
        id_list: 电机编号组成的列表
        speed_list: 电机目标速度（r/min）组成的列表
        param:  mode=1, 前馈扭矩（Nm); mode!=1,或目标加速度（(r/min)/s）
        mode:   控制模式选择
                mode=1, 速度前馈控制模式，电机将目标速度直接设为speed
                mode!=1,速度爬升控制模式，电机将按照目标加速度axis0.controller.config_.vel_ramp_rate变化到speed。

    Note:
        在速度爬升模式下，如果目标加速度axis0.controller.config_.vel_ramp_rate设置为0，则电机速度将保持当前值不变。

    Returns:
        True: 运行正常
        False: 运行过程中出现异常

    Raises:
        "---error in set_speeds---"

    """
    try:
        if len(id_list) == len(speed_list):
            for i in range(len(id_list)):
                preset_speed(id_num=id_list[i], speed=speed_list[i], param=param, mode=mode)
            send_command(order="mv " + str(0), cn=CHECK_FLAG)
    except Exception as e:
        print("---error in set_speeds---：", e)
        return False
    return True


# 扭矩控制
def set_torque(id_num=0, torque=0.1, param=0, mode=1):
    """单个电机力矩（电流）闭环控制函数。

    控制指定电机编号的电机输出指定的扭矩（Nm）

    Args:
        id_num: 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
        torque: 电机输出（Nm)
        param: mode=1,改参数无意义；mode!=1,扭矩上升速度axis0.controller.config.torque_ramp_rate（Nm/s）
        mode:   控制模式选择
                mode=1, 扭矩直接控制模式，电机将目标扭矩直接设为torque
                mode!=1,扭矩爬升控制模式，电机将按照扭矩上升速率axis0.controller.config.torque_ramp_rate（Nm/s）变化到torque。

    Note;
        如果电机转速超过您设置的 vel_limit ，电机输出的力矩将会减小。
        可以设置 axis0.controller.config.enable_current_mode_vel_limit = False 来禁止力矩减小。
        另外在扭矩爬升控制模式下，如果点击扭矩上升速率axis0.controller.config.torque_ramp_rate为0，则点击扭矩将在当前值保持不变。

    Returns:
        True: 运行正常
        False: 运行过程中出现异常

    Raises:
        "---error in set_torque---"

    """
    try:
        str_command = 'c'
        str_motor = str(id_num)
        str_space = ' '
        str_torque = str(torque)
        if mode == 1:
            str_input_mode = str(INPUT_MODE_PASSTHROUGH)
            str_ramp_rate = str(0)  # 扭矩直接控制模式下这个参数没有作用，直接给0即可
        else:
            str_input_mode = str(INPUT_MODE_TORQUE_RAMP)
            str_ramp_rate = str(param)
        str_order = str_command + str_space + str_motor + str_space + str_torque + str_space + str_ramp_rate \
                    + str_space + str_input_mode + str_space
        send_command(order=str_order, cn=CHECK_FLAG)
    except Exception as e:
        print("---error in set_torque---：", e)
        return False
    return True


# 多个电机扭矩控制
def set_torques(id_list=[1, 2, 3], torque_list=[3.0, 4.0, 5.0], param=0, mode=1):
    """多个个电机力矩控制函数。

    同时控制多个电机编号的电机目标扭矩（Nm）

    Args:
        id_list: 电机编号组成的列表
        torque_list:    
        param: mode=1,改参数无意义；mode!=1,扭矩上升速度axis0.controller.config.torque_ramp_rate（Nm/s）
        mode:   控制模式选择
                mode=1, 扭矩直接控制模式，电机将目标扭矩直接设为torque
                mode!=1,扭矩爬升控制模式，电机将按照扭矩上升速率axis0.controller.config.torque_ramp_rate（Nm/s）变化到torque。

    Note;
        如果电机转速超过您设置的 vel_limit ，电机输出的力矩将会减小。
        可以设置 axis0.controller.config.enable_current_mode_vel_limit = False 来禁止力矩减小。
        另外在扭矩爬升控制模式下，如果点击扭矩上升速率axis0.controller.config.torque_ramp_rate为0，则点击扭矩将在当前值保持不变。

    Returns:
        True: 运行正常
        False: 运行过程中出现异常

    Raises:
        "---error in set_torque---"

    """
    try:
        if len(id_list) == len(torque_list):
            for i in range(len(id_list)):
                preset_torque(id_num=id_list[i], torque=torque_list[i], param=param, mode=mode)
            send_command(order="mc " + str(0), cn=CHECK_FLAG)
    except Exception as e:
        print("---error in set_torques---：", e)
        return False
    return True


# 阻抗控制
def impedance_control(id_num=0, pos=0, vel=0, tff=0, kp=0, kd=0):
    """单个电机阻抗控制函数。

    对指定电机编号的电机进行阻抗控制。

    Args:
        id_num: 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
        pos: 电机目标角度（度）
        vel: 电机目标速度（r/min）
        tff: 前馈扭矩（Nm)
        kp: 刚度系数(rad/Nm)
        kd: 阻尼系数(rad/s/Nm)

    Note: 阻抗控制为MIT开源方案中的控制模式，其目标输出扭矩计算公式如下：
          torque = kp*( pos – pos_) + t_ff + kd*(vel – vel_)
          其中pos_和vel_分别为输出轴当前实际位置（degree）和当前实际速度（r/min）, kp和kd为刚度系数和阻尼系数，系数比例与MIT等效

    Returns:
        True: 运行正常
        False: 运行过程中出现异常

    Raises:
        "---error in impedance_control---"

    """
    try:
        send_command("d {} {} {} {} {} {}".format(id_num, pos, vel, tff, kp, kd), CHECK_FLAG)
    except Exception as e:
        print("---error in impedance_control--：", e)


# 急停函数
def estop(id_num=0):
    """急停函数

    控制电机紧急停止。电机急停后将切换到IDLE待机模式，电机卸载并生成ERROR_ESTOP_REQUESTED错误标志，不再响应set_angle/speed/torque指令。
    如果要恢复正常控制模式，需要首先用clear_error清除错误标志后,然后用set_mode函数将模式设置为2（闭环控制模式）。

    Args:
        id_num: 需要急停的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。

    Returns:
        无

    Raises:
        无

    """
    send_command(order="st " + str(id_num), cn=CHECK_FLAG)


"""
参数设置功能
"""


# 设置电机ID号
def set_id(id_num=0, new_id=1):
    """设置电机ID号。

    改变电机ID号（掉电保存）

    Args:
        id_num: 需要重新设置编号的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则多个电机会被设置成相同编号
        new_id: 新电机编号，电机ID号范围为1~63

    Returns:
        无

    Raises:
        无

    """
    write_property(id_num=id_num, property='axis0.config.can_node_id', value=new_id)
    save_config(id_num=new_id)


# 设置UART串口波特率
def set_uart_baud_rate(id_num=0, baud_rate=115200):
    """设置电机串口波特率。

    设置UART串口波特率（掉电保存）

    Args:
        id_num: 需要重新设置波特率的电机编号,如果不知道当前电机编号，可以用0广播。
        baud_rate: uart串口波特率，支持9600,19200,57600,115200中任意一种，修改成功后需手动将主控UART波特率也修改为相同值


    Note: 这个串口波特率只对UART总线（TX/RX）接口有效，USB接口中的串口为虚拟串口，波特率不用设置，可以自动适应上位机的波特率。

    Returns:
        无

    Raises:
        无

    """
    write_property(id_num=id_num, property='config.uart_baudrate', value=baud_rate)
    save_config(id_num=id_num)


# 设置CAN波特率
def set_can_baud_rate(id_num=0, baud_rate=500000):
    """设置电机CAN波特率。

    设置CAN波特率（掉电保存）

    Args:
        id_num: 需要重新设置波特率的电机编号,如果不知道当前电机编号，可以用0广播。
        baud_rate: CAN波特率，支持125k,250k,500k,1M中任意一种,修改成功后需手动将主控CAN波特率也修改为相同值。

    Returns:
        无

    Raises:
        无

    """
    write_property(id_num=id_num, property='can.config.baud_rate', value=baud_rate)
    save_config(id_num=id_num)


# 模式设置
def set_mode(id_num=0, mode=2):
    """设置电机模式。

    设置电机进入不同的控制模式。

    Args:
        id_num: 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
        mode: 电机模式编号
              mode = 1: IDLE待机模式，电机将关掉PWM输出，电机卸载
              mode = 2: 闭环控制模式，set_angle， set_speed, set_torque函数必须在闭环控制模式下才能进行控制。（电机上电后的默认模式）

    Note: 模式3和模式4是用来校准电机和编码器参数，出厂前已完成校准，正常情况下不要使用。



    Returns:
        无

    Raises:
        无

    """
    if mode == 1:
        write_property(id_num=id_num, property='axis.requested_state', value=AXIS_STATE_IDLE)
    elif mode == 2:
        write_property(id_num=id_num, property='axis.requested_state', value=AXIS_STATE_CLOSED_LOOP_CONTROL)


# 设置当前位置为零点位置
def set_zero_position(id_num=0):
    """设置电机零点位置函数

    设置当前位置为电机输出轴零点，设置完后当前位置为0度

    Args:
        id_num: 需要设置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。

    Returns:
        无
    Raises:
        无

    """
    send_command(order="sz " + str(id_num), cn=CHECK_FLAG)


def set_GPIO_mode(id_num=0, mode='tx_rx', param=115200):
    """设置GPIO控制接口模式

    设置电机预留的两个引脚模式，支持的模式有UART串口和Step/Dir接口，通过调用该函数进行切换并设置基本参数

    Args:
        id_num: 需要设置的电机编号,如果不知道当前电机编号，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
        mode: 需要选择的模式，'tx_rx'表示选择uart串口模式，'step_dir'表示选择Step/Dir接口控制模式
        param: 当mode='tx_rx'时，param表示串口的波特率，支持9600、19200、57600、115200其中一种；当mode='step_dir'时，param表示电机转一圈对应的脉冲数，支持1-1024(必须是整数)

    Returns:
        无

    Raises:
        无

    """
    print("修改前：")
    enable_state = get_GPIO_mode(id_num=id_num)
    if mode == 'tx_rx':
        if READ_FLAG == -1 or not enable_state[0] == 1:
            write_property(id_num=id_num, property='config.enable_uart', value=1)
            time.sleep(0.1)
        if READ_FLAG == -1 or not enable_state[1] == 0:
            write_property(id_num=id_num, property='axis0.config.enable_step_dir', value=0)
            time.sleep(0.1)
        write_property(id_num=id_num, property='config.uart_baudrate', value=param)
        time.sleep(0.1)
        print("修改后：")
        get_GPIO_mode(id_num=id_num)
        save_config(id_num=id_num)
        time.sleep(0.1)
        reboot(id_num=id_num)
    elif mode == 'step_dir':
        if READ_FLAG == -1 or not enable_state[0] == 0:
            write_property(id_num=id_num, property='config.enable_uart', value=0)
            time.sleep(0.1)
        if READ_FLAG == -1 or not enable_state[1] == 1:
            write_property(id_num=id_num, property='axis0.config.enable_step_dir', value=1)
            time.sleep(0.1)
        write_property(id_num=id_num, property='axis0.output_shaft.steps_per_turn', value=param)
        time.sleep(0.1)
        print("修改后：")
        get_GPIO_mode(id_num=id_num)
        save_config(id_num=id_num)
        time.sleep(0.1)
        reboot(id_num=id_num)


def set_angle_range(id_num=0, angle_min=-360, angle_max=360):
    """设置电机软件限位极限位置

    设置电机预输出轴软件限位极限位置值，设置成功后电机在位置、速度及扭矩控制模式电机输出轴将被限制在[angle_min, angle_max]范围内
    （注意：当前输出轴位置必须在[angle_min, angle_max]范围内，否则将设置失败）

    Args:
        id_num: 需要设置的电机编号,如果不知道当前电机编号，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
        angle_min:软件限位最小角度（改参数与axis0.output_shaft.circular_setpoint_min对应）
        angle_max:软件限位最大角度（改参数与axis0.output_shaft.circular_setpoint_max对应）

    Returns:
        True: 设置成功
        False: 设置失败

    Raises:
        无

    """
    pos_vel = get_state(id_num=id_num)
    if READ_FLAG == 1:
        if pos_vel[0] >= angle_min and pos_vel[0] <= angle_max:
            write_property(id_num=id_num, property='axis0.output_shaft.circular_setpoint_min', value=angle_min)
            write_property(id_num=id_num, property='axis0.output_shaft.circular_setpoint_max', value=angle_max)
            write_property(id_num=id_num, property='axis0.config.extra_setting.enable_circular_setpoint_limit', value=1)
            save_config(id_num=id_num)
            return True
        else:
            print("电机输出轴当前位置不在" + str([angle_min, angle_max]) + "范围内，软件限位范围设置失败！")
            return False
    else:
        print("电机角度读取失败，软件限位范围设置失败！")
        return False


def set_traj_mode(id_num=0, mode=1):
    global TRAJ_MODE
    write_property(id_num=id_num, property='axis0.trap_traj.config.traj_mode', value=mode)
    TRAJ_MODE = mode


# 参数属性写入
def write_property(id_num=0, property="", value=0):
    """修改电机属性参数

    修改电机属性参数，这里的属性参数为电机控制参数

    Args:
        id_num: 需要修改的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。
        property: 需要读取的属性参数名称，例如"vbus_voltage"，"axis0.config.can_node_id"等，具体参数名称见enums.py文件里property_address字典里的键值。
        value: 对应参数的目标值。

    Returns:
        无

    Raises:
        无

    """
    try:
        str_motor = str(id_num)
        str_value = str(value)
        str_space = ' '
        if 'odrv' in property:
            property = property.split('.', 1)[-1]
        if 'axis' in property:
            str_property = property.split('.', 1)[-1]
            str_command = 'w axis'
            str_order = str_command + str_motor + '.' + str_property + str_space + str_value + str_space
        else:
            str_order = 'w ' + property + str_space + str_value + str_space + str_motor + str_space
        send_command(order=str_order, cn=CHECK_FLAG)
    except Exception as e:
        print("---error in write_property---：", e)
        return False


"""
参数回读功能
"""


# 读取电机ID号
def get_id(id_num=0):
    """读取电机ID。

    读取电机DI号。

    Args:
        id_num: 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。

    Returns:
        无

    Raises:
        无

    """
    return read_property(id_num=id_num, property='axis0.config.can_node_id')


# 读取电机当前角度及速度
def get_state(id_num=0):
    """读取电机的当前位置和速度

    读取电机输出轴当前位置和速度列表，单位分别为度（°）和转每分钟(r/min)

    Args:
        id_num: 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。

    Returns:
        [pos, vel]: 位置和速度列表

    Raises:
        无

    """
    try:
        str_command = 'f'
        str_motor = str(id_num)
        str_space = ' '
        str_order = str_command + str_space + str_motor + str_space
        read_data(0)
        send_command(order=str_order, cn=CHECK_FLAG)
        rdata = read_data(1)
        if READ_FLAG == 1:
            pos_vel = to_num(rdata)
            pos_vel[0] = round(pos_vel[0], 2)
            pos_vel[1] = round(pos_vel[1], 1)
            return pos_vel
        else:
            return
    except Exception as e:
        print("---error in get_state---：", e)
        return False


# 读取电机当前电压及电流
def get_volcur(id_num=0):
    """读取电机的当前电压和电流

    读取电机当前电压和q轴电流列表，单位分别为伏（V）和安(A)

    Args:
        id_num: 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。

    Returns:
        [vol, cur]: 电压和电流列表

    Raises:
        无

    """
    try:
        str_command = 'g'
        str_motor = str(id_num)
        str_space = ' '
        str_order = str_command + str_space + str_motor + str_space
        read_data(0)
        send_command(order=str_order, cn=CHECK_FLAG)
        rdata = read_data(1)
        if READ_FLAG == 1:
            vol_cur = to_num(rdata)
            vol_cur[0] = round(vol_cur[0], 1)
            vol_cur[1] = round(vol_cur[1], 2)
            return vol_cur
        else:
            return
    except Exception as e:
        print("---error in get_state---：", e)
        return False


def get_GPIO_mode(id_num=0):
    """读取GPIO控制接口模式

    读取电机预留的两个引脚当前模式及参数，支持的模式有UART串口和Step/Dir接口

    Args:
        id_num: 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。

    Returns:
        无

    Raises:
        无

    """
    enable_uart = read_property(id_num=id_num, property='config.enable_uart')
    enable_step_dir = read_property(id_num=id_num, property='axis0.config.enable_step_dir')
    if READ_FLAG == 1:
        if enable_uart == 1 and enable_step_dir == 0:
            uart_baudrate = read_property(id_num=id_num, property='config.uart_baudrate')
            print("当前模式为UART模式，串口波特率=" + str(uart_baudrate))
        elif enable_uart == 0 and enable_step_dir == 1:
            steps_per_turn = read_property(id_num=id_num, property='axis0.output_shaft.steps_per_turn')
            print("当前模式为Step/Dir模式，脉冲数/每圈=" + str(steps_per_turn))
        else:
            print("GPIO模式配置或读取错误：" + str([enable_uart, enable_step_dir]))
        return [enable_uart, enable_step_dir]


# 参数属性读取
def read_property(id_num=0, property=""):
    """读取电机属性参数

    读取电机属性参数，这里的属性参数包括电机状态量及电机控制参数

    Args:
        id_num: 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。
        property: 需要读取的属性参数名称，例如"vbus_voltage"，"axis0.config.can_node_id"等，具体参数名称见enums.py文件里property_address字典里的键值。

    Returns:
        value: 返回对应属性参数的值

    Raises:
        无

    """
    try:
        str_space = ' '
        str_motor = str(id_num)
        if 'odrv' in property:
            property = property.split('.', 1)[-1]
        if 'axis' in property:
            str_property = property.split('.', 1)[-1]
            str_command = 'r axis'
            str_order = str_command + str_motor + '.' + str_property + str_space
        else:
            str_space = ' '
            str_order = 'r ' + property + str_space + str_motor + str_space
        read_data(0)
        send_command(order=str_order, cn=CHECK_FLAG)
        rdata = read_data(1)
        if READ_FLAG == 1:
            return to_num(rdata)
        else:
            return
    except Exception as e:
        print("---error in read_property---：", e)
        return False


"""
其他系统辅助函数，一般情况下无需使用
"""


# 清除电机错误标志
def clear_error(id_num=0):
    """清除错误标志函数

    一旦电机运行过程中出现任何错误，电机将进入IDLE模式，如果要恢复正常控制模式，需要首先用clear_error清除错误标志后,然后用set_mode函数将模式设置为2（闭环控制模式）。

    Args:
        id_num: 需要清除错误标志的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。

    Returns:
        无

    Raises:
        无

    """
    send_command(order="sc " + str(id_num), cn=CHECK_FLAG)


# 打印电机错误标志
def dump_error(id_num=0):
    """打印电机错误编号

    读取电机错误信息编码，如果错误编码为0，表示无异常。如果错误编码不为0，则表示存在故障。

    Args:
        id_num: 需要读取的电机编号,如果不知道当前电机编号，可以用0广播，但是这时总线上只能连一个电机，否则将报错。

    Returns:
        无

    Raises:
        无

    """
    try:
        str_command = 'e'
        str_motor = str(id_num)
        str_space = ' '
        str_order = str_command + str_space + str_motor + str_space
        read_data(0)
        send_command(order=str_order, cn=CHECK_FLAG)
        rdata = read_data(1)
        # print(rdata)
        if READ_FLAG == 1:
            print(rdata)
        else:
            return
    except Exception as e:
        print("---error in read_error---：", e)
        return False


# 保存电机配置
def save_config(id_num=0):
    """保存配置函数

    正常情况下，通过write_property修改的属性电机上电重启之后，会恢复为修改前的直，如果想永久保存，则需要用save_config函数将相关参数保存到flash中，掉电不丢失。

    Args:
        id_num: 需要保存配置的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。

    Returns:
        无

    Raises:
        无

    """
    try:
        send_command(order="ss " + str(id_num), cn=CHECK_FLAG)
    except Exception as e:
        print("---error in save_config---：", e)
        return False


# 重启
def reboot(id_num=0):
    """电机重启函数

    电机软件重启，效果与重新上电类似。

    Args:
        id_num: 需要重启的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。

    Returns:
        无

    Raises:
        无

    """
    try:
        send_command(order="sr " + str(id_num), cn=CHECK_FLAG)
    except Exception as e:
        print("---error in reboot---：", e)
        return False


# 等待单个电机运动到目标角度
def position_done(id_num=0):
    """单个电机等待函数

    延时等待直到给定电机到达目标位置(只对角度控制指令有效)

    Args:
        id_num: 需要重启的电机ID编号,如果不知道当前电机ID，可以用0广播，如果总线上有多个电机，则多个电机都会执行该操作。

    Returns:
        无

    Raises:
        无

    """
    traj_done = 0
    while traj_done == 0 or traj_done == None:
        traj_done = read_property(id_num, property='axis0.controller.trajectory_done')


# 等待多个电机运动都到目标角度
def positions_done(id_list=[1, 2, 3]):
    """多个电机等待函数

    程序等待（阻塞）直到所有电机都到达目标位置(只对角度控制指令有效)

    Args:
        id_list: 电机编号组成的列表
    Returns:
        无

    """
    for i in range(len(id_list)):
        position_done(id_list[i])
