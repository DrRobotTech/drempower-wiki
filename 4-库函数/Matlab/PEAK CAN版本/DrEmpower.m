classdef DrEmpower < handle
    properties %成员属性块--开始
        id
        baudrate
        READ_FLAG
        MOTOR_NUM
        id_list
        cur_angle_list
        enable_reply_state
        motor_state
        TRAJ_MODE
        dev
        can_id  %用于保存电机返回数据帧CAN ID
        reply_state_error
    end %成员属性块--结束
    
    methods %成员函数块--开始
        % 构造函数（初始化CAN接口）
        function obj = DrEmpower(id_list,can_baudrate)  %%Point2D类的构造函数
            obj.id = 1;
            obj.baudrate=can_baudrate;
            obj.READ_FLAG = 1;
            obj.id_list = id_list;
            obj.MOTOR_NUM = 16;
            obj.cur_angle_list = [];
            obj.motor_state = zeros(obj.MOTOR_NUM,5);
            obj.enable_reply_state = 0;
            obj.TRAJ_MODE = 1;
            obj.can_id = 0;
            obj.reply_state_error = 0;
            obj.dev = canChannel('PEAK-System','PCAN_USBBUS1');
            stop(obj.dev);
            configBusSpeed(obj.dev,can_baudrate);
            start(obj.dev);
        end
        % CAN发送函数
        function [] = send_command(obj,id_num, cmd, data, rtr)
            if (obj.enable_reply_state || rtr)&& obj.dev.MessagesAvailable>0
                receive(obj.dev,obj.dev.MessagesAvailable)
            end
            can_id = bitshift(id_num ,5) + cmd;
            msg =  canMessage(can_id,false,8);
            msg.Data = data;
            %            for i = 1:length(data)
            %                pack(msg,data(i),(i-1)*8,8,'LittleEndian');
            %            end
            transmit(obj.dev,msg);
        end
        % CAN接收函数
        function rdata = receive_data(obj)
            tic
            obj.READ_FLAG = -1;
            while obj.dev.MessagesAvailable==0 && toc<0.05  % 目前测试大概在2ms以内
            end
            if obj.dev.MessagesAvailable==0
                toc
            end
            rmsg = receive(obj.dev,1);
            if isempty(rmsg)
                rdata = [];
            else
                obj.READ_FLAG = 1;
                obj.can_id = rmsg.can_id;
                rdata = rmsg.Data;
            end
        end
        
        % 返回电机状态 -- 过程辅助函数
        function reply_state(obj,id_num)
            if obj.enable_reply_state &&id_num <= obj.MOTOR_NUM
                obj.READ_FLAG = 0;
                cdata = obj.receive_data();
                if obj.READ_FLAG == 1
                    if id_num == 0
                        id_num = bitand(bitshift(obj.can_id,-5),0x3F);
                    end
                    factor = 0.01;
                    data = format_data(cdata, 'f s16 s16', 'decode');
                    obj.motor_state(round(id_num),1) = data(1);
                    obj.motor_state(round(id_num),2) = data(2) * factor;
                    obj.motor_state(round(id_num),3) = data(3) * factor;
                    obj.motor_state(round(id_num),4) =round(bitshift(bitand(obj.can_id,0x02),-1));
                    obj.motor_state(round(id_num),5) = round(bitshift(bitand(obj.can_id,0x04),-2));
                else
                    obj.READ_FLAG = -1;
                    obj.reply_state_error = obj.reply_state_error + 1;
                end
            end
        end
        % 预设角度 -- 过程辅助函数
        function preset_angle(obj,id_num, angle, t, param, mode)
            factor = 0.01;
            if mode == 0
                f_angle = angle;
                s16_time = single(abs(t) / factor);
                if param > 300
                    param = 300;
                end
                s16_width = single(abs(param / factor));
                data = format_data([f_angle, s16_time, s16_width], 'f s16 s16', 'encode');
            elseif mode == 1
                f_angle = angle;
                s16_time = single(abs(t) / factor);
                s16_accel = single((abs(param)) / factor);
                data = format_data([f_angle, s16_time, s16_accel], 'f s16 s16', 'encode');
            elseif mode == 2
                f_angle = angle;
                s16_speed_ff = single((t) / factor);
                s16_torque_ff = single((param) / factor);
                data = format_data([f_angle, s16_speed_ff, s16_torque_ff], 'f s16 s16', 'encode');
            end
            obj.send_command(id_num, 0x0C, data, 0);
            obj.reply_state(id_num);
        end
        % 预设速度 -- 过程辅助函数
        function preset_speed(obj,id_num, speed, param, mode)
            INPUT_MODE_PASSTHROUGH = 1;
            INPUT_MODE_VEL_RAMP = 2;
            factor = 0.01;
            f_speed = speed;
            if mode == 1
                s16_torque = single((param) / factor);
                if f_speed == 0
                    s16_torque = 0;
                end
                s16_input_mode = single(INPUT_MODE_PASSTHROUGH / factor);
                data = format_data([f_speed, s16_torque, s16_input_mode], 'f s16 s16', 'encode');
            else
                s16_ramp_rate = single((param) / factor);
                s16_input_mode = single(INPUT_MODE_VEL_RAMP / factor);
                data = format_data([f_speed, s16_ramp_rate, s16_input_mode], 'f s16 s16', 'encode');
            end
            obj.send_command(id_num, 0x0C, data, 0);
            obj.reply_state(id_num);
        end
        % 预设扭矩  -- 过程辅助函数
        function preset_torque(obj,id_num, torque, param, mode)
            INPUT_MODE_PASSTHROUGH = 1;
            INPUT_MODE_TORQUE_RAMP = 6;
            factor = 0.01;
            f_torque = torque;
            if mode == 1
                s16_input_mode = single(INPUT_MODE_PASSTHROUGH / factor);
                s16_ramp_rate = 0;
            else
                s16_input_mode = single(INPUT_MODE_TORQUE_RAMP / factor);
                s16_ramp_rate = single((param) / factor);
            end
            data = format_data([f_torque, s16_ramp_rate, s16_input_mode], 'f s16 s16', 'encode');
            obj.send_command(id_num, 0x0C, data, 0);
            obj.reply_state(id_num);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%用户函数%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %单个电机绝对角度控制
        function set_angle(obj,id_num, angle, speed, param, mode)
            factor = 0.01;
            if mode == 0
                f_angle = angle;
                s16_speed = single((abs(speed)) / factor);
                if param > 300
                    param = 300;
                end
                s16_width = single(abs(param / factor));
                data = format_data([f_angle, s16_speed, s16_width], 'f s16 s16', 'encode');
                obj.send_command(id_num, 0x19, data, 0);
            elseif mode == 1
                if speed > 0 && param > 0
                    f_angle = single(angle);
                    s16_speed = single((abs(speed)) / factor);
                    s16_accel = single((abs(param)) / factor);
                    data = format_data([f_angle, s16_speed, s16_accel], 'f s16 s16', 'encode');
                    obj.send_command(id_num, 0x1A, data, 0);
                end
            elseif mode == 2
                f_angle = angle;
                s16_speed_ff = single((speed) / factor);
                s16_torque_ff = single((param) / factor);
                data = format_data([f_angle, s16_speed_ff, s16_torque_ff], 'f s16 s16', 'encode');
                obj.send_command(id_num, 0x1B, data, 0);
            end
            obj.reply_state(id_num);
        end
        %多个电机绝对角度控制
        function set_angles(obj,id_list, angle_list, speed, param, mode)
            if length(obj.cur_angle_list) ~= length(angle_list)
                obj.cur_angle_list = [];
                for i = 1:length(id_list)
                    angle = obj.read_property(id_list(i),38007,0);
                    if obj.READ_FLAG == 1
                        obj.cur_angle_list = [obj.cur_angle_list,angle];
                    else
                        obj.cur_angle_list = [obj.cur_angle_list,0];
                    end
                end
            end
            if length(id_list) == length(angle_list)
                if mode == 0
                    for i = 1:length(id_list)
                        obj.preset_angle(id_list(i), angle_list(i), speed, param, mode);
                    end
                    order_num = 0x10;
                    data = format_data([order_num, 0], 'u32 u16', 'encode');
                    obj.send_command(0 , 0x08, data, 0)
                elseif mode == 1
                    if speed > 0 && param > 0
                        DELTA_angle_list = [];
                        for i = 1:length(angle_list)
                            DELTA_angle_list = [DELTA_angle_list,abs(angle_list(i)-obj.cur_angle_list(i))];
                        end
                        delta_angle = max(DELTA_angle_list);
                        if obj.TRAJ_MODE == 2
                            if delta_angle <= (12 * speed * speed / abs(param))
                                speed = sqrt((abs(param) * delta_angle) / 12);
                                t = 4 * speed / abs(param);
                            else
                                t = delta_angle / (6 * speed) + 2 * speed / abs(param);
                            end
                        else
                            if delta_angle <= (6 * speed * speed / abs(param))
                                t = 2 * sqrt(delta_angle / (6 * abs(param)));
                            else
                                t = speed / abs(param) + delta_angle / (6 * speed);
                            end
                        end
                        for i = 1:length(id_list)
                            obj.preset_angle(id_list(i), angle_list(i), t, param, mode);
                        end
                        order_num = 0x11;
                        data = format_data([order_num, 0], 'u32 u16', 'encode');
                        obj.send_command(0 , 0x08, data, 0);
                    end
                elseif mode == 2
                    for i = 1:length(id_list)
                        obj.preset_angle(id_list(i), angle_list(i), speed, param, mode);
                    end
                    order_num = 0x12;
                    data = format_data([order_num, 0], 'u32 u16', 'encode');
                    obj.send_command(0 , 0x08, data, 0) ;
                end
                obj.cur_angle_list = angle_list;
            end
        end
        %单个电机相对角度控制
        function step_angle(obj,id_num, angle, speed, param, mode)
            obj.step_angles([id_num], [angle], speed, param,mode);
        end
        %多个电机相对角度同步控制
        function step_angles(obj,id_list, angle_list, speed, param, mode)
            if length(id_list) == length(angle_list)
                if mode == 0
                    for i = 1:length(id_list)
                        obj.preset_angle(id_list(i), angle_list(i), speed, param, mode);
                    end
                    order_num = 0x10;
                    data = format_data([order_num, 1], 'u32 u16', 'encode');
                    obj.send_command(0 , 0x08, data, 0);
                elseif mode == 1
                    if speed > 0 && param > 0
                        DETA_angle_list = abs(angle_list);
                        delta_angle = max(DETA_angle_list);
                        if obj.TRAJ_MODE == 2
                            if delta_angle <= (12 * speed * speed / abs(param))
                                speed = sqrt((abs(param) * delta_angle) / 12);
                                t = 4 * speed / abs(param);
                            else
                                t = delta_angle / (6 * speed) + 2 * speed / abs(param);
                            end
                        else
                            if delta_angle <= (6 * speed * speed / abs(param))
                                t = 2 * sqrt(delta_angle / (6 * abs(param)));
                            else
                                t = speed / abs(param) + delta_angle / (6 * speed);
                            end
                        end
                        for i = 1:length(id_list)
                            obj.preset_angle(id_list(i), angle_list(i), t, param, mode)
                        end
                        order_num = 0x11;
                        data = format_data([order_num, 1], 'u32 u16', 'encode');
                        obj.send_command(0 , 0x08, data, 0)  ;
                    end
                elseif mode == 2
                    for i = 1:length(id_list)
                        obj.preset_angle(id_list(i), angle_list(i), speed, param, mode);
                    end
                    order_num = 0x12;
                    data = format_data([order_num, 1], 'u32 u16', 'encode');
                    obj.send_command(0 , 0x08, data, 0);
                end
            end
        end
        %单个电机速度控制
        function set_speed(obj,id_num, speed, param, mode)
            INPUT_MODE_PASSTHROUGH = 1;
            INPUT_MODE_VEL_RAMP = 2;
            factor = 0.01;
            f_speed = speed;
            if mode == 1
                s16_torque = int32(param / factor);
                if f_speed == 0
                    s16_torque = 0;
                end
                u16_input_mode = INPUT_MODE_PASSTHROUGH;
                data = format_data([f_speed, s16_torque, u16_input_mode], 'f s16 u16', 'encode');
            else
                s16_ramp_rate = int32((param) / factor);
                u16_input_mode = INPUT_MODE_VEL_RAMP;
                data = format_data([f_speed, s16_ramp_rate, u16_input_mode], 'f s16 u16', 'encode');
            end
            obj.send_command(id_num , 0x1C, data, 0);
            obj.reply_state(id_num)
        end
        %多个电机速度控制
        function set_speeds(obj,id_list, speed_list, param, mode)
            if length(id_list) == length(speed_list)
                for i = 1:length(id_list)
                    obj.preset_speed(id_list(i), speed_list(i), param, mode);
                end
                order_num = 0x13;
                data = format_data([order_num], 'u32', 'encode');
                obj.send_command(0 , 0x08, data, 0);
            end
        end
        %单个电机扭矩控制
        function set_torque(obj, id_num, torque, param, mode)
            INPUT_MODE_PASSTHROUGH = 1;
            INPUT_MODE_TORQUE_RAMP = 6;
            factor = 0.01;
            f_torque = torque;
            if mode == 1
                u16_input_mode = INPUT_MODE_PASSTHROUGH;
                s16_ramp_rate = 0;
            else
                u16_input_mode = INPUT_MODE_TORQUE_RAMP;
                s16_ramp_rate = int32((param) / factor);
            end
            data = format_data([f_torque, s16_ramp_rate, u16_input_mode], 'f s16 u16', 'encode');
            obj.send_command(id_num , 0x1D, data, 0)
            obj.reply_state(id_num)
        end
        %多个电机扭矩控制
        function set_torques(obj, id_list, torque_list, param, mode)
            if length(id_list) == length(torque_list)
                for i = 1:length(id_list)
                    obj.preset_torque(id_list(i), torque_list(i), param, mode);
                end
                order_num = 0x14;
                data = format_data([order_num], 'u32', 'encode');
                obj.send_command(0 , 0x08, data, 0);
            end
        end
        %单个电机阻抗控制
        function impedance_control(obj,id_num, pos, vel, tff, kp, kd)
            factor = 0.01;
            obj.preset_angle(id_num, pos, vel, tff, 2);
            order_num = 0x15;
            data = format_data([order_num, int32(kp / factor), int32(kd / factor)], 'u32 s16 s16', 'encode');
            obj.send_command(0 , 0x08, data, 0);
        end
        %急停
        function estop(obj,id_num)
            order_num = 0x06;
            data = format_data([order_num], 'u32', 'encode');
            obj.send_command(id_num , 0x08, data, 0);
        end
        %修改电机ID号
        function set_id(obj, id_num, new_id)
            obj.write_property(id_num,31001,3,new_id);
            obj.save_config(new_id);
        end
        %设置UART串口波特率
        function set_uart_baud_rate(obj, id_num, baud_rate)
            obj.write_property(id_num, 10001, 3, baud_rate);
            obj.save_config(id_num);
        end
        %设置CAN波特率
        function set_can_baud_rate(obj,id_num, baud_rate)
            obj.write_property(id_num, 21001, 3,baud_rate);
            stop(obj.dev);
            configBusSpeed(obj.dev,baud_rate);
            start(obj.dev);
            obj.save_config(id_num);
            obj.baudrate = baud_rate;
        end
        %设置电机模式
        function set_mode(obj,id_num, mode)
            AXIS_STATE_IDLE = 1;
            AXIS_STATE_CLOSED_LOOP_CONTROL = 8;
            if mode == 1
                obj.write_property(id_num,30003,3,AXIS_STATE_IDLE);
            elseif mode == 2
                obj.write_property(id_num,30003,3, AXIS_STATE_CLOSED_LOOP_CONTROL);
            end
        end
        %设置电机零点
        function set_zero_position(obj, id_num)
            order_num = 0x05;
            data = format_data([order_num], 'u32', 'encode');
            obj.send_command(id_num , 0x08, data, 0);
        end
        %配置GPIO模式及参数
        function set_GPIO_mode(obj, id_num, mode, param)
            enable_state = obj.get_GPIO_mode(id_num);
            if strcmp(mode,'tx_rx')==1
                if obj.READ_FLAG == -1 || enable_state(1) ~= 1
                    obj.write_property(id_num, 10008,3, 1)
                end
                if obj.READ_FLAG == -1 || enable_state(2) ~= 0
                    obj.write_property(id_num, 31006,3, 0);
                end
                obj.write_property(id_num, 10001,3, param);
                obj.get_GPIO_mode(id_num);
                obj.save_config(id_num);
                obj.reboot(id_num);
            elseif strcmp(mode,'step_dir')==1
                if obj.READ_FLAG == -1 || enable_state(1) ~= 0
                    obj.write_property(id_num,10008,3, 0);
                end
                if obj.READ_FLAG == -1 || enable_state(2) ~= 1
                    obj.write_property(id_num, 31006,3, 1);
                end
                obj.write_property(id_num, 38019,0,param);
                obj.get_GPIO_mode(id_num);
                obj.save_config(id_num);
                obj.reboot(id_num)
            end
        end
        %配置软件限位范围
        function set_angle_range(obj, id_num, angle_min, angle_max)
            angle = obj.read_property(id_num,38007,0);
            if obj.READ_FLAG == 1
                if angle >= angle_min && angle <= angle_max
                    obj.write_property(id_num, 38010, 0, angle_min);
                    obj.write_property(id_num, 38011, 0, angle_max);
                    obj.write_property(id_num, 31202, 1, 1);
                    obj.save_config(id_num);
                end
            end
        end
        %设置速度曲线轨迹类型
        function set_traj_mode(obj, id_num, mode)
            obj.write_property(id_num, 35104, 3, mode);
            obj.TRAJ_MODE = mode;
        end
        %参数属性修改
        function write_property(obj,id_num,address,data_type,value)
            data_types = ["f", "u16", "s16", "u32", "s32"];
            data = format_data([single(address), single(data_type), single(value)], "u16 u16 " + data_types(data_type+1), 'encode');
            obj.send_command(id_num , 0x1F, data, 0)
        end
        %读取电机ID号
        function id = get_id(obj, id_num)
            id = obj.read_property(id_num,31001,3);
        end
        %读取电机角度和速度
        function state = get_state(obj, id_num)
            data = format_data([0x00, 0x00], 'u16 u16', 'encode');
            obj.send_command(id_num, 0x1E, data, 1);  % rtr=1用于发送前清空接收区
            cdata = obj.receive_data();
            if obj.READ_FLAG == 1 && id_num <= obj.MOTOR_NUM
                if id_num == 0
                    id_num = bitand(bitshift(obj.can_id,-5),0x3F);
                end
                factor = 0.01;
                data = format_data(cdata, 'f s16 s16', 'decode');
                obj.motor_state(round(id_num),1) = data(1);
                obj.motor_state(round(id_num),2) = data(2) * factor;
                obj.motor_state(round(id_num),3) = data(3) * factor;
                obj.motor_state(round(id_num),4) =round(bitshift(bitand(obj.can_id,0x02),-1));
                obj.motor_state(round(id_num),5) = round(bitshift(bitand(obj.can_id,0x04),-2));
                state = [data(1),data(2) * factor];
            else
                obj.READ_FLAG = -1;
                obj.reply_state_error = obj.reply_state_error + 1;
                state = [];
            end
        end
        %读取电机电压和电流
        function volcur = get_volcur(obj, id_num)
            vol  = obj.read_property(id_num, 1, 0);
            if obj.READ_FLAG == 1
                cur  = obj.read_property(id_num, 33206,0);
                if obj.READ_FLAG == 1
                    volcur = [vol,cur];
                else
                    volcur = [];
                end
            else
                volcur = [];
            end
        end
        %获取GPIO配置参数
        function mode = get_GPIO_mode(obj, id_num)
            enable_uart = obj.read_property(id_num, 10008,3);
            if obj.READ_FLAG==1
                enable_step_dir = obj.read_property(id_num,31006,3);
                if obj.READ_FLAG==1
                    mode = [enable_uart, enable_step_dir];
                else
                    mode = [];
                end
            else
                mode = [];
            end
        end
        %参数属性读取
		%参数中的data_type（0-float，1-uint16,2-int16,3-uint32,4-int32）用来指定对应地址变量的数据类型，查表《DrEmpower系列电机常用参数及地址表》确定
        function value = read_property(obj,id_num,address,data_type)
            data = format_data([address, data_type], 'u16 u16', 'encode');
            obj.send_command(id_num, 0x1E, data, 1);  % rtr=1用于发送前清空接收区
            cdata = obj.receive_data();
            if obj.READ_FLAG == 1
                data_types = ["f", "u16", "s16", "u32", "s32"];
                property = format_data(cdata, "u16 u16 " + data_types(data_type+1), 'decode');
                if length(property) > 0
                    switch data_type
                        case 1
                            value = uint16(property(end));
                        case 2
                            value = int16(property(end));
                        case 3
                            value = uint32(property(end));
                        case 4
                            value = int32(property(end));
                        otherwise
                            value = single(property(end));
                    end
                else
                    value = [];
                end
            else
                value = [];
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%系统辅助函数%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %清除电机错误标志
        function clear_error(obj,id_num)
            order_num = 0x04;
            data = format_data([order_num], 'u32', 'encode');
            obj.send_command(id_num , 0x08, data, 0);
        end
        %读取电机错误标志
        function error_code = dump_error(obj,id_num)
            axis_error = obj.read_property(id_num, 30001,3);
            if obj.READ_FLAG==1&&axis_error~=0
                error_code = 1;
                return;
            elseif obj.READ_FLAG~=1
                error_code = -1;
                return;
            end
            motor_error = obj.read_property(id_num, 33001,3);
            if obj.READ_FLAG==1&&motor_error~=0
                error_code = 2;
                return;
            elseif obj.READ_FLAG~=1
                error_code = -2;
                return;
            end
            controller_error = obj.read_property(id_num, 32001,3);
            if obj.READ_FLAG==1&&controller_error~=0
                error_code = 3;
                return;
            elseif obj.READ_FLAG~=1
                error_code = -3;
                return;
            end
            encoder_error = obj.read_property(id_num, 34001,3);
            if obj.READ_FLAG==1&&encoder_error~=0
                error_code = 4;
                return;
            elseif obj.READ_FLAG~=1
                error_code = -4;
                return;
            end
            can_error = obj.read_property(id_num, 20001,3);
            if obj.READ_FLAG==1&&can_error~=0
                error_code = 5;
                return;
            elseif obj.READ_FLAG~=1
                error_code = -5;
                return;
            end
            fet_thermistor_error = obj.read_property(id_num, 36001,3);
            if obj.READ_FLAG==1&&fet_thermistor_error~=0
                error_code = 6;
                return;
            elseif obj.READ_FLAG~=1
                error_code = -6;
                return;
            end
            motor_thermistor_error = obj.read_property(id_num, 37001,3);
            if obj.READ_FLAG==1&&motor_thermistor_error~=0
                error_code = 7;
                return
            elseif obj.READ_FLAG~=1
                error_code = -7;
                return;
            else
                error_code = 0;
            end
        end
        %保存配置
        function save_config(obj,id_num)
            order_num = 0x01;
            data = format_data([order_num], 'u32', 'encode');
            obj.send_command(id_num , 0x08, data, 0);
        end
        %软件重启
        function reboot(obj,id_num)
            order_num = 0x03;
            data = format_data([order_num], 'u32', 'encode');
            obj.send_command(id_num , 0x08, data, 0);
        end
        %         function error_count = test(obj)
        %             error_count = 0;
        %             t1 = tic
        %             for i = 1:1000
        %                 obj.get_id(0);
        %                  obj.write_property(0,31001,3,1);
        %
        %                 if obj.READ_FLAG~=1
        %                     error_count = error_count +1
        %                 end
        %             end
        %             time = toc(t1)/1000
        %         end
    end %成员函数块--结束
end %类定义结束

function rdata = format_data(data, format, type)
format_list = string(format).split();
rdata = [];
if type == "decode" && length(data) == 8
    p = 0;
    for i =1:length(format_list)
        f = format_list(i);
        len = 0;
        s_f = '';
        if f == 'f'
            len = 4;
            s_f = 'single';
        elseif f == 'u16'
            len = 2;
            s_f = 'uint16';
        elseif f == 's16'
            len = 2;
            s_f = 'int16';
        elseif f == 'u32'
            len = 4;
            s_f = 'uint32';
        elseif f == 's32'
            len = 4;
            s_f = 'int32';
        end
        ba = [];
        if len > 0
            for j = 1:len
                ba=[ba,data(p+1)];
                p = p+1;
            end
            rdata = [rdata,single(typecast(uint8(ba),s_f))];
        else
            rdata = [];
        end
    end
elseif type=="encode"&&length(format_list)==length(data)
    for i =1:length(format_list)
        f = format_list(i);
        ba = 0;
        len = 0;
        if f == 'f'
            ba = single(data(i));
            len = 4;
        elseif f == 'u16'
            ba = uint16(data(i));
            len = 2;
        elseif f == 's16'
            ba = int16(data(i));
            len = 2;
        elseif f == 'u32'
            ba = uint32(data(i));
            len = 4;
        elseif f == 's32'
            ba = int32(data(i));
            len = 4;
        end
        bytes = [];
        if len > 0
            bytes = typecast(ba,'uint8');
            rdata = [rdata,bytes];
        else
            print32('unkown format in format_data(): ' + f);
            rdata = [];
        end
    end
end
end