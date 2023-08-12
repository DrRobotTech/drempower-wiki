//把下面代码放在main的/* USER CODE BEGIN WHILE */下面

//例程1：在正负180度之间来回转动
uint8_t id_num = 1;
while (1)
{
    /* USER CODE END WHILE */
    set_angle(id_num,180,30,60,1);
    position_done(id_num);
    //用实时状态读取接口函数get_state实现position_done功能
    // motor_state[id_num-1][3] = 0;       //发送新指令后，先把是否已到达目标位置置0,
	//   while(motor_state[id_num-1][3]!=1)  //为id_num号关节电机实时状态保存在motor_state[id_num-1]数组中，包含位置、速度、扭矩、是否到达目标位置、是否报错5个状态量
    // {
    //     get_state(id_num);  //采用实时状态快速读取接口函数
    // }
    set_angle(id_num,-180,30,60,1);
    position_done(id_num);
    //用实时状态读取接口函数get_state实现position_done功能
    // motor_state[id_num-1][3] = 0;
    // while(motor_state[id_num-1][3]!=1)
    // {
    //     get_state(id_num);  //采用实时状态快速读取接口函数
    // }
    /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */

