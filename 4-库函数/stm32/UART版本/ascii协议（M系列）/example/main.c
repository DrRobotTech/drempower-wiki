把下面代码放在main的/* USER CODE BEGIN WHILE */下面

  int num=10,num1;     //num为指定角度，num1记录当前角度
    while (1)
    {
      /* USER CODE END WHILE */
  	  set_angle(0, num, 10, 10, 1); //转动到指定角度
  	  HAL_Delay(1000);    //延时1秒
  	  num1=get_state(0).angle;  //获取当前的角度数据
  	  HAL_Delay(1000);
  	      if(READ_FLAG==1){
  	      	if(num<=(num1+0.1)&&num>=(num1-0.1)){  //获取,当前角度并和设置的角度差在0.1内，电机旋转10°
  	      		num=num+10;
  	      	}
  	      	if(num1>360){  //当前设置角度大于360°跳出循环
  	      		num=10;
  	      		set_angle(0, num, 10, 10, 1);
  	      		break;
  	      	}
  	      }
  	      else{
  	      	break;//获取失败跳出循环
  	      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

