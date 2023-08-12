#include "DrEmpower.h"
//uint8_t idList[] = {1, 20, 33, 45};
//float paramList[] = {0, 1, 12, 16.8};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}
int angle=10,pre_angle;//angle为指定角度，pre_angle记录当前角度
void loop() {
  // put your main code here, to run repeatedly:
  set_angle(0, angle, 10, 10, 1);//转动到指定角度
  delay(1000);
  pre_angle=get_state(0).angle;//获取当前的角度数据
  delay(1000);
    if(READ_FLAG==1){
          if(angle<=(pre_angle+0.1)&&angle>=(pre_angle-0.1)){  //获取,当前角度并和设置的角度差在0.1内，电机旋转10°
            angle=angle+10;
          }
          if(pre_angle>360){  //当前设置角度大于360°跳出循环
            angle=10;
            set_angle(0, angle, 10, 10, 1);
             exit(0);
//            break;
          }
        }
        else{
           exit(0);
//          break;//获取失败跳出循环
        }

}
