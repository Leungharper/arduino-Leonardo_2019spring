//2019.3.30————测距 + 声控、叠方块（从三个位置）

#include <NewPing.h>
#include <LobotServoController.h>
#include "U8glib.h"

#define TRIG    9         /*超声波TRIG引脚为 9号IO*/
#define ECHO    8         /*超声波ECHO引脚为 8号IO*/
#define MAX_DISTANCE 40   /*最大检测距离为40cm*/

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0);  //实例化OLED类

LobotServoController myse;
LobotServoController myse1(Serial1);
LobotServoController Controller(Serial1);  //实例化舵机控制板二次开发类,使用1号串口作为通信接口
NewPing Sonar(TRIG, ECHO, MAX_DISTANCE);      //实例化超声波测距类
int gDistance;    //全局变量，用于存储超声波测得的距离

/*Logo字模数据*/
const  U8G_PROGMEM unsigned char gImage_mm[1024] = { /* 0X10,0X01,0X00,0X80,0X00,0X40, */
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X20, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X60, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X20, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X20, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X02, 0X00, 0X00, 0XE0, 0XFF, 0X3F, 0XF0, 0XFF, 0XFF, 0X01, 0XFC, 0XFF, 0XFF, 0XFF, 0XFF, 0X7F,
  0X07, 0X00, 0X00, 0XF8, 0XFF, 0XFF, 0XF0, 0XFF, 0XFF, 0X07, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF,
  0X07, 0X00, 0X00, 0XF8, 0XFF, 0XFF, 0XF1, 0XFF, 0XFF, 0X8F, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0X7F,
  0X07, 0X00, 0X00, 0X1C, 0X00, 0XC0, 0X31, 0X00, 0X00, 0XCE, 0X03, 0X00, 0X00, 0X00, 0X38, 0X00,
  0X07, 0X00, 0X00, 0X0C, 0X00, 0X80, 0X33, 0X00, 0X00, 0XCC, 0X01, 0X00, 0X00, 0X00, 0X38, 0X00,
  0X07, 0X00, 0X00, 0X0E, 0X00, 0X00, 0X73, 0X00, 0X00, 0XCE, 0X00, 0X00, 0X70, 0X00, 0X18, 0X00,
  0X07, 0X00, 0X00, 0X0E, 0X06, 0X03, 0XF3, 0XFF, 0XFF, 0XC7, 0X00, 0X00, 0X70, 0X00, 0X1C, 0X00,
  0X07, 0X00, 0X00, 0X0E, 0X06, 0X03, 0XF3, 0XFF, 0XFF, 0XC7, 0X00, 0X00, 0X70, 0X00, 0X1C, 0X00,
  0X07, 0X00, 0X00, 0X0E, 0X00, 0X00, 0X73, 0X00, 0X00, 0XCE, 0X00, 0X00, 0X70, 0X00, 0X1C, 0X00,
  0X07, 0X00, 0X00, 0X00, 0X00, 0X80, 0X33, 0X00, 0X00, 0XCC, 0X01, 0X00, 0X38, 0X00, 0X1C, 0X00,
  0X07, 0X00, 0X00, 0X00, 0X00, 0XC0, 0X71, 0X00, 0X00, 0X8E, 0X03, 0X00, 0X3C, 0X00, 0X1C, 0X00,
  0XFE, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XF1, 0XFF, 0XFF, 0X87, 0XFF, 0XFF, 0X1F, 0X00, 0X1C, 0X00,
  0XFC, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XF0, 0XFF, 0XFF, 0X07, 0XFF, 0XFF, 0X0F, 0X00, 0X1C, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0XF8, 0X7E, 0X00, 0X80, 0XF9, 0X01, 0X00, 0X20, 0X00, 0X00, 0X08, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X04, 0X03, 0X00, 0XC0, 0X00, 0X03, 0X80, 0XFF, 0X1F, 0X00, 0XFC, 0X7F, 0X00, 0X00,
  0X00, 0X00, 0X04, 0X01, 0X00, 0X60, 0X00, 0X02, 0X80, 0XFF, 0X0F, 0X00, 0X06, 0X60, 0X00, 0X00,
  0X00, 0X00, 0X04, 0X03, 0X00, 0X30, 0X03, 0X02, 0X80, 0X00, 0X10, 0X00, 0X80, 0X01, 0X00, 0X00,
  0X00, 0X00, 0X60, 0X4B, 0X00, 0XC0, 0X01, 0X02, 0X80, 0XFF, 0X09, 0X00, 0X8C, 0X31, 0X00, 0X00,
  0X00, 0X00, 0X18, 0X21, 0X00, 0X60, 0X06, 0X02, 0X80, 0X01, 0X18, 0X00, 0X8C, 0X21, 0X00, 0X00,
  0X00, 0X00, 0X0C, 0X61, 0X00, 0X30, 0X06, 0X02, 0X80, 0X61, 0X08, 0X00, 0X04, 0X21, 0X00, 0X00,
  0X00, 0X00, 0XC6, 0XC1, 0X00, 0XF0, 0XC3, 0X03, 0X80, 0X30, 0X18, 0X00, 0XC6, 0X40, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
  0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
};

//用到的字模数据
const u8g_fntpgm_uint8_t chinese_test[] U8G_SECTION(".progmem.chinese_test") = {
  0, 16, 16, 0, 254, 0, 0, 0, 0, 0, 161, 165, 0, 14, 254, 0, 0,
  //"电"
  13, 16, 32, 16, 2, 254, 4, 0, 4, 0, 4, 0, 255, 224, 132,
  32, 132, 32, 132, 32, 255, 224, 132, 32, 132, 32, 132, 32, 255, 224, 132,
  40, 4, 8, 4, 8, 3, 248,
  //"压“
  15, 15, 30, 16, 0, 254, 63, 254, 32, 0, 32, 128, 32, 128, 32,
  128, 32, 128, 47, 252, 32, 128, 32, 128, 32, 144, 32, 136, 32, 136, 64,
  128, 95, 254, 128, 0,
  //"距"
  15, 14, 28, 16, 0, 255, 125, 254, 69, 0, 69, 0, 69, 0, 125,
  252, 17, 4, 17, 4, 93, 4, 81, 4, 81, 252, 81, 0, 93, 0, 225,
  0, 1, 254,
  //"离“
  15, 16, 32, 16, 0, 254, 2, 0, 1, 0, 255, 254, 0, 0, 20,
  80, 19, 144, 20, 80, 31, 240, 1, 0, 127, 252, 66, 4, 68, 68, 79,
  228, 68, 36, 64, 20, 64, 8,
  //":"
  2, 8, 8, 16, 7, 0, 192, 192, 0, 0, 0, 0, 192, 192,
};

void draw() {                 //OLED绘制
  static uint32_t TimerDraw;  //静态变量，用于计时

  if (TimerDraw > millis())   //如果TimerDraw 小于 运行总毫秒数，返回，否则继续运行此函数
    return;

  u8g.firstPage();
  do {
    u8g.setFont(chinese_test);                //设置字体位chinese_test
    u8g.drawStr( 0, 20, "\xa1\xa2\xa5");      //显示字符串 电压 ：
    u8g.setFont(u8g_font_unifont);            //设置字体为u8g_font_unifont
    u8g.setPrintPos(18 * 2 + 10, 20);         //设置位置为 (46,20)
    u8g.print(Controller.batteryVoltage);     //显示电池电压
    u8g.print(" mv");                         //显示单位"mv"
    u8g.setFont(chinese_test);                //设置字体为chinese_test
    u8g.drawStr( 0, 19 + 20, "\xa3\xa4\xa5"); //显示字符串 距离：
    u8g.setFont(u8g_font_unifont);            //设置字体为u8g_font_unifont
    u8g.setPrintPos(18 * 2 + 10, 19 + 20);    //设置位置为 (46,39)
    u8g.print(gDistance);                     //显示电池电压
    u8g.print(" cm");                         //显示单位"mv"
  } while ( u8g.nextPage());

  TimerDraw = millis() + 500;                 //TimerDraw 赋值为 运行的总毫秒数 + 500，实现500毫秒后再次运行
}

bool ledON = true;            //led点亮标识，true时点亮，false熄灭
void ledFlash() {
  static uint32_t Timer;      //定义静态变量Timer， 用于计时
  if (Timer > millis())       //Timer 大于 millis（）（运行的总毫秒数）时返回，
    return;

  //Timer 小于 运行总毫秒数时继续
  if (ledON) {
    digitalWrite(13, HIGH);   //如果点亮标识true，13号IO置高电平,板上LED灯被点亮
    Timer = millis() + 20;    //Timer = 当前运行的总毫秒数 + 20  实现 20毫秒后再次运行
    ledON = false;            //置点亮标识为false
  } else {
    ledON = false;            //如果点亮标识不是true，置点亮标识为false
    digitalWrite(13, LOW);    //置13号IO为低电平，板上LED熄灭
  }
}

int getDistance() {       //获得距离
  uint16_t lEchoTime;     //变量 ，用于保存检测到的脉冲高电平时间
  lEchoTime = Sonar.ping_median(6);           //检测6次超声波，排除错误的结果
  int lDistance = Sonar.convert_cm(lEchoTime);//转换检测到的脉冲高电平时间为厘米
  return lDistance;                           //返回检测到的距离
}

void updateBatteryState() {      //更新电池电压状态
  static uint32_t Timer = 0;     //静态变量Timer，用于计时

  if (Timer > millis())          //如果Timer小于 运行总毫秒数，返回，否则继续运行此函数
    return;
  Controller.getBatteryVoltage();//发送获得电池电压命令

  Timer = millis() + 1000;       //Timer 赋值为 运行的总毫秒数 + 1000，实现1000毫秒后再次运行
}

void updateDistance() {          //更新距离
  static uint32_t Timer = 0;     //静态变量Timer，用于计时

  if (Timer > millis())          //如果Timer小于 运行总毫秒数，返回，否则继续运行此函数
    return;

  gDistance = getDistance();     //获得距离，并将距离保存在gDistance

  Timer = millis() + 100;        //Timer 赋值为 运行的总毫秒数 + 500，实现500毫秒后再次运行(改动）
}

void moveBlock () {
  static uint32_t Timer = 0;    //定义静态变量Timer ,用于计时
  static uint8_t  stepx = 0;    //定义静态变量stepx ,用于步骤记录

  if (Timer > millis())         //如果Timer 小于 运行总毫秒数，返回，否则继续运行此函数
    return;

  gDistance = getDistance();    //获得距离，并将距离保存在gDistance

  switch (stepx) {              //根据stepx 进行分支
    case 0:
      if (gDistance >= 5 && gDistance < 14) {   //如果距离>=5cm、<14cm
        if (!Controller.isRunning) {            //如果机械手不是在运行动作组
          myse.moveServos(6,2000,  1,1500,2,1500,3,1500,4,1500,5,1500,6,1500);delay(100); //复位
		  
          myse.moveServos(6,1000,  1, 995,2,1500,3,1500,4,1500,5,1500,6,1259);delay(100);
          myse.moveServos(6,2000,  1, 995,2,1500,3,1575,4,1316,5, 656,6,1259);delay(100);
          myse.moveServos(6,2000,  1,1532,2,1500,3,1575,4,1316,5, 656,6,1259);delay(100);
		  
          myse.moveServos(6,2000,  1,1532,2,1500,3,1575,4,1382,5,1446,6,1259);delay(100);
          myse.moveServos(6,2000,  1,1532,2,1500,3,1575,4,1382,5,1446,6,2400);delay(100);
		  myse.moveServos(6,2000,  1,1532,2,1500,3,1575,4,1164,5, 750,6,2400);delay(100);
		  
		  myse.moveServos(6,2000,  1,1532,2,1500,3,1650,4,1075,5, 750,6,2400);delay(100);
		  myse.moveServos(6,2000,  1, 930,2,1500,3,1650,4,1075,5, 750,6,2400);delay(100);
		  myse.moveServos(6,2000,  1, 930,2,1500,3,1650,4,1360,5,1400,6,2400);delay(100);
          
          myse.moveServos(6,2000,  1,1500,2,1500,3,1500,4,1500,5,1400,6,1500);delay(100); //复位
          ledON = true;                         //置亮灯标识为true
          stepx = 1;
        }
      } else {
        stepx = 1;
      }//步骤赋值为1
      break;         //退出switch

    case 1:
      if (gDistance >= 14 && gDistance < 20) {   //如果距离>=14cm、<20cm
        if (!Controller.isRunning) {            //如果机械手不是在运行动作组
		  myse.moveServos(6,2000,  1,1500,2,1500,3,1500,4,1500,5,1500,6,1500);delay(100); //复位
		  
          myse.moveServos(6,1000,  1, 887,2,1500,3,1500,4,1500,5,1400,6,1403);delay(100);
          myse.moveServos(6,2000,  1, 887,2,1630,3,1495,4,1489,5, 576,6,1403);delay(100);
          myse.moveServos(6,2000,  1,1554,2,1630,3,1495,4,1489,5, 576,6,1403);delay(100);
		  
          myse.moveServos(6,2000,  1,1554,2,1630,3,1495,4,1489,5,1400,6,1403);delay(100);
          myse.moveServos(6,2000,  1,1554,2,1630,3,1495,4,1489,5,1400,6,2415);delay(100);
		  myse.moveServos(6,2000,  1,1554,2,1486,3,1510,4,1199,5, 795,6,2400);delay(100);
		  
		  myse.moveServos(6,2000,  1, 909,2,1486,3,1490,4,1199,5, 795,6,2400);delay(100);
		  myse.moveServos(6,2000,  1, 909,2,1486,3,1490,4,1199,5,1400,6,2400);delay(100);
		  
          myse.moveServos(6,2000,  1,1500,2,1500,3,1500,4,1500,5,1400,6,1500);delay(100); //复位
          ledON = true;                         //置亮灯标识为true
          stepx = 2;
        }
      } else {
        stepx = 2;     //步骤赋值为2
      }
      break;         //退出switch

    case 2:
      if (gDistance >= 20 && gDistance <= 30) {   //如果距离>=20cm、<=30cm
        if (!Controller.isRunning) {            //如果机械手不是在运行动作组
          myse.moveServos(6,2000,  1,1500,2,1500,3,1500,4,1500,5,1500,6,1500);delay(100); //复位
		  
          myse.moveServos(6,1000,  1, 887,2,1500,3,1500,4,1500,5,1400,6,1498);delay(100);
          myse.moveServos(6,2000,  1, 737,2,1795,3,1640,4,1365,5, 615,6,1568);delay(100);
          myse.moveServos(6,2000,  1,1554,2,1795,3,1640,4,1365,5, 615,6,1568);delay(100);
		  
          myse.moveServos(6,2000,  1,1554,2,1486,3,1640,4,1365,5,1440,6,1568);delay(100);
          myse.moveServos(6,2000,  1,1554,2,1486,3,1745,4,1255,5,1440,6,2415);delay(100);
		  myse.moveServos(6,2000,  1,1554,2,1486,3,1640,4, 970,5,1005,6,2415);delay(100);
		  
		  myse.moveServos(6,2000,  1, 909,2,1486,3,1640,4, 970,5,1005,6,2415);delay(100);
		  myse.moveServos(6,2000,  1, 909,2,1486,3,1640,4, 970,5,1400,6,2415);delay(100);
		  
          myse.moveServos(6,2000,  1,1500,2,1500,3,1500,4,1500,5,1400,6,1500);delay(100); //复位
          ledON = true;                         //置亮灯标识为true
          stepx = 0;
        }
      } else {
        stepx = 0;    //步骤赋值为0，回到第一步
      }
      break;        //退出switch

    default:        //未定义步骤时默认操作，步骤数赋值0重新回到第一步
      stepx = 0;
      break;
  }
  Timer = millis() + 50;     //Timer 赋值为 运行的总毫秒数 + 50，实现50毫秒后再次运行
}


void setup() {
  // put your setup code here, to run once
  u8g.firstPage();                            //OLED绘制
  do {
    u8g.drawXBMP( 0, 0, 128, 64, gImage_mm);  //在左上角(0,0)，右下角(128,64)的空间上绘制图片gImage_mm
  } while ( u8g.nextPage() );

  Serial.begin(9600);                        //初始化0号串口
  Serial1.begin(9600);                       //初始化1号串口
  pinMode(13, OUTPUT);                        //设置板上led 13号IO口为输出
  Controller.runActionGroup(0, 1);            //运行0号动作组， 回初始位置****************************************************************************************************************************************************复位不变
  delay(2000);                                //延时两秒，简单延时，确保0号动作组运行完毕
  Controller.stopActionGroup();               //停止动作运行，确保停止
  //———— 以下是声音传感器设置 ————
  pinMode(7,INPUT_PULLUP);     //配置7号IO口为输入【并拉高】，要将声音传感器的OUT引脚接在7号IO上
  pinMode(13,OUTPUT);   //配置Arduino 板上led所在的13号IO为输入[此为LED灯IO口]
}

void loop() {
  // put your main code here, to run repeatedly:
  updateBatteryState();       //更新电池电压状态
  updateDistance();           //更新距离
  Controller.receiveHandle(); //接收处理函数，从串口接收缓存中取出数据
  draw();                     //OLED绘制
  if(digitalRead(7) == LOW){
    moveBlock();                //移动物体的逻辑实现
    //delay(2000);   //延时一段时间，等待动作组运行
    digitalWrite(13,LOW);    //熄灭Arduino上的LED灯
  }
  //ledFlash();                 //led闪灯，用于运行状态提示
}

