/************************************************************
**  名称: 样题7主车底层程序
	版本: V5.21-15:45
	修改: 无
*************************************************************/
#include "stm32f10x.h"  
#include "sys.h"					
#include "usart.h"		
#include "delay_drv.h"
#include "init.h"
#include "led.h"		 	 
#include "test.h"
#include "djqd.h"
#include "key.h"
#include "xj.h"
#include "csb.h"
#include "hw.h"
#include "74hc595.h"
#include "bh1750.h"
#include "uart_my.h"
#include "canp_hostcom.h"
#include "can_drv.h"
#include "fifo_drv.h"
#include "data_channel.h"
#include "power_check.h"
#include "syn7318.h"
#include "iic.h"
#include "hmc5883l.h"
#include "full_automation.h"


#define  ZCKZ_ADDR    0xAA  // 定义运输标志物地址编号
#define  YSBZW_ADDR   0x02  // 定义运输标志物地址编号
#define  DZ_ADDR      0x03  // 定义道闸标志物地址编号
#define  LEDXS_ADDR   0x04  // 定义LED显示标志物地址编号
#define  JXB_ADDR     0x05  // 定义机械臂标志物地址编号

void Key_Onclink();
void IO_Init(void); //IO初始化	
void DIY_BEEP(u16 T_Hz,u8 timer);
void ab_c();

u8 G_Tab[6];	   //定义红外发射数组
u8 S_Tab[NUM]; 	   //定义主返回数据数组
u8 C_Tab[NUM]; 	   //定义运动标志物返回数据数组

u8 Stop_Flag=0;    //状态标志位
u8 Track_Flag=0;     //循迹标志位
u8 G_Flag=0;	   //前进标志位
u8 B_Flag=0;	   //后退标志位
u8 L_Flag=0;	   //左转标志位
u8 R_Flag=0;	   //右转标志位
u8 SD_Flag=1;      //运动标志物数据返回允许标志位
u8 Rx_Flag =0;

u16 CodedDisk=0;   //码盘值统计
u16 tempMP=0;	   //接收码盘值
u16 MP;			   //控制码盘值
int Car_Spend = 50;//小车速度默认值
u32 count = 0;	   //在循迹时  计数遇到循迹灯全亮的次数  调节此参数可防止循迹灯误判
int LSpeed;		   //循迹左轮速度
int RSpeed;		   //循迹右轮速度
u8 Line_Flag=0;	   // 
u8 send_Flag=0;	   // 发送标志位

unsigned Light=0; //光照度

u16 error_Flag = 0;

// 主函数

RCC_ClocksTypeDef RCC_Clocks;

void SysTick_Handler(void)
{
	global_times++;
	if(delay_ms_const)
		delay_ms_const--;
}


extern void pwm_test(void);

int main(void)
{
	u8 ut=0;
	u8 i;
	u16 track_temp_data =0;
	global_times = 0;
	SystemInit();
	Delay_us_Init();
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);	
	
	NVIC_Configuration(); 	 //设置NVIC中断分组0:2位抢占优先级，2位响应优先级
		
	
	SYN7318_Init();
	

	IO_Init();                  //IO初始化

	S_Tab[0]=0x55;
	S_Tab[1]=0xaa;

	C_Tab[0]=0x55;
	C_Tab[1]=0x02;
	
	CanP_Init();
	
	Power_Check();  //电量检测  上电先检测一次电量
	Send_Electric( Electric_Buf[0],Electric_Buf[1]);
	

	STOP();
	Host_Close_UpTrack();
	
	Send_Debug_Info("A1B2C3\n",8); // 上传调试信息
	
 	Set_Track_Init();			   //设置循迹初始化参数
	
	//Readcard_daivce_Init();

	while(1)
	{	 
		Key_Onclink();
		while(Full_Circulate_Flag)
		{
			LED1 = 1;
			LED0 = 0;
			Full_Motion();
			error_Flag = 0;				//超时状态标志位清零
			Wifi_Rx_flag = 0;			//WIFI接收状态标志位清零
			Zigbee_Rx_flag = 0;			//ZigBee接收状态标志位清零
		}
		
		if(ETC_Open_Flag == 1)					
		{
			if(error_Flag == 50 || error_Flag == 100 || 
			   error_Flag == 150 || error_Flag == 200 || error_Flag == 250)		// 5s 10s 15s
			{
				Full_Go(80,3);
			}
			
		}
		
		LED0 = !LED0;					//程序状态	
		delay_ms(100);
		LED1 = 0;
		if(Wifi_Rx_flag ==1)  			// wifi 接收标记
		{
			  Wifi_Rx_flag =0;
			  if(Wifi_Rx_Buf[0]==0x55)  // 接收到55开头数据
			  {			  	 
			  	     Normal_data(); 	//正常数据处理
			  }
		}

		if(Rx_Flag ==1)						   //接收到控制指令
		{
			if(Wifi_Rx_Buf[1]==ZCKZ_ADDR) 	   //主车控制
			{
				switch(Wifi_Rx_Buf[2])
				{
					case 0xB1:				   //开始指令
						//Car_Position_news[0] = Wifi_Rx_Buf[3];		//得到主车起始位置
						//Car_Position_news[1] = Wifi_Rx_Buf[4];		//得到主车入库位置
						//Car_Position_news[2] = Wifi_Rx_Buf[5];		//未使用
						mark = 5;
						Full_Circulate_Flag = 1;
						break;
				}
	    	Rx_Flag =0;
			}
		} 
		
		if(Zigbee_Rx_flag ==1)	 //zigbee返回信息
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[1]==0x0c) && (ETC_Open_Flag == 1)) 			//ETC
			{
			   if(Zigb_Rx_Buf[2]==0x01)
			   {
					if(Zigb_Rx_Buf[3]==0x01)							//ETC打开
					{
						Full_Circulate_Flag = 1;
					}					
			   }
			}
		}
	}
	
}

void Key_Onclink()
{
		//u8 Traffic[6]={0xff,0x14,0x02,0x00,0x00,0x00};
		//u8 LED[6]={1,2,3,4,5,6};
		 if(!KEY0)			  //按键K1按下=======================================================================================================
		 {
			 delay_ms(30);
				beep=1;
				delay_ms(10);
				beep=0;
			 if(!KEY0)
			 {
        Transmition(HW_K,6);
				 
				 
				 
				 
				 
				 
				 
				 
				 
				 
				 //				 int i;						
//				 const int N = 3;
//				 //Control_LED_show_data(2,SMG_JL);//（）第二行显示距离，后面的是红外指令的地址，在test.h里面。u8 X,u8 *Data
//				 Control_LED_Nixie_light(1);	//led第一排计时，，数字1是开，2是暂停（用zigbee控制的） 
//				 delay_ms(500);
//				 Control_Sluice_gate();			//道闸_开（用zigbee的）
//				 delay_ms(500);
//				 Full_Tracking(45);	
//				 Go(30,25);					
//				 delay_ms(1000);
//				 Full_Right(80);			
//				 delay_ms(1000);				
//				 for(i=1;i<=N;i++)
//				 {
//					 if(i==1)
//					 {
//						 Go(30,100);				//进地形标志物
//						 delay_ms(3000);  //刚出地形标志物
//						 Full_Tracking(15);	//循迹  车头方向修正
//						 Go(30,25);
//					 }
//					 else
//					 {
//						 Full_Tracking(35);			//想让它在 i = 1 的时候不执行，其他情况都执行
//						 Go(30,25);	
//					 }				
//					 delay_ms(500);
//					 switch(i)
//					 {
//					  case 1:
//							Transmition(H_SD,4);							//隧道通风
//							delay_ms(1000);
//					 	 break;
//					  case 2:
//							//delay_ms(500);
//							Transmition(HW_K,6);			//报警台_开
//							delay_ms(1000);	
//					 	 break;
//					  case 3:
//							Light_StepCtr(3);			//智能路灯_挡位加2
//							delay_ms(1000);			
//					 	 break;
//					 }
//					 Full_Left(80);				//左转
//					 delay_ms(1000);		
//					 //Full_Tracking(35);					 
//				 }
//				 Full_Tracking(35);	
//				 Go(30,25);					
//				 Control_Sluice_gate();			//道闸_开（用zigbee的）
//				 delay_ms(1000);
//				 Full_Tracking(35);		//入库
//				 Go(30,25);						//进库
//				 Control_LED_Nixie_light(2);	//led第一排计时，，数字1是开，2是暂停（用zigbee控制的）
//				 
				 
				 
				 
				 
				 
				 
				 
				 
//				 Transmition(H_SD,4);							//隧道通风
																					//下一条好像是显示test.h里面的数据。
//				 Control_LED_show_data(2,SMG_JL);//（）第二行显示距离，后面的是红外指令的地址，在test.h里面。u8 X,u8 *Data
//				 Control_LED_Nixie_light(2);	//led第一排计时，，数字1是开，2是暂停（用zigbee控制的）
//				 Control_Sluice_gate();			//道闸_开（用zigbee的）
//					Transmition(H_2,4);				//智能路灯_挡位加2
//				 Transmition(HW_K,6);			//报警台_开
			 }
		}
	   if(!KEY1)			  //按键K2按下========================================================================================================
		 {  
			 delay_ms(30);
				beep=1;
				delay_ms(10);
				beep=0;
			 if(!KEY1)
			 {
					liticheku(1);
				 
				 
				 
				 
				 
				 
				 //				 int i,j;						
//		//		 const int N = 3;
//				 //Control_LED_show_data(2,SMG_JL);//（）第二行显示距离，后面的是红外指令的地址，在test.h里面。u8 X,u8 *Data
//				 Control_LED_Nixie_light(1);	//led第一排计时，，数字1是开，2是暂停（用zigbee控制的） 
//				 delay_ms(500);
//				 Control_Sluice_gate();			//道闸_开（用zigbee的）
//				 delay_ms(500);
//				 Full_Tracking(45);			//循迹前进
//				 Go(30,25);							//进入b7路口
//				 delay_ms(1000);
//				 //*************转弯操作**************
//				 Right(80);			//右转
//				 delay_ms(1000);				
//				 Car_Back(40,20);//右转后的后退
//				 delay_ms(1000);
//				 Full_Tracking(35);//循迹到地形标志物前面
//				 delay_ms(1000);
//				 //************************************
//				 //MP_Tracking(15,30);
////				 for(i=0;i;i++)
////				 {
////				 }
//				 
//				 Go(30,85);				//进地形标志物
//				 delay_ms(3000);  //刚出地形标志物
//				 Full_Tracking(15);	//循迹  车头方向修正
//				 Go(30,25);
//				 delay_ms(2000);//等待打开ETC
//				 Full_Tracking(35);
//				 Go(30,25);
//				 delay_ms(100);
//				 for(i=0;i<4;i++)		//路灯前路口
//				 {
//					 Light_StepCtr(1);//1为加1档，二为加二当
//					 delay_ms(1000);
//				 }
//				 Full_Left(80);				//左转(智能路灯的路口前)F6
//				 delay_ms(1000);
//				 Full_Tracking(35);
//				 Go(30,25);
//				 delay_ms(1000);
//				 //*************智能语音播报************
//				 SYN_7318_One_test(1,0x06);		// mode 模式测试 1 随机指令 0 指定指令   语音识�
//				 
//				 //*************************************
//				 Full_Left(80);		//左转，面向左4
//				 delay_ms(1000);
//				 
//				 Full_Tracking(35);
//				 Go(30,25);
//				 delay_ms(100);
				 
				 
				 
				 
				 
				 
				 
				 
				 
				 
				 
				 
				 
				 
				 
				 
				 
				 
				 
				 
				 
			 
//			 Transmition(H_SD,4);
//			 delay_ms(1000);
//				 Full_Tracking(35);
//			delay_ms(2000);
//			Go(50,80);//74
//			delay_ms(2000);
//			Full_Tracking(35);
				 //Send_Debug_Num(654321);
				 
				 
				  
				 
			 }
		}
	   if(!KEY2)			  //按键K3按下========================================================================================================
		{
			delay_ms(30);
				beep=1;
				delay_ms(10);
				beep=0;
			if(!KEY2)
			{ 
				Control_LED_Nixie_light(2);

				//send_data_zigbee( YY_ZZW,8);
//SYN_TTS("向左转");
				
//				stereo_reset();//立体显示
//			  Transmition(H_SD,4);//隧道风
//				Transmition(HW_K,6);//报警台

					
					
					
					
					
					
					
					
					
					
					
				
				// RL = 1 左转 RL = 2 右转 timer 决定角度  经验值 timer = 28 即45度
//				delay_ms(2500);
//				Full_RL_45(45,2,28);
//				Full_Tracking(35);
				
				
				//SYN_7318_One_test(1,0x06);		// mode 模式测试 1 随机指令 0 指定指令   语音识别
				//Control_voice(0x10,0x23);//语音播报

				
//				Full_Tracking(35);//继续下一个循迹前行，重复操作。
//			 Go(30,25);//进入十字路口中间位置
//			 delay_ms(1000);
//			 Full_Left(85);
//				delay_ms(1000);
//				Full_Tracking(35);//继续下一个循迹前行，重复操作。
//			 Go(30,25);//进入十字路口中间位置
//			 delay_ms(1000);
//				Full_Right(85);//F6路口
//			 delay_ms(1000);
//				Transmition(H_SD,4);//隧道通风系统
//				
//			 Full_Tracking(35);//继续下一个循迹前行，重复操作。
//			 Go(30,25);//进入十字路口中间位置
//			 delay_ms(2000);
//			 Full_Tracking(35);//继续下一个循迹前行，重复操作。
//			 Go(30,25);//进入十字路口中间位置
//			 delay_ms(1000);
//			 Full_Right(85);//B6路口
//			 delay_ms(1000);
//			 Full_Tracking(45);//继续下一个循迹前行，重复操作。
//			 Go(30,25);//进入十字路口中间位置
//			 Transmition(HW_K,6);
//			 delay_ms(3000);
//			 Full_Right(85);//B4路口
//			 delay_ms(1000);
//			 Full_Tracking(45);//继续下一个循迹前行，重复操作。
//			 Go(30,25);//进入十字路口中间位置
//			 Full_Tracking(45);//继续下一个循迹前行，重复操作。
//			 Go(30,25);//进入十字路口中间位置
//			 delay_ms(1000);
//			 Control_Sluice_gate();//打开道闸--------------------------道闸前的延迟
//			 delay_ms(1000);
//			 Full_Tracking(45);//继续下一个循迹前行，重复操作。
//			 Go(30,25);//进入十字路口中间位置
//			 delay_ms(1000);
//			 Full_Right(80);//转弯F4路口
//			 delay_ms(1000);
//			 Full_Tracking(40);//继续下一个循迹前行，重复操作。
//			 Go(30,25);//进入十字路口中间位置
//			 delay_ms(1000);
//			 Full_Right(80);//转弯F8路口
//			 delay_ms(1000);
			}
		}
		if(!KEY3)			    //按键K4按下========================================================================================================
		{
			delay_ms(30);
				beep=1;
				delay_ms(10);
				beep=0;
			if(!KEY3)
			{
				ab_c();

				


//        //入库充电
				
				
				
				
				
				
				

				
				
				
			
				
				
				
				
				
				
			}
				
				
				
				
				
/****************************************************************
函数功能：语音播报控制
参	  数：first为主指令  second为副指�     顄8 first,u8 second
返回  值：无
*****************************************************************/
        
//				SYN_7318_One_test(1,0x06);  // mode 模式测试 1 随机指令 0 指定指令   语音识别
//				delay_ms(1000);
//				Control_voice(0x10,0x06);//语音播报
				
				
				
				
/****************************************************************
函数功能：LED数码管控制
参	  数：Mode_num=1 打开计时  Mode_num=2 关闭计时
返回  值：无
*****************************************************************/
//        Control_LED_Nixie_light(1);//计时开
//				delay_ms(5000);
//				Control_LED_Nixie_light(0);//计时关
				
				
				
/****************************************************************
函数功能：LED数码管显示数据
参	  数：*HW 红外指令地址
返回  值：无
*****************************************************************/
//         Control_LED_show_data(2,u8 *Data);//失败，第二排显示数字不会显示
				
				
				
				
				
				//Stereo_Display(1);
//				Transmition(Traffic,6);  //立体显示  --------------
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
			}
		}




void DIY_BEEP(u16 T_Hz,u8 timer)
{
	u16 i = 0;
	for(i=0;i<timer;i++)
	{
		BEEP = 0;
		delay_ms(timer);
		BEEP = 1;
		delay_ms(timer);
	}
}
/***************************************************************
** 功能：     初始化核心板所使用端口
** 参数：	  无参数
** 返回值：    无
****************************************************************/
void IO_Init(void)
{
	YJ_INIT();			//初始化硬件

	GPIOB->CRH&=0XFFFFFFF0;	  
	GPIOB->CRH|=0X00000008;//PB8 设置成输入   			   
	GPIOB->ODR|=1<<8;	   //PB8上拉

	GPIOC->CRH&=0X000FFFFF; 
	GPIOC->CRH|=0X33300000;   //PC13/PC14/PC15推挽输出  	 
    GPIOC->ODR|=0XE000;       //PC13/PC14/PC15输出高
		   	 
	GPIOD->CRH&=0XFFF0FFFF; 
	GPIOD->CRH|=0X00030000;   //PD12推挽输出  	 
    GPIOD->ODR|=0X1000;       //PD12推挽高											  
	
	LED_L=1;				  
	LED_R=1;
	BEEP=1;
	
	beep=0;
		
}

void ab_c()
{
					int b;
				//***************************开始准备
	      Control_LED_Nixie_light(1);
	      delay_ms(100);
				liticheku(1);
	      delay_ms(100);
			  Control_Sluice_gate();
				delay_ms(100);                    
				Full_Tracking(30);
				Go(30,28);
				delay_ms(1000); 
				
        //***************************摆正车身				
				Full_Right(83);	
				delay_ms(1000);
				Car_Back(30,43);
				delay_ms(1000);
				Full_Tracking(15);               
				Go(30,10);
				delay_ms(1000);
				
				//****************************进入地标
			  Full_Tracking(15);
			  delay_ms(1000);                  
				Go(30,80);
				delay_ms(2500);                   
				Full_Tracking(15);
				Go(50,25);
				delay_ms(2000);                   
				Full_Tracking(30);                
				Go(50,30);
				
			  //*****************************灯光换挡		
				for(b=0;b<4;b++)									
				{
				Transmition(H_1,4);										
				delay_ms(700);						        
				}
				
				Full_Left(83);	
				delay_ms(1000); 
				Full_Tracking(30);
				delay_ms(1000); 
				Go(30,28);
				delay_ms(1000);
				
				//******************************语音播报左转弯
				Full_Left(83);	                  
				delay_ms(1000); 
				Full_Tracking(30);
				Go(30,28);
				delay_ms(1000);
				
				//******************************立体显示	
				Full_RL_45(83,1,28);
				delay_ms(1000);
				for(b=0;b<4;b++)									
				{
				stereo_reset();
				delay_ms(700);						        
				}		
				Full_Right(83);	
				delay_ms(1000);
				Full_Tracking(30);
				Go(30,28);
				delay_ms(1000);
				
				//*******************************报警台
				Full_Right(83);
				delay_ms(500);
				Full_RL_45(83,2,30);
				Transmition(HW_K,6);
				
				//*******************************驶出路口
				delay_ms(500);
				Full_Left(83);
				delay_ms(500);
				Full_Tracking(30);
				Go(30,20);
				delay_ms(1000);
				Full_Right(83);
				delay_ms(500);
				
				//*******************************隧道
				Full_Tracking(30);
				Transmition(H_SD,4);
				Go(30,25);				
				delay_ms(500);
				//*******************************行驶至车库门口
				delay_ms(500);				
				Full_Tracking(30);
				Go(30,28);
				delay_ms(1000);
				Full_Right(83);	
				delay_ms(1000);
				Full_Tracking(40);
				Go(30,28);
				delay_ms(1000);
				Full_Right(83);
				delay_ms(1000);
				Full_Tracking(30);				
				Go(30,28);
				delay_ms(1000);
				Full_Tracking(30);
			  Go(30,28);
				delay_ms(1000);
				Full_Right(83);
				delay_ms(1000);
				Full_Tracking(30);
				Go(30,100);	
				delay_ms(2500);
				
				//********************************入库充电
				liticheku(4);
				delay_ms(2000);
				Control_Magnetic_suspension();
				delay_ms(100);
				Control_LED_Nixie_light(2);
}