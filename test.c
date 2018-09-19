/************************************************************
**  Ãû³Æ: ÑùÌâ7Ö÷³µµ×²ã³ÌĞò
	°æ±¾: V5.21-15:45
	ĞŞ¸Ä: ÎŞ
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


#define  ZCKZ_ADDR    0xAA  // ¶¨ÒåÔËÊä±êÖ¾ÎïµØÖ·±àºÅ
#define  YSBZW_ADDR   0x02  // ¶¨ÒåÔËÊä±êÖ¾ÎïµØÖ·±àºÅ
#define  DZ_ADDR      0x03  // ¶¨ÒåµÀÕ¢±êÖ¾ÎïµØÖ·±àºÅ
#define  LEDXS_ADDR   0x04  // ¶¨ÒåLEDÏÔÊ¾±êÖ¾ÎïµØÖ·±àºÅ
#define  JXB_ADDR     0x05  // ¶¨Òå»úĞµ±Û±êÖ¾ÎïµØÖ·±àºÅ

void Key_Onclink();
void IO_Init(void); //IO³õÊ¼»¯	
void DIY_BEEP(u16 T_Hz,u8 timer);
void ab_c();

u8 G_Tab[6];	   //¶¨ÒåºìÍâ·¢ÉäÊı×é
u8 S_Tab[NUM]; 	   //¶¨ÒåÖ÷·µ»ØÊı¾İÊı×é
u8 C_Tab[NUM]; 	   //¶¨ÒåÔË¶¯±êÖ¾Îï·µ»ØÊı¾İÊı×é

u8 Stop_Flag=0;    //×´Ì¬±êÖ¾Î»
u8 Track_Flag=0;     //Ñ­¼£±êÖ¾Î»
u8 G_Flag=0;	   //Ç°½ø±êÖ¾Î»
u8 B_Flag=0;	   //ºóÍË±êÖ¾Î»
u8 L_Flag=0;	   //×ó×ª±êÖ¾Î»
u8 R_Flag=0;	   //ÓÒ×ª±êÖ¾Î»
u8 SD_Flag=1;      //ÔË¶¯±êÖ¾ÎïÊı¾İ·µ»ØÔÊĞí±êÖ¾Î»
u8 Rx_Flag =0;

u16 CodedDisk=0;   //ÂëÅÌÖµÍ³¼Æ
u16 tempMP=0;	   //½ÓÊÕÂëÅÌÖµ
u16 MP;			   //¿ØÖÆÂëÅÌÖµ
int Car_Spend = 50;//Ğ¡³µËÙ¶ÈÄ¬ÈÏÖµ
u32 count = 0;	   //ÔÚÑ­¼£Ê±  ¼ÆÊıÓöµ½Ñ­¼£µÆÈ«ÁÁµÄ´ÎÊı  µ÷½Ú´Ë²ÎÊı¿É·ÀÖ¹Ñ­¼£µÆÎóÅĞ
int LSpeed;		   //Ñ­¼£×óÂÖËÙ¶È
int RSpeed;		   //Ñ­¼£ÓÒÂÖËÙ¶È
u8 Line_Flag=0;	   // 
u8 send_Flag=0;	   // ·¢ËÍ±êÖ¾Î»

unsigned Light=0; //¹âÕÕ¶È

u16 error_Flag = 0;

// Ö÷º¯Êı

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
	
	NVIC_Configuration(); 	 //ÉèÖÃNVICÖĞ¶Ï·Ö×é0:2Î»ÇÀÕ¼ÓÅÏÈ¼¶£¬2Î»ÏìÓ¦ÓÅÏÈ¼¶
		
	
	SYN7318_Init();
	

	IO_Init();                  //IO³õÊ¼»¯

	S_Tab[0]=0x55;
	S_Tab[1]=0xaa;

	C_Tab[0]=0x55;
	C_Tab[1]=0x02;
	
	CanP_Init();
	
	Power_Check();  //µçÁ¿¼ì²â  ÉÏµçÏÈ¼ì²âÒ»´ÎµçÁ¿
	Send_Electric( Electric_Buf[0],Electric_Buf[1]);
	

	STOP();
	Host_Close_UpTrack();
	
	Send_Debug_Info("A1B2C3\n",8); // ÉÏ´«µ÷ÊÔĞÅÏ¢
	
 	Set_Track_Init();			   //ÉèÖÃÑ­¼£³õÊ¼»¯²ÎÊı
	
	//Readcard_daivce_Init();

	while(1)
	{	 
		Key_Onclink();
		while(Full_Circulate_Flag)
		{
			LED1 = 1;
			LED0 = 0;
			Full_Motion();
			error_Flag = 0;				//³¬Ê±×´Ì¬±êÖ¾Î»ÇåÁã
			Wifi_Rx_flag = 0;			//WIFI½ÓÊÕ×´Ì¬±êÖ¾Î»ÇåÁã
			Zigbee_Rx_flag = 0;			//ZigBee½ÓÊÕ×´Ì¬±êÖ¾Î»ÇåÁã
		}
		
		if(ETC_Open_Flag == 1)					
		{
			if(error_Flag == 50 || error_Flag == 100 || 
			   error_Flag == 150 || error_Flag == 200 || error_Flag == 250)		// 5s 10s 15s
			{
				Full_Go(80,3);
			}
			
		}
		
		LED0 = !LED0;					//³ÌĞò×´Ì¬	
		delay_ms(100);
		LED1 = 0;
		if(Wifi_Rx_flag ==1)  			// wifi ½ÓÊÕ±ê¼Ç
		{
			  Wifi_Rx_flag =0;
			  if(Wifi_Rx_Buf[0]==0x55)  // ½ÓÊÕµ½55¿ªÍ·Êı¾İ
			  {			  	 
			  	     Normal_data(); 	//Õı³£Êı¾İ´¦Àí
			  }
		}

		if(Rx_Flag ==1)						   //½ÓÊÕµ½¿ØÖÆÖ¸Áî
		{
			if(Wifi_Rx_Buf[1]==ZCKZ_ADDR) 	   //Ö÷³µ¿ØÖÆ
			{
				switch(Wifi_Rx_Buf[2])
				{
					case 0xB1:				   //¿ªÊ¼Ö¸Áî
						//Car_Position_news[0] = Wifi_Rx_Buf[3];		//µÃµ½Ö÷³µÆğÊ¼Î»ÖÃ
						//Car_Position_news[1] = Wifi_Rx_Buf[4];		//µÃµ½Ö÷³µÈë¿âÎ»ÖÃ
						//Car_Position_news[2] = Wifi_Rx_Buf[5];		//Î´Ê¹ÓÃ
						mark = 5;
						Full_Circulate_Flag = 1;
						break;
				}
	    	Rx_Flag =0;
			}
		} 
		
		if(Zigbee_Rx_flag ==1)	 //zigbee·µ»ØĞÅÏ¢
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[1]==0x0c) && (ETC_Open_Flag == 1)) 			//ETC
			{
			   if(Zigb_Rx_Buf[2]==0x01)
			   {
					if(Zigb_Rx_Buf[3]==0x01)							//ETC´ò¿ª
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
		 if(!KEY0)			  //°´¼üK1°´ÏÂ=======================================================================================================
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
//				 //Control_LED_show_data(2,SMG_JL);//£¨£©µÚ¶şĞĞÏÔÊ¾¾àÀë£¬ºóÃæµÄÊÇºìÍâÖ¸ÁîµÄµØÖ·£¬ÔÚtest.hÀïÃæ¡£u8 X,u8 *Data
//				 Control_LED_Nixie_light(1);	//ledµÚÒ»ÅÅ¼ÆÊ±£¬£¬Êı×Ö1ÊÇ¿ª£¬2ÊÇÔİÍ££¨ÓÃzigbee¿ØÖÆµÄ£© 
//				 delay_ms(500);
//				 Control_Sluice_gate();			//µÀÕ¢_¿ª£¨ÓÃzigbeeµÄ£©
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
//						 Go(30,100);				//½øµØĞÎ±êÖ¾Îï
//						 delay_ms(3000);  //¸Õ³öµØĞÎ±êÖ¾Îï
//						 Full_Tracking(15);	//Ñ­¼£  ³µÍ··½ÏòĞŞÕı
//						 Go(30,25);
//					 }
//					 else
//					 {
//						 Full_Tracking(35);			//ÏëÈÃËüÔÚ i = 1 µÄÊ±ºò²»Ö´ĞĞ£¬ÆäËûÇé¿ö¶¼Ö´ĞĞ
//						 Go(30,25);	
//					 }				
//					 delay_ms(500);
//					 switch(i)
//					 {
//					  case 1:
//							Transmition(H_SD,4);							//ËíµÀÍ¨·ç
//							delay_ms(1000);
//					 	 break;
//					  case 2:
//							//delay_ms(500);
//							Transmition(HW_K,6);			//±¨¾¯Ì¨_¿ª
//							delay_ms(1000);	
//					 	 break;
//					  case 3:
//							Light_StepCtr(3);			//ÖÇÄÜÂ·µÆ_µ²Î»¼Ó2
//							delay_ms(1000);			
//					 	 break;
//					 }
//					 Full_Left(80);				//×ó×ª
//					 delay_ms(1000);		
//					 //Full_Tracking(35);					 
//				 }
//				 Full_Tracking(35);	
//				 Go(30,25);					
//				 Control_Sluice_gate();			//µÀÕ¢_¿ª£¨ÓÃzigbeeµÄ£©
//				 delay_ms(1000);
//				 Full_Tracking(35);		//Èë¿â
//				 Go(30,25);						//½ø¿â
//				 Control_LED_Nixie_light(2);	//ledµÚÒ»ÅÅ¼ÆÊ±£¬£¬Êı×Ö1ÊÇ¿ª£¬2ÊÇÔİÍ££¨ÓÃzigbee¿ØÖÆµÄ£©
//				 
				 
				 
				 
				 
				 
				 
				 
				 
//				 Transmition(H_SD,4);							//ËíµÀÍ¨·ç
																					//ÏÂÒ»ÌõºÃÏñÊÇÏÔÊ¾test.hÀïÃæµÄÊı¾İ¡£
//				 Control_LED_show_data(2,SMG_JL);//£¨£©µÚ¶şĞĞÏÔÊ¾¾àÀë£¬ºóÃæµÄÊÇºìÍâÖ¸ÁîµÄµØÖ·£¬ÔÚtest.hÀïÃæ¡£u8 X,u8 *Data
//				 Control_LED_Nixie_light(2);	//ledµÚÒ»ÅÅ¼ÆÊ±£¬£¬Êı×Ö1ÊÇ¿ª£¬2ÊÇÔİÍ££¨ÓÃzigbee¿ØÖÆµÄ£©
//				 Control_Sluice_gate();			//µÀÕ¢_¿ª£¨ÓÃzigbeeµÄ£©
//					Transmition(H_2,4);				//ÖÇÄÜÂ·µÆ_µ²Î»¼Ó2
//				 Transmition(HW_K,6);			//±¨¾¯Ì¨_¿ª
			 }
		}
	   if(!KEY1)			  //°´¼üK2°´ÏÂ========================================================================================================
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
//				 //Control_LED_show_data(2,SMG_JL);//£¨£©µÚ¶şĞĞÏÔÊ¾¾àÀë£¬ºóÃæµÄÊÇºìÍâÖ¸ÁîµÄµØÖ·£¬ÔÚtest.hÀïÃæ¡£u8 X,u8 *Data
//				 Control_LED_Nixie_light(1);	//ledµÚÒ»ÅÅ¼ÆÊ±£¬£¬Êı×Ö1ÊÇ¿ª£¬2ÊÇÔİÍ££¨ÓÃzigbee¿ØÖÆµÄ£© 
//				 delay_ms(500);
//				 Control_Sluice_gate();			//µÀÕ¢_¿ª£¨ÓÃzigbeeµÄ£©
//				 delay_ms(500);
//				 Full_Tracking(45);			//Ñ­¼£Ç°½ø
//				 Go(30,25);							//½øÈëb7Â·¿Ú
//				 delay_ms(1000);
//				 //*************×ªÍä²Ù×÷**************
//				 Right(80);			//ÓÒ×ª
//				 delay_ms(1000);				
//				 Car_Back(40,20);//ÓÒ×ªºóµÄºóÍË
//				 delay_ms(1000);
//				 Full_Tracking(35);//Ñ­¼£µ½µØĞÎ±êÖ¾ÎïÇ°Ãæ
//				 delay_ms(1000);
//				 //************************************
//				 //MP_Tracking(15,30);
////				 for(i=0;i;i++)
////				 {
////				 }
//				 
//				 Go(30,85);				//½øµØĞÎ±êÖ¾Îï
//				 delay_ms(3000);  //¸Õ³öµØĞÎ±êÖ¾Îï
//				 Full_Tracking(15);	//Ñ­¼£  ³µÍ··½ÏòĞŞÕı
//				 Go(30,25);
//				 delay_ms(2000);//µÈ´ı´ò¿ªETC
//				 Full_Tracking(35);
//				 Go(30,25);
//				 delay_ms(100);
//				 for(i=0;i<4;i++)		//Â·µÆÇ°Â·¿Ú
//				 {
//					 Light_StepCtr(1);//1Îª¼Ó1µµ£¬¶şÎª¼Ó¶şµ±
//					 delay_ms(1000);
//				 }
//				 Full_Left(80);				//×ó×ª(ÖÇÄÜÂ·µÆµÄÂ·¿ÚÇ°)F6
//				 delay_ms(1000);
//				 Full_Tracking(35);
//				 Go(30,25);
//				 delay_ms(1000);
//				 //*************ÖÇÄÜÓïÒô²¥±¨************
//				 SYN_7318_One_test(1,0x06);		// mode Ä£Ê½²âÊÔ 1 Ëæ»úÖ¸Áî 0 Ö¸¶¨Ö¸Áî   ÓïÒôÊ¶±
//				 
//				 //*************************************
//				 Full_Left(80);		//×ó×ª£¬ÃæÏò×ó4
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
	   if(!KEY2)			  //°´¼üK3°´ÏÂ========================================================================================================
		{
			delay_ms(30);
				beep=1;
				delay_ms(10);
				beep=0;
			if(!KEY2)
			{ 
				Control_LED_Nixie_light(2);

				//send_data_zigbee( YY_ZZW,8);
//SYN_TTS("Ïò×ó×ª");
				
//				stereo_reset();//Á¢ÌåÏÔÊ¾
//			  Transmition(H_SD,4);//ËíµÀ·ç
//				Transmition(HW_K,6);//±¨¾¯Ì¨

					
					
					
					
					
					
					
					
					
					
					
				
				// RL = 1 ×ó×ª RL = 2 ÓÒ×ª timer ¾ö¶¨½Ç¶È  ¾­ÑéÖµ timer = 28 ¼´45¶È
//				delay_ms(2500);
//				Full_RL_45(45,2,28);
//				Full_Tracking(35);
				
				
				//SYN_7318_One_test(1,0x06);		// mode Ä£Ê½²âÊÔ 1 Ëæ»úÖ¸Áî 0 Ö¸¶¨Ö¸Áî   ÓïÒôÊ¶±ğ
				//Control_voice(0x10,0x23);//ÓïÒô²¥±¨

				
//				Full_Tracking(35);//¼ÌĞøÏÂÒ»¸öÑ­¼£Ç°ĞĞ£¬ÖØ¸´²Ù×÷¡£
//			 Go(30,25);//½øÈëÊ®×ÖÂ·¿ÚÖĞ¼äÎ»ÖÃ
//			 delay_ms(1000);
//			 Full_Left(85);
//				delay_ms(1000);
//				Full_Tracking(35);//¼ÌĞøÏÂÒ»¸öÑ­¼£Ç°ĞĞ£¬ÖØ¸´²Ù×÷¡£
//			 Go(30,25);//½øÈëÊ®×ÖÂ·¿ÚÖĞ¼äÎ»ÖÃ
//			 delay_ms(1000);
//				Full_Right(85);//F6Â·¿Ú
//			 delay_ms(1000);
//				Transmition(H_SD,4);//ËíµÀÍ¨·çÏµÍ³
//				
//			 Full_Tracking(35);//¼ÌĞøÏÂÒ»¸öÑ­¼£Ç°ĞĞ£¬ÖØ¸´²Ù×÷¡£
//			 Go(30,25);//½øÈëÊ®×ÖÂ·¿ÚÖĞ¼äÎ»ÖÃ
//			 delay_ms(2000);
//			 Full_Tracking(35);//¼ÌĞøÏÂÒ»¸öÑ­¼£Ç°ĞĞ£¬ÖØ¸´²Ù×÷¡£
//			 Go(30,25);//½øÈëÊ®×ÖÂ·¿ÚÖĞ¼äÎ»ÖÃ
//			 delay_ms(1000);
//			 Full_Right(85);//B6Â·¿Ú
//			 delay_ms(1000);
//			 Full_Tracking(45);//¼ÌĞøÏÂÒ»¸öÑ­¼£Ç°ĞĞ£¬ÖØ¸´²Ù×÷¡£
//			 Go(30,25);//½øÈëÊ®×ÖÂ·¿ÚÖĞ¼äÎ»ÖÃ
//			 Transmition(HW_K,6);
//			 delay_ms(3000);
//			 Full_Right(85);//B4Â·¿Ú
//			 delay_ms(1000);
//			 Full_Tracking(45);//¼ÌĞøÏÂÒ»¸öÑ­¼£Ç°ĞĞ£¬ÖØ¸´²Ù×÷¡£
//			 Go(30,25);//½øÈëÊ®×ÖÂ·¿ÚÖĞ¼äÎ»ÖÃ
//			 Full_Tracking(45);//¼ÌĞøÏÂÒ»¸öÑ­¼£Ç°ĞĞ£¬ÖØ¸´²Ù×÷¡£
//			 Go(30,25);//½øÈëÊ®×ÖÂ·¿ÚÖĞ¼äÎ»ÖÃ
//			 delay_ms(1000);
//			 Control_Sluice_gate();//´ò¿ªµÀÕ¢--------------------------µÀÕ¢Ç°µÄÑÓ³Ù
//			 delay_ms(1000);
//			 Full_Tracking(45);//¼ÌĞøÏÂÒ»¸öÑ­¼£Ç°ĞĞ£¬ÖØ¸´²Ù×÷¡£
//			 Go(30,25);//½øÈëÊ®×ÖÂ·¿ÚÖĞ¼äÎ»ÖÃ
//			 delay_ms(1000);
//			 Full_Right(80);//×ªÍäF4Â·¿Ú
//			 delay_ms(1000);
//			 Full_Tracking(40);//¼ÌĞøÏÂÒ»¸öÑ­¼£Ç°ĞĞ£¬ÖØ¸´²Ù×÷¡£
//			 Go(30,25);//½øÈëÊ®×ÖÂ·¿ÚÖĞ¼äÎ»ÖÃ
//			 delay_ms(1000);
//			 Full_Right(80);//×ªÍäF8Â·¿Ú
//			 delay_ms(1000);
			}
		}
		if(!KEY3)			    //°´¼üK4°´ÏÂ========================================================================================================
		{
			delay_ms(30);
				beep=1;
				delay_ms(10);
				beep=0;
			if(!KEY3)
			{
				ab_c();

				


//        //Èë¿â³äµç
				
				
				
				
				
				
				

				
				
				
			
				
				
				
				
				
				
			}
				
				
				
				
				
/****************************************************************
º¯Êı¹¦ÄÜ£ºÓïÒô²¥±¨¿ØÖÆ
²Î	  Êı£ºfirstÎªÖ÷Ö¸Áî  secondÎª¸±Ö¸Á     îu8 first,u8 second
·µ»Ø  Öµ£ºÎŞ
*****************************************************************/
        
//				SYN_7318_One_test(1,0x06);  // mode Ä£Ê½²âÊÔ 1 Ëæ»úÖ¸Áî 0 Ö¸¶¨Ö¸Áî   ÓïÒôÊ¶±ğ
//				delay_ms(1000);
//				Control_voice(0x10,0x06);//ÓïÒô²¥±¨
				
				
				
				
/****************************************************************
º¯Êı¹¦ÄÜ£ºLEDÊıÂë¹Ü¿ØÖÆ
²Î	  Êı£ºMode_num=1 ´ò¿ª¼ÆÊ±  Mode_num=2 ¹Ø±Õ¼ÆÊ±
·µ»Ø  Öµ£ºÎŞ
*****************************************************************/
//        Control_LED_Nixie_light(1);//¼ÆÊ±¿ª
//				delay_ms(5000);
//				Control_LED_Nixie_light(0);//¼ÆÊ±¹Ø
				
				
				
/****************************************************************
º¯Êı¹¦ÄÜ£ºLEDÊıÂë¹ÜÏÔÊ¾Êı¾İ
²Î	  Êı£º*HW ºìÍâÖ¸ÁîµØÖ·
·µ»Ø  Öµ£ºÎŞ
*****************************************************************/
//         Control_LED_show_data(2,u8 *Data);//Ê§°Ü£¬µÚ¶şÅÅÏÔÊ¾Êı×Ö²»»áÏÔÊ¾
				
				
				
				
				
				//Stereo_Display(1);
//				Transmition(Traffic,6);  //Á¢ÌåÏÔÊ¾  --------------
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
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
** ¹¦ÄÜ£º     ³õÊ¼»¯ºËĞÄ°åËùÊ¹ÓÃ¶Ë¿Ú
** ²ÎÊı£º	  ÎŞ²ÎÊı
** ·µ»ØÖµ£º    ÎŞ
****************************************************************/
void IO_Init(void)
{
	YJ_INIT();			//³õÊ¼»¯Ó²¼ş

	GPIOB->CRH&=0XFFFFFFF0;	  
	GPIOB->CRH|=0X00000008;//PB8 ÉèÖÃ³ÉÊäÈë   			   
	GPIOB->ODR|=1<<8;	   //PB8ÉÏÀ­

	GPIOC->CRH&=0X000FFFFF; 
	GPIOC->CRH|=0X33300000;   //PC13/PC14/PC15ÍÆÍìÊä³ö  	 
    GPIOC->ODR|=0XE000;       //PC13/PC14/PC15Êä³ö¸ß
		   	 
	GPIOD->CRH&=0XFFF0FFFF; 
	GPIOD->CRH|=0X00030000;   //PD12ÍÆÍìÊä³ö  	 
    GPIOD->ODR|=0X1000;       //PD12ÍÆÍì¸ß											  
	
	LED_L=1;				  
	LED_R=1;
	BEEP=1;
	
	beep=0;
		
}

void ab_c()
{
					int b;
				//***************************¿ªÊ¼×¼±¸
	      Control_LED_Nixie_light(1);
	      delay_ms(100);
				liticheku(1);
	      delay_ms(100);
			  Control_Sluice_gate();
				delay_ms(100);                    
				Full_Tracking(30);
				Go(30,28);
				delay_ms(1000); 
				
        //***************************°ÚÕı³µÉí				
				Full_Right(83);	
				delay_ms(1000);
				Car_Back(30,43);
				delay_ms(1000);
				Full_Tracking(15);               
				Go(30,10);
				delay_ms(1000);
				
				//****************************½øÈëµØ±ê
			  Full_Tracking(15);
			  delay_ms(1000);                  
				Go(30,80);
				delay_ms(2500);                   
				Full_Tracking(15);
				Go(50,25);
				delay_ms(2000);                   
				Full_Tracking(30);                
				Go(50,30);
				
			  //*****************************µÆ¹â»»µ²		
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
				
				//******************************ÓïÒô²¥±¨×ó×ªÍä
				Full_Left(83);	                  
				delay_ms(1000); 
				Full_Tracking(30);
				Go(30,28);
				delay_ms(1000);
				
				//******************************Á¢ÌåÏÔÊ¾	
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
				
				//*******************************±¨¾¯Ì¨
				Full_Right(83);
				delay_ms(500);
				Full_RL_45(83,2,30);
				Transmition(HW_K,6);
				
				//*******************************Ê»³öÂ·¿Ú
				delay_ms(500);
				Full_Left(83);
				delay_ms(500);
				Full_Tracking(30);
				Go(30,20);
				delay_ms(1000);
				Full_Right(83);
				delay_ms(500);
				
				//*******************************ËíµÀ
				Full_Tracking(30);
				Transmition(H_SD,4);
				Go(30,25);				
				delay_ms(500);
				//*******************************ĞĞÊ»ÖÁ³µ¿âÃÅ¿Ú
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
				
				//********************************Èë¿â³äµç
				liticheku(4);
				delay_ms(2000);
				Control_Magnetic_suspension();
				delay_ms(100);
				Control_LED_Nixie_light(2);
}