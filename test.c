/************************************************************
**  ����: ����7�����ײ����
	�汾: V5.21-15:45
	�޸�: ��
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


#define  ZCKZ_ADDR    0xAA  // ���������־���ַ���
#define  YSBZW_ADDR   0x02  // ���������־���ַ���
#define  DZ_ADDR      0x03  // �����բ��־���ַ���
#define  LEDXS_ADDR   0x04  // ����LED��ʾ��־���ַ���
#define  JXB_ADDR     0x05  // �����е�۱�־���ַ���

void Key_Onclink();
void IO_Init(void); //IO��ʼ��	
void DIY_BEEP(u16 T_Hz,u8 timer);
void ab_c();

u8 G_Tab[6];	   //������ⷢ������
u8 S_Tab[NUM]; 	   //������������������
u8 C_Tab[NUM]; 	   //�����˶���־�ﷵ����������

u8 Stop_Flag=0;    //״̬��־λ
u8 Track_Flag=0;     //ѭ����־λ
u8 G_Flag=0;	   //ǰ����־λ
u8 B_Flag=0;	   //���˱�־λ
u8 L_Flag=0;	   //��ת��־λ
u8 R_Flag=0;	   //��ת��־λ
u8 SD_Flag=1;      //�˶���־�����ݷ��������־λ
u8 Rx_Flag =0;

u16 CodedDisk=0;   //����ֵͳ��
u16 tempMP=0;	   //��������ֵ
u16 MP;			   //��������ֵ
int Car_Spend = 50;//С���ٶ�Ĭ��ֵ
u32 count = 0;	   //��ѭ��ʱ  ��������ѭ����ȫ���Ĵ���  ���ڴ˲����ɷ�ֹѭ��������
int LSpeed;		   //ѭ�������ٶ�
int RSpeed;		   //ѭ�������ٶ�
u8 Line_Flag=0;	   // 
u8 send_Flag=0;	   // ���ͱ�־λ

unsigned Light=0; //���ն�

u16 error_Flag = 0;

// ������

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
	
	NVIC_Configuration(); 	 //����NVIC�жϷ���0:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
		
	
	SYN7318_Init();
	

	IO_Init();                  //IO��ʼ��

	S_Tab[0]=0x55;
	S_Tab[1]=0xaa;

	C_Tab[0]=0x55;
	C_Tab[1]=0x02;
	
	CanP_Init();
	
	Power_Check();  //�������  �ϵ��ȼ��һ�ε���
	Send_Electric( Electric_Buf[0],Electric_Buf[1]);
	

	STOP();
	Host_Close_UpTrack();
	
	Send_Debug_Info("A1B2C3\n",8); // �ϴ�������Ϣ
	
 	Set_Track_Init();			   //����ѭ����ʼ������
	
	//Readcard_daivce_Init();

	while(1)
	{	 
		Key_Onclink();
		while(Full_Circulate_Flag)
		{
			LED1 = 1;
			LED0 = 0;
			Full_Motion();
			error_Flag = 0;				//��ʱ״̬��־λ����
			Wifi_Rx_flag = 0;			//WIFI����״̬��־λ����
			Zigbee_Rx_flag = 0;			//ZigBee����״̬��־λ����
		}
		
		if(ETC_Open_Flag == 1)					
		{
			if(error_Flag == 50 || error_Flag == 100 || 
			   error_Flag == 150 || error_Flag == 200 || error_Flag == 250)		// 5s 10s 15s
			{
				Full_Go(80,3);
			}
			
		}
		
		LED0 = !LED0;					//����״̬	
		delay_ms(100);
		LED1 = 0;
		if(Wifi_Rx_flag ==1)  			// wifi ���ձ��
		{
			  Wifi_Rx_flag =0;
			  if(Wifi_Rx_Buf[0]==0x55)  // ���յ�55��ͷ����
			  {			  	 
			  	     Normal_data(); 	//�������ݴ���
			  }
		}

		if(Rx_Flag ==1)						   //���յ�����ָ��
		{
			if(Wifi_Rx_Buf[1]==ZCKZ_ADDR) 	   //��������
			{
				switch(Wifi_Rx_Buf[2])
				{
					case 0xB1:				   //��ʼָ��
						//Car_Position_news[0] = Wifi_Rx_Buf[3];		//�õ�������ʼλ��
						//Car_Position_news[1] = Wifi_Rx_Buf[4];		//�õ��������λ��
						//Car_Position_news[2] = Wifi_Rx_Buf[5];		//δʹ��
						mark = 5;
						Full_Circulate_Flag = 1;
						break;
				}
	    	Rx_Flag =0;
			}
		} 
		
		if(Zigbee_Rx_flag ==1)	 //zigbee������Ϣ
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[1]==0x0c) && (ETC_Open_Flag == 1)) 			//ETC
			{
			   if(Zigb_Rx_Buf[2]==0x01)
			   {
					if(Zigb_Rx_Buf[3]==0x01)							//ETC��
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
		 if(!KEY0)			  //����K1����=======================================================================================================
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
//				 //Control_LED_show_data(2,SMG_JL);//�����ڶ�����ʾ���룬������Ǻ���ָ��ĵ�ַ����test.h���档u8 X,u8 *Data
//				 Control_LED_Nixie_light(1);	//led��һ�ż�ʱ��������1�ǿ���2����ͣ����zigbee���Ƶģ� 
//				 delay_ms(500);
//				 Control_Sluice_gate();			//��բ_������zigbee�ģ�
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
//						 Go(30,100);				//�����α�־��
//						 delay_ms(3000);  //�ճ����α�־��
//						 Full_Tracking(15);	//ѭ��  ��ͷ��������
//						 Go(30,25);
//					 }
//					 else
//					 {
//						 Full_Tracking(35);			//�������� i = 1 ��ʱ��ִ�У����������ִ��
//						 Go(30,25);	
//					 }				
//					 delay_ms(500);
//					 switch(i)
//					 {
//					  case 1:
//							Transmition(H_SD,4);							//���ͨ��
//							delay_ms(1000);
//					 	 break;
//					  case 2:
//							//delay_ms(500);
//							Transmition(HW_K,6);			//����̨_��
//							delay_ms(1000);	
//					 	 break;
//					  case 3:
//							Light_StepCtr(3);			//����·��_��λ��2
//							delay_ms(1000);			
//					 	 break;
//					 }
//					 Full_Left(80);				//��ת
//					 delay_ms(1000);		
//					 //Full_Tracking(35);					 
//				 }
//				 Full_Tracking(35);	
//				 Go(30,25);					
//				 Control_Sluice_gate();			//��բ_������zigbee�ģ�
//				 delay_ms(1000);
//				 Full_Tracking(35);		//���
//				 Go(30,25);						//����
//				 Control_LED_Nixie_light(2);	//led��һ�ż�ʱ��������1�ǿ���2����ͣ����zigbee���Ƶģ�
//				 
				 
				 
				 
				 
				 
				 
				 
				 
//				 Transmition(H_SD,4);							//���ͨ��
																					//��һ����������ʾtest.h��������ݡ�
//				 Control_LED_show_data(2,SMG_JL);//�����ڶ�����ʾ���룬������Ǻ���ָ��ĵ�ַ����test.h���档u8 X,u8 *Data
//				 Control_LED_Nixie_light(2);	//led��һ�ż�ʱ��������1�ǿ���2����ͣ����zigbee���Ƶģ�
//				 Control_Sluice_gate();			//��բ_������zigbee�ģ�
//					Transmition(H_2,4);				//����·��_��λ��2
//				 Transmition(HW_K,6);			//����̨_��
			 }
		}
	   if(!KEY1)			  //����K2����========================================================================================================
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
//				 //Control_LED_show_data(2,SMG_JL);//�����ڶ�����ʾ���룬������Ǻ���ָ��ĵ�ַ����test.h���档u8 X,u8 *Data
//				 Control_LED_Nixie_light(1);	//led��һ�ż�ʱ��������1�ǿ���2����ͣ����zigbee���Ƶģ� 
//				 delay_ms(500);
//				 Control_Sluice_gate();			//��բ_������zigbee�ģ�
//				 delay_ms(500);
//				 Full_Tracking(45);			//ѭ��ǰ��
//				 Go(30,25);							//����b7·��
//				 delay_ms(1000);
//				 //*************ת�����**************
//				 Right(80);			//��ת
//				 delay_ms(1000);				
//				 Car_Back(40,20);//��ת��ĺ���
//				 delay_ms(1000);
//				 Full_Tracking(35);//ѭ�������α�־��ǰ��
//				 delay_ms(1000);
//				 //************************************
//				 //MP_Tracking(15,30);
////				 for(i=0;i;i++)
////				 {
////				 }
//				 
//				 Go(30,85);				//�����α�־��
//				 delay_ms(3000);  //�ճ����α�־��
//				 Full_Tracking(15);	//ѭ��  ��ͷ��������
//				 Go(30,25);
//				 delay_ms(2000);//�ȴ���ETC
//				 Full_Tracking(35);
//				 Go(30,25);
//				 delay_ms(100);
//				 for(i=0;i<4;i++)		//·��ǰ·��
//				 {
//					 Light_StepCtr(1);//1Ϊ��1������Ϊ�Ӷ���
//					 delay_ms(1000);
//				 }
//				 Full_Left(80);				//��ת(����·�Ƶ�·��ǰ)F6
//				 delay_ms(1000);
//				 Full_Tracking(35);
//				 Go(30,25);
//				 delay_ms(1000);
//				 //*************������������************
//				 SYN_7318_One_test(1,0x06);		// mode ģʽ���� 1 ���ָ�� 0 ָ��ָ��   ����ʶ�
//				 
//				 //*************************************
//				 Full_Left(80);		//��ת��������4
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
	   if(!KEY2)			  //����K3����========================================================================================================
		{
			delay_ms(30);
				beep=1;
				delay_ms(10);
				beep=0;
			if(!KEY2)
			{ 
				Control_LED_Nixie_light(2);

				//send_data_zigbee( YY_ZZW,8);
//SYN_TTS("����ת");
				
//				stereo_reset();//������ʾ
//			  Transmition(H_SD,4);//�����
//				Transmition(HW_K,6);//����̨

					
					
					
					
					
					
					
					
					
					
					
				
				// RL = 1 ��ת RL = 2 ��ת timer �����Ƕ�  ����ֵ timer = 28 ��45��
//				delay_ms(2500);
//				Full_RL_45(45,2,28);
//				Full_Tracking(35);
				
				
				//SYN_7318_One_test(1,0x06);		// mode ģʽ���� 1 ���ָ�� 0 ָ��ָ��   ����ʶ��
				//Control_voice(0x10,0x23);//��������

				
//				Full_Tracking(35);//������һ��ѭ��ǰ�У��ظ�������
//			 Go(30,25);//����ʮ��·���м�λ��
//			 delay_ms(1000);
//			 Full_Left(85);
//				delay_ms(1000);
//				Full_Tracking(35);//������һ��ѭ��ǰ�У��ظ�������
//			 Go(30,25);//����ʮ��·���м�λ��
//			 delay_ms(1000);
//				Full_Right(85);//F6·��
//			 delay_ms(1000);
//				Transmition(H_SD,4);//���ͨ��ϵͳ
//				
//			 Full_Tracking(35);//������һ��ѭ��ǰ�У��ظ�������
//			 Go(30,25);//����ʮ��·���м�λ��
//			 delay_ms(2000);
//			 Full_Tracking(35);//������һ��ѭ��ǰ�У��ظ�������
//			 Go(30,25);//����ʮ��·���м�λ��
//			 delay_ms(1000);
//			 Full_Right(85);//B6·��
//			 delay_ms(1000);
//			 Full_Tracking(45);//������һ��ѭ��ǰ�У��ظ�������
//			 Go(30,25);//����ʮ��·���м�λ��
//			 Transmition(HW_K,6);
//			 delay_ms(3000);
//			 Full_Right(85);//B4·��
//			 delay_ms(1000);
//			 Full_Tracking(45);//������һ��ѭ��ǰ�У��ظ�������
//			 Go(30,25);//����ʮ��·���м�λ��
//			 Full_Tracking(45);//������һ��ѭ��ǰ�У��ظ�������
//			 Go(30,25);//����ʮ��·���м�λ��
//			 delay_ms(1000);
//			 Control_Sluice_gate();//�򿪵�բ--------------------------��բǰ���ӳ�
//			 delay_ms(1000);
//			 Full_Tracking(45);//������һ��ѭ��ǰ�У��ظ�������
//			 Go(30,25);//����ʮ��·���м�λ��
//			 delay_ms(1000);
//			 Full_Right(80);//ת��F4·��
//			 delay_ms(1000);
//			 Full_Tracking(40);//������һ��ѭ��ǰ�У��ظ�������
//			 Go(30,25);//����ʮ��·���м�λ��
//			 delay_ms(1000);
//			 Full_Right(80);//ת��F8·��
//			 delay_ms(1000);
			}
		}
		if(!KEY3)			    //����K4����========================================================================================================
		{
			delay_ms(30);
				beep=1;
				delay_ms(10);
				beep=0;
			if(!KEY3)
			{
				ab_c();

				


//        //�����
				
				
				
				
				
				
				

				
				
				
			
				
				
				
				
				
				
			}
				
				
				
				
				
/****************************************************************
�������ܣ�������������
��	  ����firstΪ��ָ��  secondΪ��ָ�     �u8 first,u8 second
����  ֵ����
*****************************************************************/
        
//				SYN_7318_One_test(1,0x06);  // mode ģʽ���� 1 ���ָ�� 0 ָ��ָ��   ����ʶ��
//				delay_ms(1000);
//				Control_voice(0x10,0x06);//��������
				
				
				
				
/****************************************************************
�������ܣ�LED����ܿ���
��	  ����Mode_num=1 �򿪼�ʱ  Mode_num=2 �رռ�ʱ
����  ֵ����
*****************************************************************/
//        Control_LED_Nixie_light(1);//��ʱ��
//				delay_ms(5000);
//				Control_LED_Nixie_light(0);//��ʱ��
				
				
				
/****************************************************************
�������ܣ�LED�������ʾ����
��	  ����*HW ����ָ���ַ
����  ֵ����
*****************************************************************/
//         Control_LED_show_data(2,u8 *Data);//ʧ�ܣ��ڶ�����ʾ���ֲ�����ʾ
				
				
				
				
				
				//Stereo_Display(1);
//				Transmition(Traffic,6);  //������ʾ  --------------
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
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
** ���ܣ�     ��ʼ�����İ���ʹ�ö˿�
** ������	  �޲���
** ����ֵ��    ��
****************************************************************/
void IO_Init(void)
{
	YJ_INIT();			//��ʼ��Ӳ��

	GPIOB->CRH&=0XFFFFFFF0;	  
	GPIOB->CRH|=0X00000008;//PB8 ���ó�����   			   
	GPIOB->ODR|=1<<8;	   //PB8����

	GPIOC->CRH&=0X000FFFFF; 
	GPIOC->CRH|=0X33300000;   //PC13/PC14/PC15�������  	 
    GPIOC->ODR|=0XE000;       //PC13/PC14/PC15�����
		   	 
	GPIOD->CRH&=0XFFF0FFFF; 
	GPIOD->CRH|=0X00030000;   //PD12�������  	 
    GPIOD->ODR|=0X1000;       //PD12�����											  
	
	LED_L=1;				  
	LED_R=1;
	BEEP=1;
	
	beep=0;
		
}

void ab_c()
{
					int b;
				//***************************��ʼ׼��
	      Control_LED_Nixie_light(1);
	      delay_ms(100);
				liticheku(1);
	      delay_ms(100);
			  Control_Sluice_gate();
				delay_ms(100);                    
				Full_Tracking(30);
				Go(30,28);
				delay_ms(1000); 
				
        //***************************��������				
				Full_Right(83);	
				delay_ms(1000);
				Car_Back(30,43);
				delay_ms(1000);
				Full_Tracking(15);               
				Go(30,10);
				delay_ms(1000);
				
				//****************************����ر�
			  Full_Tracking(15);
			  delay_ms(1000);                  
				Go(30,80);
				delay_ms(2500);                   
				Full_Tracking(15);
				Go(50,25);
				delay_ms(2000);                   
				Full_Tracking(30);                
				Go(50,30);
				
			  //*****************************�ƹ⻻��		
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
				
				//******************************����������ת��
				Full_Left(83);	                  
				delay_ms(1000); 
				Full_Tracking(30);
				Go(30,28);
				delay_ms(1000);
				
				//******************************������ʾ	
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
				
				//*******************************����̨
				Full_Right(83);
				delay_ms(500);
				Full_RL_45(83,2,30);
				Transmition(HW_K,6);
				
				//*******************************ʻ��·��
				delay_ms(500);
				Full_Left(83);
				delay_ms(500);
				Full_Tracking(30);
				Go(30,20);
				delay_ms(1000);
				Full_Right(83);
				delay_ms(500);
				
				//*******************************���
				Full_Tracking(30);
				Transmition(H_SD,4);
				Go(30,25);				
				delay_ms(500);
				//*******************************��ʻ�������ſ�
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
				
				//********************************�����
				liticheku(4);
				delay_ms(2000);
				Control_Magnetic_suspension();
				delay_ms(100);
				Control_LED_Nixie_light(2);
}