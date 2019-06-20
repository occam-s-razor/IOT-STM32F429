#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "pcf8574.h"
#include "key.h"
#include "sdram.h"
#include "touch.h"
#include "lcd.h"
#include "includes.h"
#include "sensor.h"
#include "timer.h"
#include "ap3216c.h"
#include "dht11.h"
//START ����
//�����������ȼ�
#define START_TASK_PRIO			10  //��ʼ��������ȼ�Ϊ���
//���������ջ��С
#define START_STK_SIZE			128
//���������ջ
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata);

//����ɨ������
//�����������ȼ�
#define USART_TASK_PRIO			8
//���������ջ��С
#define USART_STK_SIZE			128
//�����ջ
OS_STK USART_TASK_STK[USART_STK_SIZE];
//������
void usart_task(void *pdata);

//����ɨ������
//�����������ȼ�
#define KEY_TASK_PRIO       	7
//���������ջ��С
#define KEY_STK_SIZE  	        128
//���������ջ�ռ�	
OS_STK KEY_TASK_STK[KEY_STK_SIZE];
//�������ӿ�
void key_task(void *pdata);

//LED����
//�����������ȼ�
#define LED_TASK_PRIO			6
//���������ջ��С
#define LED_STK_SIZE			128
//�����ջ
OS_STK LED_TASK_STK[LED_STK_SIZE];
//������
void led_task(void *pdata);

//����������
//�����������ȼ�
#define BEEP_TASK_PRIO          5 
//���������ջ��С
#define BEEP_STK_SIZE  	        128
//���������ջ�ռ�	
OS_STK BEEP_TASK_STK[BEEP_STK_SIZE];
//�������ӿ�
void beep_task(void *pdata);

//������
//�����������ȼ�
#define MAIN_TASK_PRIO          4 
//���������ջ��С
#define MAIN_STK_SIZE  	        128
//�����ջ	
OS_STK MAIN_TASK_STK[MAIN_STK_SIZE];
//������
void main_task(void *pdata);


//��������ȡ����
//�����������ȼ�
#define SENSOR_TASK_PRIO			3
//���������ջ��С
#define SENSOR_STK_SIZE			512
//�����ջ
OS_STK SENSOR_TASK_STK[SENSOR_STK_SIZE];
//������
void sensor_task(void *pdata);

//////////////////////////////////////////////////////////////////////////////
OS_EVENT * msg_key;			//���������¼���ָ��
OS_EVENT * sem_beep;		//�������ź���ָ��	
OS_EVENT * sem_led;		//LED�ź���ָ��
//ȫ�ֱ���
SENSOR  sensor;//���������ƽṹ��

int main(void)
{
    HAL_Init();                     //��ʼ��HAL��   
    Stm32_Clock_Init(360,25,2,8);   //����ʱ��,180Mhz
    delay_init(180);                //��ʼ����ʱ����
    uart_init(115200);              //��ʼ��USART
    LED_Init();                     //��ʼ��LED
    KEY_Init();                     //��ʼ������
    PCF8574_Init();                 //��ʼ��PCF8574 
	  OSInit();                       //UCOS��ʼ��
	  init_sensor(&sensor);                  //��ʼ��������״̬
	  TIM3_PWM_Init(500-1,90-1);      //90M/90=1M�ļ���Ƶ�ʣ��Զ���װ��Ϊ500����ôPWMƵ��Ϊ1M/500=2kHZ
	  TIM_SetTIM3Compare4(0x0000);	//�ط���
	  PCF8574_ReadBit(BEEP_IO);       
		while(DHT11_Init())	//DHT11��ʼ��	
		{
			delay_ms(200);
		}		
	  while(AP3216C_Init())		//��ⲻ��AP3216C
		{		
			delay_ms(500);	
		}	
    OSTaskCreateExt((void(*)(void*) )start_task,                //������
                    (void*          )0,                         //���ݸ��������Ĳ���
                    (OS_STK*        )&START_TASK_STK[START_STK_SIZE-1],//�����ջջ��
                    (INT8U          )START_TASK_PRIO,           //�������ȼ�
                    (INT16U         )START_TASK_PRIO,           //����ID����������Ϊ�����ȼ�һ��
                    (OS_STK*        )&START_TASK_STK[0],        //�����ջջ��
                    (INT32U         )START_STK_SIZE,            //�����ջ��С
                    (void*          )0,                         //�û�����Ĵ洢��
                    (INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);//����ѡ��,Ϊ�˱���������������񶼱��渡��Ĵ�����ֵ
	OSStart(); //��ʼ����
}
///////////////////////////////////////////////////////////////////////////////////////////////////
//��ʼ����
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	pdata=pdata;
  msg_key=OSMboxCreate((void*)0);	//������Ϣ����
	sem_beep=OSSemCreate(0);		//�����ź���	
	sem_led=OSSemCreate(0);		//�����ź���	
	OSStatInit();  //����ͳ������
	OS_ENTER_CRITICAL();  //�����ٽ���(�ر��ж�)
	//LED����
	OSTaskCreateExt((void(*)(void*) )led_task,                 
									(void*          )0,
									(OS_STK*        )&LED_TASK_STK[LED_STK_SIZE-1],
									(INT8U          )LED_TASK_PRIO,            
									(INT16U         )LED_TASK_PRIO,            
									(OS_STK*        )&LED_TASK_STK[0],         
									(INT32U         )LED_STK_SIZE,             
									(void*          )0,                         
									(INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);
	//����������
	OSTaskCreateExt((void(*)(void*) )sensor_task,                 
									(void*          )0,
									(OS_STK*        )&SENSOR_TASK_STK[SENSOR_STK_SIZE-1],
									(INT8U          )SENSOR_TASK_PRIO,          
									(INT16U         )SENSOR_TASK_PRIO,            
									(OS_STK*        )&SENSOR_TASK_STK[0],         
									(INT32U         )SENSOR_STK_SIZE,            
									(void*          )0, 
									(INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);											
	//����������
	OSTaskCreateExt((void(*)(void*) )beep_task,                 
									(void*          )0,
									(OS_STK*        )&BEEP_TASK_STK[BEEP_STK_SIZE-1],
									(INT8U          )BEEP_TASK_PRIO,          
									(INT16U         )BEEP_TASK_PRIO,            
									(OS_STK*        )&BEEP_TASK_STK[0],         
									(INT32U         )BEEP_STK_SIZE,            
									(void*          )0,                           
									(INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);
	 //����ɨ������
		OSTaskCreateExt((void(*)(void*) )usart_task,                 
										(void*          )0,
										(OS_STK*        )&USART_TASK_STK[USART_STK_SIZE-1],
										(INT8U          )USART_TASK_PRIO,          
										(INT16U         )USART_TASK_PRIO,            
										(OS_STK*        )&USART_TASK_STK[0],         
										(INT32U         )USART_STK_SIZE,            
										(void*          )0,                           
										(INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);
		//������
		OSTaskCreateExt((void(*)(void*) )main_task,                 
										(void*          )0,
										(OS_STK*        )&MAIN_TASK_STK[MAIN_STK_SIZE-1],
										(INT8U          )MAIN_TASK_PRIO,          
										(INT16U         )MAIN_TASK_PRIO,            
										(OS_STK*        )&MAIN_TASK_STK[0],         
										(INT32U         )MAIN_STK_SIZE,            
										(void*          )0,                           
										(INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);
		//��������
		OSTaskCreateExt((void(*)(void*) )key_task,                 
										(void*          )0,
										(OS_STK*        )&KEY_TASK_STK[KEY_STK_SIZE-1],
										(INT8U          )KEY_TASK_PRIO,          
										(INT16U         )KEY_TASK_PRIO,            
										(OS_STK*        )&KEY_TASK_STK[0],         
										(INT32U         )KEY_STK_SIZE,            
										(void*          )0,                           
										(INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);
		OS_EXIT_CRITICAL();             //�˳��ٽ���(���ж�)
	  OSTaskSuspend(START_TASK_PRIO); //����ʼ����
}
 
//LED����
void led_task(void *pdata)
{
		u8 err;
		u8 led_state;
		led_state=0;
		while(1)
		{
			OSSemPend(sem_led,0,&err);     //�����ź���   
			if(led_state==0){
				LED1=0;	//LED1��
				led_state=1;
			}	else{
				LED1=1;	//LED1��
				led_state=0;
			}
			delay_ms(10);
		}									 
}   
//����������
void beep_task(void *pdata)
{
  u8 err;
	while(1)
	{  
        OSSemPend(sem_beep,0,&err);     //�����ź���            
        PCF8574_WriteBit(BEEP_IO,0);    //�򿪷�����
        delay_ms(60);
    	  PCF8574_WriteBit(BEEP_IO,1);    //�رշ�����
        delay_ms(940);
	}									 
}
//������
void main_task(void *pdata)
{							 
	u32 key=0;	
	u8 err;						 
	while(1)
	{
		key=(u32)OSMboxPend(msg_key,10,&err);   
		switch(key)
		{
			case 1://����DS1
				OSSemPost(sem_led);
				break;
			case 2://�����ź���
				OSSemPost(sem_beep);
				break;
		}   		 
		delay_ms(10);
	}
} 
//����ɨ������
void key_task(void *pdata)
{	
	u8 key;		    						 
	while(1)
	{
		key=KEY_Scan(0);   
		if(key)OSMboxPost(msg_key,(void*)key);//������Ϣ
		if(key==1){
		//	printf("hello");
		}
 		delay_ms(10);
	}
}
void sensor_task(void *pdata){
	u16 i;
	u8 data[10]; 
  sensor.if_open_sensor=0x0f;//��ʼ��
	sensor.sensor_state=0;//��ʼ��
	data[0]=0xff;
	data[9]=0x00;
	while(1){
	  if(sensor.if_open_sensor==0xf0){//�ش�����
			  //ALS&PS
		    AP3216C_ReadData(data);	//��ȡ���� 
			  //DH11
			  PCF8574_ReadBit(BEEP_IO);   //�ͷ�PB12            
			  DHT11_Read_Data(data);//��ȡ��ʪ��ֵ		
			  for(i=0;i<10;i++){
				  while((USART3->SR&0X40)==0);//ѭ������,ֱ���������   
	        USART3->DR = (u8) data[i];      			
				} 
			  while((USART3->SR&0X40)==0);//ѭ������,ֱ���������   
			/*
				printf("����:%d\r\n",ir);
				printf("����:%d\r\n",ps);
				printf("����ǿ��:%d\r\n",als);
			  printf("�¶�:%d\r\n",temperature);
		  	printf("ʪ�ȣ�%d\r\n",humidity);*/
			 
		}else if(sensor.if_open_sensor==0x0f){//��������
			 
				sensor.sensor_state=0;//״̬��			
				OSTaskSuspend(SENSOR_TASK_PRIO); //����
		}
		delay_ms(100);
	}
}
//ARM--------->32:
//         ��������1�ֽ�                   +                    ����1�ֽ�
//		 ���ش�������0xf0                                      ��0xf0;��0x0f;
//		 �����豸��0x0f                                        ���صƣ�0x11;���ط�������0x21; 
//         ���ȿ��ƣ�0xf2                                         �Ƕ�ֵ	;	 
//����ɨ�躯����
void usart_task(void *pdata){
		u16 pwm;
		pwm=0;
		while(1)
		{
			if(USART3_RX_STA&0x8000)//���ڽ������
			{
					if(USART3_RX_BUF[0]==0x0f)//�����豸
					{
						 if(USART3_RX_BUF[1]==0x11)//���ص�
						 {
							 OSSemPost(sem_led);
						 }
						 if(USART3_RX_BUF[1]==0x21)//���ط�����
						 {
							 OSSemPost(sem_beep);
						 }
					}
					if(USART3_RX_BUF[0]==0xf0)//���ش�����
					{
						 sensor.if_open_sensor=USART3_RX_BUF[1];
						 if(USART3_RX_BUF[1]==0xf0&&sensor.sensor_state==0)
						 { 					
								OSTaskResume(SENSOR_TASK_PRIO);//���
								sensor.sensor_state=1;//״̬��				 
						 }		   
					}	
					if(USART3_RX_BUF[0]==0xf2)//���ȿ���
					{
						 pwm=(u16)USART3_RX_BUF[1];
						 TIM_SetTIM3Compare4(pwm);	//�޸ıȽ�ֵ���޸�ռ�ձ�		
					}
					USART3_RX_STA=0;
		}//���ڽ������
		delay_ms(20);
	}
}


