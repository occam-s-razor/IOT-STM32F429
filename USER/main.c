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
//START 任务
//设置任务优先级
#define START_TASK_PRIO			10  //开始任务的优先级为最低
//设置任务堆栈大小
#define START_STK_SIZE			128
//任务任务堆栈
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);

//串口扫描任务
//设置任务优先级
#define USART_TASK_PRIO			8
//设置任务堆栈大小
#define USART_STK_SIZE			128
//任务堆栈
OS_STK USART_TASK_STK[USART_STK_SIZE];
//任务函数
void usart_task(void *pdata);

//按键扫描任务
//设置任务优先级
#define KEY_TASK_PRIO       	7
//设置任务堆栈大小
#define KEY_STK_SIZE  	        128
//创建任务堆栈空间	
OS_STK KEY_TASK_STK[KEY_STK_SIZE];
//任务函数接口
void key_task(void *pdata);

//LED任务
//设置任务优先级
#define LED_TASK_PRIO			6
//设置任务堆栈大小
#define LED_STK_SIZE			128
//任务堆栈
OS_STK LED_TASK_STK[LED_STK_SIZE];
//任务函数
void led_task(void *pdata);

//蜂鸣器任务
//设置任务优先级
#define BEEP_TASK_PRIO          5 
//设置任务堆栈大小
#define BEEP_STK_SIZE  	        128
//创建任务堆栈空间	
OS_STK BEEP_TASK_STK[BEEP_STK_SIZE];
//任务函数接口
void beep_task(void *pdata);

//主任务
//设置任务优先级
#define MAIN_TASK_PRIO          4 
//设置任务堆栈大小
#define MAIN_STK_SIZE  	        128
//任务堆栈	
OS_STK MAIN_TASK_STK[MAIN_STK_SIZE];
//任务函数
void main_task(void *pdata);


//传感器读取任务
//设置任务优先级
#define SENSOR_TASK_PRIO			3
//设置任务堆栈大小
#define SENSOR_STK_SIZE			512
//任务堆栈
OS_STK SENSOR_TASK_STK[SENSOR_STK_SIZE];
//任务函数
void sensor_task(void *pdata);

//////////////////////////////////////////////////////////////////////////////
OS_EVENT * msg_key;			//按键邮箱事件块指针
OS_EVENT * sem_beep;		//蜂鸣器信号量指针	
OS_EVENT * sem_led;		//LED信号量指针
//全局变量
SENSOR  sensor;//传感器控制结构体

int main(void)
{
    HAL_Init();                     //初始化HAL库   
    Stm32_Clock_Init(360,25,2,8);   //设置时钟,180Mhz
    delay_init(180);                //初始化延时函数
    uart_init(115200);              //初始化USART
    LED_Init();                     //初始化LED
    KEY_Init();                     //初始化按键
    PCF8574_Init();                 //初始化PCF8574 
	  OSInit();                       //UCOS初始化
	  init_sensor(&sensor);                  //初始化传感器状态
	  TIM3_PWM_Init(500-1,90-1);      //90M/90=1M的计数频率，自动重装载为500，那么PWM频率为1M/500=2kHZ
	  TIM_SetTIM3Compare4(0x0000);	//关风扇
	  PCF8574_ReadBit(BEEP_IO);       
		while(DHT11_Init())	//DHT11初始化	
		{
			delay_ms(200);
		}		
	  while(AP3216C_Init())		//检测不到AP3216C
		{		
			delay_ms(500);	
		}	
    OSTaskCreateExt((void(*)(void*) )start_task,                //任务函数
                    (void*          )0,                         //传递给任务函数的参数
                    (OS_STK*        )&START_TASK_STK[START_STK_SIZE-1],//任务堆栈栈顶
                    (INT8U          )START_TASK_PRIO,           //任务优先级
                    (INT16U         )START_TASK_PRIO,           //任务ID，这里设置为和优先级一样
                    (OS_STK*        )&START_TASK_STK[0],        //任务堆栈栈底
                    (INT32U         )START_STK_SIZE,            //任务堆栈大小
                    (void*          )0,                         //用户补充的存储区
                    (INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);//任务选项,为了保险起见，所有任务都保存浮点寄存器的值
	OSStart(); //开始任务
}
///////////////////////////////////////////////////////////////////////////////////////////////////
//开始任务
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	pdata=pdata;
  msg_key=OSMboxCreate((void*)0);	//创建消息邮箱
	sem_beep=OSSemCreate(0);		//创建信号量	
	sem_led=OSSemCreate(0);		//创建信号量	
	OSStatInit();  //开启统计任务
	OS_ENTER_CRITICAL();  //进入临界区(关闭中断)
	//LED任务
	OSTaskCreateExt((void(*)(void*) )led_task,                 
									(void*          )0,
									(OS_STK*        )&LED_TASK_STK[LED_STK_SIZE-1],
									(INT8U          )LED_TASK_PRIO,            
									(INT16U         )LED_TASK_PRIO,            
									(OS_STK*        )&LED_TASK_STK[0],         
									(INT32U         )LED_STK_SIZE,             
									(void*          )0,                         
									(INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);
	//蜂鸣器任务
	OSTaskCreateExt((void(*)(void*) )sensor_task,                 
									(void*          )0,
									(OS_STK*        )&SENSOR_TASK_STK[SENSOR_STK_SIZE-1],
									(INT8U          )SENSOR_TASK_PRIO,          
									(INT16U         )SENSOR_TASK_PRIO,            
									(OS_STK*        )&SENSOR_TASK_STK[0],         
									(INT32U         )SENSOR_STK_SIZE,            
									(void*          )0, 
									(INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);											
	//蜂鸣器任务
	OSTaskCreateExt((void(*)(void*) )beep_task,                 
									(void*          )0,
									(OS_STK*        )&BEEP_TASK_STK[BEEP_STK_SIZE-1],
									(INT8U          )BEEP_TASK_PRIO,          
									(INT16U         )BEEP_TASK_PRIO,            
									(OS_STK*        )&BEEP_TASK_STK[0],         
									(INT32U         )BEEP_STK_SIZE,            
									(void*          )0,                           
									(INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);
	 //串口扫描任务
		OSTaskCreateExt((void(*)(void*) )usart_task,                 
										(void*          )0,
										(OS_STK*        )&USART_TASK_STK[USART_STK_SIZE-1],
										(INT8U          )USART_TASK_PRIO,          
										(INT16U         )USART_TASK_PRIO,            
										(OS_STK*        )&USART_TASK_STK[0],         
										(INT32U         )USART_STK_SIZE,            
										(void*          )0,                           
										(INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);
		//主任务
		OSTaskCreateExt((void(*)(void*) )main_task,                 
										(void*          )0,
										(OS_STK*        )&MAIN_TASK_STK[MAIN_STK_SIZE-1],
										(INT8U          )MAIN_TASK_PRIO,          
										(INT16U         )MAIN_TASK_PRIO,            
										(OS_STK*        )&MAIN_TASK_STK[0],         
										(INT32U         )MAIN_STK_SIZE,            
										(void*          )0,                           
										(INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);
		//按键任务
		OSTaskCreateExt((void(*)(void*) )key_task,                 
										(void*          )0,
										(OS_STK*        )&KEY_TASK_STK[KEY_STK_SIZE-1],
										(INT8U          )KEY_TASK_PRIO,          
										(INT16U         )KEY_TASK_PRIO,            
										(OS_STK*        )&KEY_TASK_STK[0],         
										(INT32U         )KEY_STK_SIZE,            
										(void*          )0,                           
										(INT16U         )OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR|OS_TASK_OPT_SAVE_FP);
		OS_EXIT_CRITICAL();             //退出临界区(开中断)
	  OSTaskSuspend(START_TASK_PRIO); //挂起开始任务
}
 
//LED任务
void led_task(void *pdata)
{
		u8 err;
		u8 led_state;
		led_state=0;
		while(1)
		{
			OSSemPend(sem_led,0,&err);     //请求信号量   
			if(led_state==0){
				LED1=0;	//LED1开
				led_state=1;
			}	else{
				LED1=1;	//LED1关
				led_state=0;
			}
			delay_ms(10);
		}									 
}   
//蜂鸣器任务
void beep_task(void *pdata)
{
  u8 err;
	while(1)
	{  
        OSSemPend(sem_beep,0,&err);     //请求信号量            
        PCF8574_WriteBit(BEEP_IO,0);    //打开蜂鸣器
        delay_ms(60);
    	  PCF8574_WriteBit(BEEP_IO,1);    //关闭蜂鸣器
        delay_ms(940);
	}									 
}
//主任务
void main_task(void *pdata)
{							 
	u32 key=0;	
	u8 err;						 
	while(1)
	{
		key=(u32)OSMboxPend(msg_key,10,&err);   
		switch(key)
		{
			case 1://控制DS1
				OSSemPost(sem_led);
				break;
			case 2://发送信号量
				OSSemPost(sem_beep);
				break;
		}   		 
		delay_ms(10);
	}
} 
//按键扫描任务
void key_task(void *pdata)
{	
	u8 key;		    						 
	while(1)
	{
		key=KEY_Scan(0);   
		if(key)OSMboxPost(msg_key,(void*)key);//发送消息
		if(key==1){
		//	printf("hello");
		}
 		delay_ms(10);
	}
}
void sensor_task(void *pdata){
	u16 i;
	u8 data[10]; 
  sensor.if_open_sensor=0x0f;//初始关
	sensor.sensor_state=0;//初始关
	data[0]=0xff;
	data[9]=0x00;
	while(1){
	  if(sensor.if_open_sensor==0xf0){//回传数据
			  //ALS&PS
		    AP3216C_ReadData(data);	//读取数据 
			  //DH11
			  PCF8574_ReadBit(BEEP_IO);   //释放PB12            
			  DHT11_Read_Data(data);//读取温湿度值		
			  for(i=0;i<10;i++){
				  while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
	        USART3->DR = (u8) data[i];      			
				} 
			  while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
			/*
				printf("红外:%d\r\n",ir);
				printf("距离:%d\r\n",ps);
				printf("光照强度:%d\r\n",als);
			  printf("温度:%d\r\n",temperature);
		  	printf("湿度：%d\r\n",humidity);*/
			 
		}else if(sensor.if_open_sensor==0x0f){//挂起本任务
			 
				sensor.sensor_state=0;//状态关			
				OSTaskSuspend(SENSOR_TASK_PRIO); //挂起
		}
		delay_ms(100);
	}
}
//ARM--------->32:
//         命令类型1字节                   +                    命令1字节
//		 开关传感器：0xf0                                      开0xf0;关0x0f;
//		 开关设备：0x0f                                        开关灯：0x11;开关蜂鸣器：0x21; 
//         风扇控制：0xf2                                         角度值	;	 
//串口扫描函任务
void usart_task(void *pdata){
		u16 pwm;
		pwm=0;
		while(1)
		{
			if(USART3_RX_STA&0x8000)//串口接收完成
			{
					if(USART3_RX_BUF[0]==0x0f)//开关设备
					{
						 if(USART3_RX_BUF[1]==0x11)//开关灯
						 {
							 OSSemPost(sem_led);
						 }
						 if(USART3_RX_BUF[1]==0x21)//开关蜂鸣器
						 {
							 OSSemPost(sem_beep);
						 }
					}
					if(USART3_RX_BUF[0]==0xf0)//开关传感器
					{
						 sensor.if_open_sensor=USART3_RX_BUF[1];
						 if(USART3_RX_BUF[1]==0xf0&&sensor.sensor_state==0)
						 { 					
								OSTaskResume(SENSOR_TASK_PRIO);//解挂
								sensor.sensor_state=1;//状态开				 
						 }		   
					}	
					if(USART3_RX_BUF[0]==0xf2)//风扇控制
					{
						 pwm=(u16)USART3_RX_BUF[1];
						 TIM_SetTIM3Compare4(pwm);	//修改比较值，修改占空比		
					}
					USART3_RX_STA=0;
		}//串口接收完成
		delay_ms(20);
	}
}


