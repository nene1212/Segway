#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "mpu9250.h"
#include "myiic.h"
#include "pid.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
/****************PIN DEFINE************************/
/**************************************************/
/**************************************************/
/*            ********电机驱动********            */
/*                                                */  
/*        PC6->PWM1输出     PC7->PWM2输出         */
/*        PC8->PWM1输出     PC9->PWM2输出         */
/*                                                */  
/*          ******MPU6050/9250********            */
/*                                                */  
/*        PC15->SDA      	PC14->SCL	          */
/*                                                */
/**************************************************/
/**************************************************/


float pitch,roll,yaw; 		//欧拉角
int acc;
int main(void)
 {		
	unsigned int freq,moto;
	moto = 100;							//电机输出pwm(100%)
	freq = 100000/moto-1;				//pwm转换为freq
	
	delay_init();	    	 			//延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 				//串口初始化为115200
 	LED_Init();							//初始化LED
	IIC_Init();							//初始化IIC
	MPU9250_Init();						//初始化陀螺仪
	PID_init();							//初始化pid参数
	LED0 = 0;
	delay_ms(10000);
	while(mpu_dmp_init()){				//初始化陀螺仪dmp
		delay_ms(200);
		LED0 = !LED0;
		delay_ms(200);
	}   
		LED0 = 1;

	//72000000/720 = 100000
	//100000/x = freq
	
	TIM4_PWM_Init(freq,720); 			//设置电机输出pwm
	TIM3_PWM_Init(99,7199);				//设置采样率(定时中断进行pid调节)
		
   	while(1);
	 
 }

void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
		{
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx更新中断标志 
			while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){
				delay_ms(2);
			}
			acc = PID_realize(roll);					//根据pid调节pwm 返回acc值便于打印观察
			LED0 = !LED0;
				 
		}
}

