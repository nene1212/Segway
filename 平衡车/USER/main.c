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
/*            ********�������********            */
/*                                                */  
/*        PC6->PWM1���     PC7->PWM2���         */
/*        PC8->PWM1���     PC9->PWM2���         */
/*                                                */  
/*          ******MPU6050/9250********            */
/*                                                */  
/*        PC15->SDA      	PC14->SCL	          */
/*                                                */
/**************************************************/
/**************************************************/


float pitch,roll,yaw; 		//ŷ����
int acc;
int main(void)
 {		
	unsigned int freq,moto;
	moto = 100;							//������pwm(100%)
	freq = 100000/moto-1;				//pwmת��Ϊfreq
	
	delay_init();	    	 			//��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 				//���ڳ�ʼ��Ϊ115200
 	LED_Init();							//��ʼ��LED
	IIC_Init();							//��ʼ��IIC
	MPU9250_Init();						//��ʼ��������
	PID_init();							//��ʼ��pid����
	LED0 = 0;
	delay_ms(10000);
	while(mpu_dmp_init()){				//��ʼ��������dmp
		delay_ms(200);
		LED0 = !LED0;
		delay_ms(200);
	}   
		LED0 = 1;

	//72000000/720 = 100000
	//100000/x = freq
	
	TIM4_PWM_Init(freq,720); 			//���õ�����pwm
	TIM3_PWM_Init(99,7199);				//���ò�����(��ʱ�жϽ���pid����)
		
   	while(1);
	 
 }

void TIM3_IRQHandler(void)   //TIM3�ж�
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
		{
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //���TIMx�����жϱ�־ 
			while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){
				delay_ms(2);
			}
			acc = PID_realize(roll);					//����pid����pwm ����accֵ���ڴ�ӡ�۲�
			LED0 = !LED0;
				 
		}
}

