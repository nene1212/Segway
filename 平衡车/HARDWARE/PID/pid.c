#include<pid.h>
#include<delay.h>
#include<timer.h>
#include<usart.h>


#define value 18
#define TOP 400
float kp = 9.5;
float ki = 0.01; // 0.00707
float kd = 0.707;

int abs(int a){
	if(a<=0)
		a = -a;
	return a;
}


struct _pid {
	float SetAngle;		//�����趨ֵ
	float ActualAngle;		//����ʵ��ֵ
	float err;		//����ƫ��ֵ
	float err_last;		//������һ��ƫ��ֵ
	float Kp, Ki, Kd;		//������������֡�΢��ϵ��
	float integral;		//�������ֵ
	float acc;			//У׼���ֵ
	float T;			//����
	int pwm;			//�����ѹֵ������ִ�����ı�����
}pid;


void PID_init(void) {
	pid.SetAngle = 0.0;
	pid.ActualAngle = 0;
	pid.err = 0.0;
	pid.err_last = 0.0;
	pid.integral = 0.0;
	pid.T = 0.01;


	pid.Kp = kp;
	pid.Ki = ki/pid.T;    //�����˻���ϵ��
	pid.Kd = kd/pid.T;



	printf("PID_init end \n");
}


int PID_realize(float ActualAngle) {
	float index;
	pid.SetAngle = 0;
	pid.err = pid.SetAngle - ActualAngle;
	if(abs(pid.err)>0.1){
		if (abs(pid.err)>value)		//����ֹ���
		{
			index = 0.0;
		}
		else if (abs(pid.err)<value-value/10) {
			index = 1.0;
			pid.integral += pid.err;
		}
		else {
			index = (value - abs(pid.err)) / value/10;
			pid.integral += pid.err;
		}

		pid.acc = (pid.Kp*pid.err + index*pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last));
		pid.err_last = pid.err;
		pid.pwm = (int)pid.acc;
		printf("%d\r\n",pid.pwm);
		if(pid.pwm>TOP)
			pid.pwm = TOP;
		if(pid.pwm<-TOP)
			pid.pwm = -TOP;
		
		if(pid.pwm >=0){
			pid.pwm+=180;
			TIM_SetCompare1(TIM4,0);
			TIM_SetCompare2(TIM4,pid.pwm);
			pid.pwm-=1;		
			TIM_SetCompare3(TIM4,0);
			TIM_SetCompare4(TIM4,pid.pwm);
		}else{
			pid.pwm = -pid.pwm;
			pid.pwm+=180;
			TIM_SetCompare1(TIM4,pid.pwm);
			TIM_SetCompare2(TIM4,0);
			pid.pwm-=1;		
			TIM_SetCompare3(TIM4,pid.pwm);
			TIM_SetCompare4(TIM4,0);
			
		}
		//printf("acc:%d\r\n",pid.pwm);
}
		else{
			TIM_SetCompare1(TIM4,0);
			TIM_SetCompare2(TIM4,0);
			TIM_SetCompare3(TIM4,0);
			TIM_SetCompare4(TIM4,0);
		}
	return pid.pwm;
}


