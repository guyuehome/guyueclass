#include "pid.h"

float kp=50,ki=3,kd=5;
int PID_control(int speed_fact,int speed_sure)
{
	static int pwm_out,speed_err,speed_sum,speed_err_last;
	speed_err=speed_fact-speed_sure;
	speed_sum+=speed_err;//Çó»ı·Ö
	pwm_out=kp*speed_err+ki*speed_sum+kd*(speed_err-speed_err_last);
	speed_err_last=speed_err;
	return pwm_out;
}
