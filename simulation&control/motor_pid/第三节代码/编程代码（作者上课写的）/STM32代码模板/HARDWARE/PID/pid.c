#include "pid.h"

//pout=kp*err+ki*err_sum+kd(err-err_last)
float kp=70,ki=15,kd=0;

int PID_control(int act,int set)
{
	static int pwm_out,speed_err,err_sum,speed_err_last;
	speed_err=act-set;//Æ«²î
	err_sum+=speed_err;
	pwm_out=kp*speed_err+ki*err_sum+kd*(speed_err-speed_err_last);
	speed_err_last=speed_err;
	return pwm_out;
}