function dM = PID_arm(kp,ki,kd,alpha,des_alpha,flag,dt)
global error
delta_alpha = des_alpha-alpha;
switch flag
    case 1%输入的转角
        dM = [0;0;kp*delta_alpha+ki*error.int_arm1+kd*(delta_alpha-error.last_arm1)/dt];
        error.int_arm1      = error.int_arm1+delta_alpha;
        error.last_arm1     = delta_alpha;
    case 2%输入的是夹角
        dM = [0;kp*delta_alpha+ki*error.int_arm2+kd*(delta_alpha-error.last_arm2)/dt;0];
        error.int_arm2      = error.int_arm2+delta_alpha;
        error.last_arm2     = delta_alpha;
    case 3%输入的是夹角
        dM = [0;kp*delta_alpha+ki*error.int_arm3+kd*(delta_alpha-error.last_arm3)/dt;0];
        error.int_arm3      = error.int_arm3+delta_alpha;
        error.last_arm3     = delta_alpha;
    case 4%输入的转角
        dM = [kp*delta_alpha+ki*error.int_arm4+kd*(delta_alpha-error.last_arm4)/dt;0;0];
        error.int_arm4      = error.int_arm4+delta_alpha;
        error.last_arm4     = delta_alpha;
    case 5%输入的是夹角
        dM = [0;kp*delta_alpha+ki*error.int_arm5+kd*(delta_alpha-error.last_arm5)/dt;0];
        error.int_arm5      = error.int_arm5+delta_alpha;
        error.last_arm5     = delta_alpha;
    case 6%输入的转角
        dM = [kp*delta_alpha+ki*error.int_arm6+kd*(delta_alpha-error.last_arm6)/dt;0;0];
        error.int_arm6      = error.int_arm6+delta_alpha;
        error.last_arm6     = delta_alpha;
end
end