function dM = control_parameter_guyueju(State,t,dt,i)
global des_pos
dM          = zeros(18,1);
if t<=5
    des_pos(1,i)    = pi/6*t/5;
end
if t>5&&t<=10
    des_pos(2,i)    = -pi/2+pi/6*(t-5)/5;
    des_pos(3,i)    = -pi/3*(t-5)/5;
end
if t>10&&t<=15
    des_pos(2,i)    = -pi/2+pi/6;
    des_pos(3,i)    = -pi/3;
    des_pos(5,i)    = -pi/2*(t-10)/5;
end
if t>15&&t<=20
    des_pos(2,i)    = -pi/2+pi/6;
    des_pos(3,i)    = -pi/3;
    des_pos(5,i)    = -pi/2;
    des_pos(4,i)    = pi/3*(t-15)/5;
end
if t>20
    des_pos(2,i)    = -pi/2+pi/6;
    des_pos(3,i)    = -pi/3;
    des_pos(5,i)    = -pi/2;
    des_pos(4,i)    = pi/3;
    des_pos(6,i)    = pi/2*(t-20);
end

dM(1:3,1)       = PID_arm(25,0,15,State(5,i),des_pos(1,i),1,dt);
dM(4:6,1)       = PID_arm(25,0,25,State(16,i),des_pos(2,i),2,dt);
dM(7:9,1)       = PID_arm(25,0,5,State(28,i)-State(16,i),des_pos(3,i),3,dt);
dM(10:12,1)     = PID_arm(25,0,5,State(42,i),des_pos(4,i),4,dt);
dM(13:15,1)     = PID_arm(25,0,2,State(52,i)-State(40,i),des_pos(5,i),5,dt);
dM(16:18,1)     = PID_arm(1.5,0,0,State(66,i),des_pos(6,i),6,dt);
end