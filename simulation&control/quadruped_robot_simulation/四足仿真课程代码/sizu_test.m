%通讯初始化
clear
clc
disp('Program started');
vrep=remApi('remoteApi'); 
vrep.simxFinish(-1); 
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);  %如果返回值为-1 则代表通讯不成功,返回值为0代表通讯成功
disp(clientID);
%%
if (clientID>-1)
   disp('Connected to remote API server');
   vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot); %开启仿真
   %声明节点，关节初始化，12个关节
 [rec ,rb_rot_3]=vrep.simxGetObjectHandle (clientID,'rb_rot_3',vrep.simx_opmode_blocking);
 [rec ,rf_rot_3]=vrep.simxGetObjectHandle (clientID,'rf_rot_3',vrep.simx_opmode_blocking);
 [rec ,rb_rot_2]=vrep.simxGetObjectHandle (clientID,'rb_rot_2',vrep.simx_opmode_blocking);
 [rec ,rf_rot_2]=vrep.simxGetObjectHandle (clientID,'rf_rot_2',vrep.simx_opmode_blocking);
 [rec ,rb_rot_1]=vrep.simxGetObjectHandle (clientID,'rb_rot_1',vrep.simx_opmode_blocking);
 [rec ,rf_rot_1]=vrep.simxGetObjectHandle (clientID,'rf_rot_1',vrep.simx_opmode_blocking);
 [rec ,lb_rot_3]=vrep.simxGetObjectHandle (clientID,'lb_rot_3',vrep.simx_opmode_blocking);
 [rec ,lf_rot_3]=vrep.simxGetObjectHandle (clientID,'lf_rot_3',vrep.simx_opmode_blocking);
 [rec ,lb_rot_2]=vrep.simxGetObjectHandle (clientID,'lb_rot_2',vrep.simx_opmode_blocking);
 [rec ,lf_rot_2]=vrep.simxGetObjectHandle (clientID,'lf_rot_2',vrep.simx_opmode_blocking);
 [rec ,lb_rot_1]=vrep.simxGetObjectHandle (clientID,'lb_rot_1',vrep.simx_opmode_blocking);
 [rec ,lf_rot_1]=vrep.simxGetObjectHandle (clientID,'lf_rot_1',vrep.simx_opmode_blocking);
 %12个电机力矩参数
 rb_rot_1_force=500;     rb_rot_2_force=500;         rb_rot_3_force=500; %第一条腿
 rf_rot_1_force=500;     rf_rot_2_force=500;         rf_rot_3_force=500; %第二条腿
 lb_rot_1_force=500;     lb_rot_2_force=500;         lb_rot_3_force=500; %第三条腿   
 lf_rot_1_force=500;     lf_rot_2_force=500;         lf_rot_3_force=500; %第四条腿
 %12个电机角度参数
 rb_rot_1_pos=0;  rb_rot_2_pos=0;  rb_rot_3_pos=0;  %第一条腿 
 rf_rot_1_pos=0;  rf_rot_2_pos=0;  rf_rot_3_pos=0;  %第二条腿
 lb_rot_1_pos=0;  lb_rot_2_pos=0;  lb_rot_3_pos=0;  %第三条腿
 lf_rot_1_pos=0;  lf_rot_2_pos=0;  lf_rot_3_pos=0;  %第四条腿
  %设置电机力矩
 rec=vrep.simxSetJointForce(clientID,rb_rot_3, rb_rot_3_force,vrep.simx_opmode_blocking);
 rec=vrep.simxSetJointForce(clientID,rf_rot_3, rf_rot_3_force,vrep.simx_opmode_blocking);
 rec=vrep.simxSetJointForce(clientID,rb_rot_2, rb_rot_2_force,vrep.simx_opmode_blocking);
 rec=vrep.simxSetJointForce(clientID,rf_rot_2, rf_rot_2_force,vrep.simx_opmode_blocking);
 rec=vrep.simxSetJointForce(clientID,rb_rot_1, rb_rot_1_force,vrep.simx_opmode_blocking);
 rec=vrep.simxSetJointForce(clientID,rf_rot_1, rf_rot_1_force,vrep.simx_opmode_blocking);
 rec=vrep.simxSetJointForce(clientID,lb_rot_3, lb_rot_3_force,vrep.simx_opmode_blocking);
 rec=vrep.simxSetJointForce(clientID,lf_rot_3, lf_rot_3_force,vrep.simx_opmode_blocking);
 rec=vrep.simxSetJointForce(clientID,lb_rot_2, lb_rot_2_force,vrep.simx_opmode_blocking);
 rec=vrep.simxSetJointForce(clientID,lf_rot_2, lf_rot_2_force,vrep.simx_opmode_blocking);
 rec=vrep.simxSetJointForce(clientID,lb_rot_1, lb_rot_1_force,vrep.simx_opmode_blocking);
 rec=vrep.simxSetJointForce(clientID,lf_rot_1, lf_rot_1_force,vrep.simx_opmode_blocking);
 
%    row=0;   pitch=0; yaw=0;
%    pos_x=0; pos_y=0.2; pos_z=-0.2;
 pause(1);   %延时1s
 t=clock;   %获取matlab系统当前时间 
 startTime=t(5)*60+t(6); %当前时间 [年 月 日 时 分 秒]
 currentTime=0; %当前时间
 gait_state=2;  %步态标志位
 %  [lb_x,lb_y,lb_z,rb_x,rb_y,rb_z,rf_x,rf_y,rf_z,lf_x,lf_y,lf_z] = pose_control(row,pitch,yaw,pos_x,pos_y,pos_z);
%  [lb_rot_1_pos,lb_rot_2_pos,lb_rot_3_pos]=xyz(lb_x,lb_y,lb_z);
%  [lf_rot_1_pos,lf_rot_2_pos,lf_rot_3_pos]=xyz(lf_x,lf_y,lf_z);
%  [rb_rot_1_pos,rb_rot_2_pos,rb_rot_3_pos]=xyz(rb_x,rb_y,rb_z);
%  [rf_rot_1_pos,rf_rot_2_pos,rf_rot_3_pos]=xyz(rf_x,rf_y,rf_z);
%% 
while (currentTime < 100)
 t=clock; 
 currentTime=t(5)*60+t(6)-startTime; %matlab当前时间，从进入主循环开始 
  if(currentTime < 5) 
   if(gait_state==2)   %walk步态
   lb_x= -0.1;    rb_x= -0.1;    lf_x=0.1;     rf_x=0.1;
   lb_z=-0.482;   rf_z=-0.482;   lf_z=-0.482;    rb_z=-0.482;
   end
   if(gait_state==1)   %trot步态
   lb_x= -0.1;    rb_x= 0.1;    lf_x=0.1;      rf_x=-0.1;
   lb_z=-0.482;    rf_z=-0.482;   lf_z=-0.482;    rb_z=-0.482;
   end
     [rec,vrep_time]=vrep.simxGetFloatSignal(clientID,'time',vrep.simx_opmode_oneshot); %获取仿真时间
  else
     [rec,vrep_realtime]=vrep.simxGetFloatSignal(clientID,'time',vrep.simx_opmode_oneshot); %获取仿真时间
      switch gait_state
               case 2 %慢步步态
                T=1;
                time1=vrep_realtime-vrep_time;          
                time2=vrep_realtime-vrep_time+0.25;    
                time3=vrep_realtime-vrep_time+0.5;       
                time4=vrep_realtime-vrep_time+0.75;      
                T1=mod(time1,T); T2=mod(time2,T);   %mod 函数为取模运算
                T3=mod(time3,T); T4=mod(time4,T);
                [lb_x,lb_z]=gait_plan2(T1,T);
                [rf_x,rf_z]=gait_plan2(T2,T);
                [rb_x,rb_z]=gait_plan2(T3,T);
                [lf_x,lf_z]=gait_plan2(T4,T); 
               case 1  %对角快步步态
                T=0.4;      %步态周期s
                time1=vrep_realtime-vrep_time;       
                T1=mod(time1,T);
                time2=vrep_realtime-vrep_time+0.2;      
                T2=mod(time2,T);
                [lb_x,lb_z]=gait_plan1(T1,T);
                [rf_x,rf_z]=gait_plan1(T1,T);
                [rb_x,rb_z]=gait_plan1(T2,T);
                [lf_x,lf_z]=gait_plan1(T2,T); 
      end
  end
 %单腿逆运动学，Y值设定为默认值
 [lb_rot_1_pos,lb_rot_2_pos,lb_rot_3_pos]=xyz(lb_x,-0.15,lb_z);                    
 [lf_rot_1_pos,lf_rot_2_pos,lf_rot_3_pos]=xyz(lf_x,-0.15,lf_z);
 [rb_rot_1_pos,rb_rot_2_pos,rb_rot_3_pos]=xyz(rb_x,-0.15,rb_z);
 [rf_rot_1_pos,rf_rot_2_pos,rf_rot_3_pos]=xyz(rf_x,-0.15,rf_z);
 %电机控制函数
 rec=vrep.simxSetJointTargetPosition(clientID,lb_rot_1,-lb_rot_1_pos,vrep.simx_opmode_oneshot);
 rec=vrep.simxSetJointTargetPosition(clientID,lb_rot_2,lb_rot_2_pos,vrep.simx_opmode_oneshot);
 rec=vrep.simxSetJointTargetPosition(clientID,lb_rot_3,lb_rot_3_pos,vrep.simx_opmode_oneshot);
 rec=vrep.simxSetJointTargetPosition(clientID,rf_rot_1,rf_rot_1_pos,vrep.simx_opmode_oneshot);
 rec=vrep.simxSetJointTargetPosition(clientID,rf_rot_2,rf_rot_2_pos,vrep.simx_opmode_oneshot);
 rec=vrep.simxSetJointTargetPosition(clientID,rf_rot_3,rf_rot_3_pos,vrep.simx_opmode_oneshot);
 rec=vrep.simxSetJointTargetPosition(clientID,rb_rot_1,-rb_rot_1_pos,vrep.simx_opmode_oneshot);
 rec=vrep.simxSetJointTargetPosition(clientID,rb_rot_2,rb_rot_2_pos,vrep.simx_opmode_oneshot);
 rec=vrep.simxSetJointTargetPosition(clientID,rb_rot_3,rb_rot_3_pos,vrep.simx_opmode_oneshot);
 rec=vrep.simxSetJointTargetPosition(clientID,lf_rot_1,lf_rot_1_pos,vrep.simx_opmode_oneshot);
 rec=vrep.simxSetJointTargetPosition(clientID,lf_rot_2,lf_rot_2_pos,vrep.simx_opmode_oneshot);
 rec=vrep.simxSetJointTargetPosition(clientID,lf_rot_3,lf_rot_3_pos,vrep.simx_opmode_oneshot);

end
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking); %仿真停止
        vrep.simxFinish(clientID);
else
        disp('Failed connecting to remote API server');
end
    vrep.delete(); 
    disp('Program ended');
