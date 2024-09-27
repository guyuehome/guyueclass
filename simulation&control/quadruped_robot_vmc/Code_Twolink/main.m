clear
clc
disp('Program started');
vrep = remApi('remoteApi'); 
vrep.simxFinish(-1); 
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);  %如果返回值为-1 则代表通讯不成功,返回值为0代表通讯成功
disp(clientID);
%% 参数初始化

last_t = 0;

FR.Pos_last.x = 0.05;FR.Pos_last.z = 0.3;

RR.Pos_last.x = 0.05;RR.Pos_last.z = 0.3;

FL.Pos_last.x = 0.05;FL.Pos_last.z = 0.3;

RL.Pos_last.x = 0.05;RL.Pos_last.z = 0.3;

FR.time=0;FR.xf=0;FR.x0=0;FR.z0=0.3;FR.State = 2;
RL.time=0;RL.xf=0;RL.x0=0;RL.z0=0.3;RL.State = 2;
FL.time=0;FL.xf=0;FL.x0=0;FL.z0=0.3;FL.State = 2;
RR.time=0;RR.xf=0;RR.x0=0;RR.z0=0.3;RR.State = 2;

flag=1;
frequency=0;
%%
if (clientID>-1)
    disp('Connected to remote API server');
    Robot = CoppeliaSim(clientID,vrep);
    res=vrep.simxSynchronous(clientID,true); %设置为同步模式
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking); %开启仿真
    inittime = vrep.simxGetLastCmdTime(clientID);   %获取进入主循环时的仿真时间，初始时间
     
    while true
        currentCmdTime = vrep.simxGetLastCmdTime(clientID);%获取当前仿真时间
        t = (currentCmdTime-inittime)/1000; %转换为秒
        %获取8个关节的角度
        FR.th1 = Robot.GetJointPosition("FR_thigh_joint",vrep);
        FR.th2 = Robot.GetJointPosition("FR_calf_joint",vrep);
        RR.th1 = Robot.GetJointPosition("RR_thigh_joint",vrep);
        RR.th2 = Robot.GetJointPosition("RR_calf_joint",vrep);
        RL.th1 = Robot.GetJointPosition("RL_thigh_joint",vrep);
        RL.th2 = Robot.GetJointPosition("RL_calf_joint",vrep);
        FL.th1 = Robot.GetJointPosition("FL_thigh_joint",vrep);
        FL.th2 = Robot.GetJointPosition("FL_calf_joint",vrep);
        %正运动学计算相对位置
        FR.Pos_current = Robot.Positive_kinematics(FR.th1,FR.th2);
        FL.Pos_current = Robot.Positive_kinematics(FL.th1,FL.th2);
        RR.Pos_current = Robot.Positive_kinematics(RR.th1,RR.th2);
        RL.Pos_current = Robot.Positive_kinematics(RL.th1,RL.th2);
        %获取仿真步长
        time_step=t-last_t;
        %获取机器人x方向和z方向运动速度
        [Robot.rec, Robot.Current_pozition] = vrep.simxGetObjectPosition(clientID,Robot.body,-1,vrep.simx_opmode_oneshot);
        Robot.Current_vx =-(Robot.Current_pozition(1)-Robot.Last_pozition(1))/time_step;
        Robot.Current_vz =-(Robot.Current_pozition(3)-Robot.Last_pozition(3))/time_step;
        %%  初始稳定状态，四足全触地 持续100个周期
         
        if(frequency<100)
                %初始站立状态
                FR.State=Robot.Support_phase;
                FR.Pos_target.z=Robot.Target_z;
               % FR.Pos_target.x=0;
                FR.V_target.z=0;
                FR.V_target.x=0;
        
                RL.State=Robot.Support_phase;
                RL.Pos_target.z=Robot.Target_z;
               % RL.Pos_target.x=0;
                RL.V_target.z=0;
                RL.V_target.x=0;
        
                FL.State=Robot.Support_phase;
                FL.Pos_target.z=Robot.Target_z;
               % FL.Pos_target.x=0;
                FL.V_target.z=0;
                FL.V_target.x=0;
                
                RR.State=Robot.Support_phase;
                RR.Pos_target.z=Robot.Target_z;
                %RR.Pos_target.x=0;
                RR.V_target.z=0;
                RR.V_target.x=0;
                %设置机器人当前状态为初始状态
                Robot.Current_State = 5;
        end
         %%  获取足端位置，检测是否接触地面
        
        [Robot.rec,FR_Foot_Position]=vrep.simxGetObjectPosition(clientID,Robot.FR_foot,-1,vrep.simx_opmode_oneshot);
        [Robot.rec,RL_Foot_Position]=vrep.simxGetObjectPosition(clientID,Robot.RL_foot,-1,vrep.simx_opmode_oneshot);
        [Robot.rec,RR_Foot_Position]=vrep.simxGetObjectPosition(clientID,Robot.RR_foot,-1,vrep.simx_opmode_oneshot);
        [Robot.rec,FL_Foot_Position]=vrep.simxGetObjectPosition(clientID,Robot.FL_foot,-1,vrep.simx_opmode_oneshot);
        
        %判断每条腿的触地状态
            if(RR_Foot_Position(3)<0.025)
                RR_Foot_touch = 1;
            else
                RR_Foot_touch=0;
            end
            if(RL_Foot_Position(3)<0.025)
                RL_Foot_touch = 1;
            else
                RL_Foot_touch=0;
            end
            if(FL_Foot_Position(3)<0.025)
                FL_Foot_touch = 1;
            else
                FL_Foot_touch=0;
            end
            if(FR_Foot_Position(3)<0.025)
                FR_Foot_touch = 1;
            else
                FR_Foot_touch=0;
            end
 %%
            if(frequency>=100)
                 % FR RL 抬腿A组  RR FL 抬腿B组
                if(RR_Foot_touch==1&&RL_Foot_touch==1&&FL_Foot_touch==1&&FR_Foot_touch==1&&(Robot.Last_State==5||Robot.Last_State==3))
                    %设置机器人当前状态为 S=0;
                    if(flag==1)
                        Robot.Current_State=0;
    
                        %FR,RL设为摆动相
                        FR.State = Robot.Wobble_phase;
                        RL.State = Robot.Wobble_phase;
                        %每条腿的时间清零
                        FR.time = 0;
                        RL.time = 0;
                        %计算摆动相起始x位置
                        FR.x0 = FR.Pos_current.x;
                        RL.x0 = RL.Pos_current.x;
                        %计算摆动相末端x位置
                        FR.xf = -0.5*Robot.Current_vx*Robot.T_sw-Robot.K_vx*(Robot.Target_vx-Robot.Current_vx);
                        RL.xf = -0.5*Robot.Current_vx*Robot.T_sw-Robot.K_vx*(Robot.Target_vx-Robot.Current_vx);
                        %计算摆动相初始z位置
                        FR.z0 = FR.Pos_current.z;
                        RL.z0 = RL.Pos_current.z;
    
                        %RR,FL设为支撑相
                        RR.State = Robot.Support_phase;
                        FL.State = Robot.Support_phase;
                        %每条腿的时间清零
                        RR.time = 0;
                        FL.time = 0;
                        %给定支撑相x方向速度
                        RR.V_target.x=Robot.Target_vx;
                        FL.V_target.x=Robot.Target_vx;
                        %给定支撑相z方向速度
                        RR.V_target.z=Robot.Target_vz;
                        FL.V_target.z=Robot.Target_vz;
                        %给定支撑相z方向位置
                        RR.Pos_target.z=Robot.Target_z;
                        FL.Pos_target.z=Robot.Target_z;
                        flag=2;
                    end
                end
                if(RR_Foot_touch==1&&FL_Foot_touch==1&&(RL_Foot_touch+FR_Foot_touch)==1&&Robot.Last_State==0&&FR.time>0.95*Robot.T_sw)
                     if(flag==2)
                        Robot.Current_State=1;
                        %判断哪条腿落地
                        if(RL_Foot_touch==1) 

                            %RL设为支撑相
                            RL.State = Robot.Support_phase;
                            %RL单腿时间清零
                            RL.time = 0;
                            %给定支撑相x方向速度
                            RL.V_target.x=Robot.Target_vx;
                            %给定支撑相z方向速度
                            RL.V_target.z=Robot.Target_vz;
                            %给定支撑相z方向位置
                            RL.Pos_target.z=Robot.Target_z;
                            
                            %FR仍为摆动相，不做处理
                        else
                            %FR设为支撑相
                            FR.State = Robot.Support_phase;
                            %FR单腿时间清零
                            FR.time = 0;
                            %给定支撑相x方向速度
                            FR.V_target.x=Robot.Target_vx;
                            %给定支撑相z方向速度
                            FR.V_target.z=Robot.Target_vz;
                            %给定支撑相z方向位置
                            FR.Pos_target.z=Robot.Target_z;
                            %RL仍为摆动相，不做处理
                        end
                            flag=3;
                     end

                end
                if(RR_Foot_touch==1&&RL_Foot_touch==1&&FL_Foot_touch==1&&FR_Foot_touch==1&&Robot.Last_State==1)
                    if(flag==3)
                        Robot.Current_State=2;
                        %RR,FL设为摆动相
                        RR.State = Robot.Wobble_phase;
                        FL.State = Robot.Wobble_phase;
                        %每条腿的时间清零
                        RR.time = 0;
                        FL.time = 0;
                        %计算摆动相起始x位置
                        RR.x0 = FR.Pos_current.x;
                        FL.x0 = RL.Pos_current.x;
                        %计算摆动相末端x位置
                        RR.xf = -0.5*Robot.Current_vx*Robot.T_sw-Robot.K_vx*(Robot.Target_vx-Robot.Current_vx);
                        FL.xf = -0.5*Robot.Current_vx*Robot.T_sw-Robot.K_vx*(Robot.Target_vx-Robot.Current_vx);
                        %计算摆动相初始z位置
                        RR.z0 = FR.Pos_current.z;
                        FL.z0 = RL.Pos_current.z;
                        %FR,RL设为支撑相
                        FR.State = Robot.Support_phase;
                        RL.State = Robot.Support_phase;
                        %每条腿的时间清零
                        FR.time = 0;
                        RL.time = 0;
                        %给定支撑相x方向速度
                        FR.V_target.x=Robot.Target_vx;
                        RL.V_target.x=Robot.Target_vx;
                        %给定支撑相z方向速度
                        FR.V_target.z=Robot.Target_vz;
                        RL.V_target.z=Robot.Target_vz;
                        %给定支撑相z方向位置
                        FR.Pos_target.z=Robot.Target_z;
                        RL.Pos_target.z=Robot.Target_z;

                        flag=4;
                    end
                end
                if(FR_Foot_touch==1&&RL_Foot_touch==1&&(FL_Foot_touch+RR_Foot_touch)==1&&Robot.Last_State==2&&RR.time>0.95*Robot.T_sw)
                    if(flag==4)
                        Robot.Current_State=3;
                        
                        if(FL_Foot_touch==1)
                            %FL设为支撑相
                            FL.State = Robot.Support_phase;
                            %FL单腿时间清零
                            FL.time = 0;
                            %给定支撑相x方向速度
                            FL.V_target.x=Robot.Target_vx;
                            %给定支撑相z方向速度
                            FL.V_target.z=Robot.Target_vz;
                            %给定支撑相z方向位置
                            FL.Pos_target.z=Robot.Target_z;
                        else
                            %RR设为支撑相
                            RR.State = Robot.Support_phase;
                            %FR单腿时间清零
                            RR.time = 0;
                            %给定支撑相x方向速度
                            RR.V_target.x=Robot.Target_vx;
                            %给定支撑相z方向速度
                            RR.V_target.z=Robot.Target_vz;
                            %给定支撑相z方向位置
                            RR.Pos_target.z=Robot.Target_z;
                            %FL仍为摆动相，不做处理
                        end
                        flag=1;
                    end
                end
                switch Robot.Current_State
                    case 0
                                [FR.Pos_target.x,FR.Pos_target.z]=Robot.Wobble_phase_trajectory(FR);
                                [RL.Pos_target.x,RL.Pos_target.z]=Robot.Wobble_phase_trajectory(RL);
                    case 1  
                            if(RL_Foot_touch==1)
                                [FR.Pos_target.x,FR.Pos_target.z]=Robot.Wobble_phase_trajectory(FR);
                            else
                                [RL.Pos_target.x,RL.Pos_target.z]=Robot.Wobble_phase_trajectory(RL);
                            end
                    case 2
                                [RR.Pos_target.x,RR.Pos_target.z]=Robot.Wobble_phase_trajectory(RR);
                                [FL.Pos_target.x,FL.Pos_target.z]=Robot.Wobble_phase_trajectory(FL); 
                    case 3  
                            if(FL_Foot_touch==1)
                                [RR.Pos_target.x,RR.Pos_target.z]=Robot.Wobble_phase_trajectory(RR);
                            else
                                [FL.Pos_target.x,FL.Pos_target.z]=Robot.Wobble_phase_trajectory(FL); 
                            end
                end
            end
            %% DEBUG

            %fprintf('Robot Current_State: %d\n',Robot.Current_State);


%%      计算关节力矩，设置关节力矩

        RR.tao = Robot.Force_PD_Calculate(time_step,RR);
        FL.tao = Robot.Force_PD_Calculate(time_step,FL);
        FR.tao = Robot.Force_PD_Calculate(time_step,FR);
        RL.tao = Robot.Force_PD_Calculate(time_step,RL);
           
        if(frequency>2)
%             Robot.rec = vrep.simxPauseCommunication(clientID,true);
            Robot.SetJointTorque("FR_thigh_joint",FR.tao(1),vrep);
            Robot.SetJointTorque("FR_calf_joint",FR.tao(2),vrep);
            Robot.SetJointTorque("FL_thigh_joint",FL.tao(1),vrep);
            Robot.SetJointTorque("FL_calf_joint",FL.tao(2),vrep);
            Robot.SetJointTorque("RR_thigh_joint",RR.tao(1),vrep);
            Robot.SetJointTorque("RR_calf_joint",RR.tao(2),vrep);
            Robot.SetJointTorque("RL_thigh_joint",RL.tao(1),vrep);
            Robot.SetJointTorque("RL_calf_joint",RL.tao(2),vrep);
%             Robot.rec = vrep.simxPauseCommunication(clientID,false);
        end
    
%%      状态更新
        
        last_t = t;  %时间更新
        %单腿相对位置更新
        FR.Pos_last.x = FR.Pos_current.x;
        FR.Pos_last.z = FR.Pos_current.z;
        RR.Pos_last.x = RR.Pos_current.x;
        RR.Pos_last.z = RR.Pos_current.z;
        RL.Pos_last.x = RL.Pos_current.x;
        RL.Pos_last.z = RL.Pos_current.z;
        FL.Pos_last.x = FL.Pos_current.x;
        FL.Pos_last.z = FL.Pos_current.z;
        %机器人本体三维坐标更新
        Robot.Last_pozition = Robot.Current_pozition;
        %每条腿的计时更新
        FR.time = FR.time+time_step;
        RL.time = RL.time+time_step;
        RR.time = RR.time+time_step;
        FL.time = FL.time+time_step;
        %机器人状态标志位更新
        Robot.Last_State=Robot.Current_State;
        frequency=frequency+1;
        vrep.simxSynchronousTrigger(clientID); %开启同步仿真
    end
end
