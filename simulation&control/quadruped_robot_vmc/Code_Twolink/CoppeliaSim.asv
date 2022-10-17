classdef CoppeliaSim <handle

    properties    
        RR_hip_joint; RR_thigh_joint; RR_calf_joint;
        RL_hip_joint; RL_thigh_joint; RL_calf_joint;
        FL_hip_joint; FL_thigh_joint; FL_calf_joint;
        FR_hip_joint; FR_thigh_joint; FR_calf_joint;
        RR_foot;FR_foot;FL_foot;RL_foot;
        RR_time=0;FR_time=0;FL_time=0;RL_time=0;
        body;
        client_ID;
        rec;
        %机器人当前运动速度
        Current_vx=0;
        Current_vz=0;
        %机器人当前和上一次位置
        Current_pozition=[0 0 0.22];
        Last_pozition=[0 0 0.22];
        %机器人期望移动速度
        Target_vx=0.12;
        Target_x=0;
        Target_z=0.32;
        Target_vz=0;
        %机器人当前和上一次状态
        Current_State = 0;
        Last_State=5;
        %机器人步态周期
        T_sw=0.1;  %摆动相
        T_st=0.1;  %支撑相
        %
    end

    properties(Constant) %所用到的常数设置
        Support_phase=1; %支撑相
        Wobble_phase=0;  %摆动相
        PI = 3.1415;
        hu=0.2115;
        hl=0.2309;
        %vmc 支撑相PD参数
        D_x=280;   
        K_z=800; 
        D_z=90;

        %摆动相落脚点速度增益
        K_vx=0.1;
    end

    methods
        function obj = CoppeliaSim(client_ID,vrep)
            %初始化，获取关节句柄
            obj.client_ID= client_ID;
            [obj.rec ,obj.RR_hip_joint]=vrep.simxGetObjectHandle (obj.client_ID,'RR_hip_joint',vrep.simx_opmode_blocking);
            [obj.rec ,obj.RR_thigh_joint]=vrep.simxGetObjectHandle (obj.client_ID,'RR_thigh_joint',vrep.simx_opmode_blocking);
            [obj.rec ,obj.RR_calf_joint]=vrep.simxGetObjectHandle (obj.client_ID,'RR_calf_joint',vrep.simx_opmode_blocking);
            [obj.rec ,obj.RL_hip_joint]=vrep.simxGetObjectHandle (obj.client_ID,'RL_hip_joint',vrep.simx_opmode_blocking);
            [obj.rec ,obj.RL_thigh_joint]=vrep.simxGetObjectHandle (obj.client_ID,'RL_thigh_joint',vrep.simx_opmode_blocking);
            [obj.rec ,obj.RL_calf_joint]=vrep.simxGetObjectHandle (obj.client_ID,'RL_calf_joint',vrep.simx_opmode_blocking);
            [obj.rec ,obj.FL_hip_joint]=vrep.simxGetObjectHandle (obj.client_ID,'FL_hip_joint',vrep.simx_opmode_blocking);
            [obj.rec ,obj.FL_thigh_joint]=vrep.simxGetObjectHandle (obj.client_ID,'FL_thigh_joint',vrep.simx_opmode_blocking);
            [obj.rec ,obj.FL_calf_joint]=vrep.simxGetObjectHandle (obj.client_ID,'FL_calf_joint',vrep.simx_opmode_blocking);
            [obj.rec ,obj.FR_hip_joint]=vrep.simxGetObjectHandle (obj.client_ID,'FR_hip_joint',vrep.simx_opmode_blocking);
            [obj.rec ,obj.FR_thigh_joint]=vrep.simxGetObjectHandle (obj.client_ID,'FR_thigh_joint',vrep.simx_opmode_blocking);
            [obj.rec ,obj.FR_calf_joint]=vrep.simxGetObjectHandle (obj.client_ID,'FR_calf_joint',vrep.simx_opmode_blocking);
            %初始化，获取足端句柄,用于触地信号判断
            [obj.rec ,obj.RR_foot]=vrep.simxGetObjectHandle (obj.client_ID,'RR_foot_respondable',vrep.simx_opmode_blocking);
            [obj.rec ,obj.FR_foot]=vrep.simxGetObjectHandle (obj.client_ID,'FR_foot_respondable',vrep.simx_opmode_blocking);
            [obj.rec ,obj.FL_foot]=vrep.simxGetObjectHandle (obj.client_ID,'FL_foot_respondable',vrep.simx_opmode_blocking);
            [obj.rec ,obj.RL_foot]=vrep.simxGetObjectHandle (obj.client_ID,'RL_foot_respondable',vrep.simx_opmode_blocking);
            %身体句柄获取，用于得到躯干速度
            [obj.rec ,obj.body]=vrep.simxGetObjectHandle (obj.client_ID,'mini_cheetah',vrep.simx_opmode_blocking);

        end

        function SetJointTorque(obj,joint_name,torque,vrep)
            %设置关节力矩
            Joint_handle=Object_name_tran(obj,joint_name);
            
            if torque<0
                set_vel = -99999;
                set_tau= -torque;
            else
                set_vel = 99999;
                set_tau = torque;
            end
            if(set_tau>20)
                set_vel = 0;
            end
             vrep.simxSetJointForce(obj.client_ID,Joint_handle,set_tau,vrep.simx_opmode_oneshot);
             vrep.simxSetJointTargetVelocity(obj.client_ID,Joint_handle,set_vel,vrep.simx_opmode_oneshot);
        end

         function Pos=GetJointPosition(obj,joint_name,vrep)
            %获取关节角度
            Joint_handle=Object_name_tran(obj,joint_name);
            [obj.rec,Pos]=vrep.simxGetJointPosition(obj.client_ID,Joint_handle,vrep.simx_opmode_oneshot);    
         end

        function torque=GetJointTorque(obj,joint_name,vrep)
            %获取关节力矩
            Joint_handle=Object_name_tran(obj,joint_name);
            [obj.rec,torque]=vrep.simxGetJointForce(obj.client_ID,Joint_handle,vrep.simx_opmode_oneshot);
        end
        %摆动相轨迹规划
        function [x,z]=Wobble_phase_trajectory(obj,leg)
            h=-0.1; %抬腿高度设为0.1m
            if(leg.time<obj.T_sw)
                sigma=2*pi*leg.time/obj.T_sw;
                x=(leg.xf-leg.x0)*((sigma-sin(sigma))/(2*pi))+leg.x0;
                z=h*(1-cos(sigma))/2+leg.z0;
            else
                x=leg.xf;
                z=leg.z0;
            end
        end

         %两自由度正运动学
        function Pos = Positive_kinematics(obj,th1,th2)
                 Pos.x=obj.hu*sin(th1)+obj.hl*sin(th1+th2);
                 Pos.z=obj.hu*cos(th1)+obj.hl*cos(th1+th2);

        end
        function Torque = Force_PD_Calculate(obj,time_step,leg)
            %摆动相力计算
                 if(leg.State==obj.Wobble_phase)                            
                      vx=(leg.Pos_current.x-leg.Pos_last.x)/(time_step);
                      vz=(leg.Pos_current.z-leg.Pos_last.z)/(time_step);
                      Force.fx=380*(leg.Pos_target.x - leg.Pos_current.x)+18*(leg.V_target.x-vx);
                      Force.fz=380*(leg.Pos_target.z - leg.Pos_current.z)+18*(leg.V_target.z-vz);
                 end
            %支撑相力计算
                  if(leg.State==obj.Support_phase)                            
                      
                      vz=(leg.Pos_current.z-leg.Pos_last.z)/time_step;
                      vx=-obj.Current_vx;

                      Force.fx=obj.D_x*(leg.V_target.x-vx);
                      Force.fz=obj.K_z*(leg.Pos_target.z - leg.Pos_current.z)+obj.D_z*(leg.V_target.z-vz);
                  end
                  %力矩 雅可比 转换
                 Torque(1) = (obj.hl*cos(leg.th1 + leg.th2) + obj.hu*cos(leg.th1))*Force.fx+(- obj.hl*sin(leg.th1 + leg.th2) - obj.hu*sin(leg.th1))*Force.fz;
                 Torque(2) =  obj.hl*cos(leg.th1 + leg.th2)*Force.fx+(-obj.hl*sin(leg.th1 + leg.th2))*Force.fz;

        end

        function Object_handle=Object_name_tran(obj,Object_name)
            %转换句柄
                 switch Object_name
                     case "RR_hip_joint"
                         Object_handle = obj.RR_hip_joint;
                     case "RL_hip_joint"
                         Object_handle = obj.RL_hip_joint;
                     case "RR_thigh_joint"
                         Object_handle = obj.RR_thigh_joint;
                     case "RL_thigh_joint"
                         Object_handle = obj.RL_thigh_joint;
                     case "RR_calf_joint"
                         Object_handle = obj.RR_calf_joint;
                     case "RL_calf_joint"
                         Object_handle = obj.RL_calf_joint;
                     case "FL_hip_joint"
                         Object_handle = obj.FL_hip_joint;
                     case "FR_hip_joint"
                         Object_handle = obj.FR_hip_joint;
                     case "FL_thigh_joint"
                         Object_handle = obj.FL_thigh_joint;
                     case "FR_thigh_joint"
                         Object_handle = obj.FR_thigh_joint;
                     case "FL_calf_joint"
                         Object_handle = obj.FL_calf_joint;
                     case "FR_calf_joint"
                         Object_handle = obj.FR_calf_joint;
                     case "RR_foot_respondable"
                         Object_handle = obj.RR_foot;
                     case "RL_foot_respondable"
                         Object_handle = obj.RL_foot;
                      case "FR_foot_respondable"
                         Object_handle = obj.FR_foot;
                     case "FL_foot_respondable"
                         Object_handle = obj.FL_foot;  
                     case "mini_cheetah"
                         Object_handle = obj.body; 
                 end

        end

    end
end