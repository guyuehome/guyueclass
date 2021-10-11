classdef TwoLinkModel <handle
    %MATHSCARA 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        robot
        interpolation=0
    end
    
    methods
       function obj=TwoLinkModel()
            
        end
        function init(obj)
            Links(1) = Link( 'a', 1, 'alpha', 0,'d', 0);
            Links(1).offset=1.57; %通过Offset指定初值
            Links(2) = Link( 'a', 1, 'alpha', 0,'d',0);
            obj.robot=SerialLink(Links,'name','robot');
            qinit=[0 0];
            obj.robot.plot(qinit,'workspace',[-2 2 -2 2 -0.5 1],'trail','k','fps',...
            20,'tile1color',[0.6 0.7 0.835],'view',[122 41]);
        end
         function MoveJ(obj,qangle)
             % 一个仍待解决的问题，如何指定复合参数例如 'trail'的参数 opt.LineWidth=1;
             pos=obj.robot.getpos(); %要想使用getpos必须先plot过一次
             qangles=qangle;
             if(obj.interpolation==1)
                 qangles=mtraj(@tpoly,pos,qangle,50);
             elseif(obj.interpolation==2)
                 qangles=mtraj(@lspb,pos,qangle,50);
             end
              obj.robot.plot(qangles,'workspace',[-1 1 -1 1 -0.5 1],'trail','k','fps',...
            20,'tile1color',[0.6 0.7 0.835]);
             % fps 可以调整动画速度
         end
    end
    
end

