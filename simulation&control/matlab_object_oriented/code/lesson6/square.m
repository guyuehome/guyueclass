classdef square < wave_obj
    properties
        amp
        fre
        duty_cycle
        T
    end

    methods
        function obj = square(amp,fre,duty_cycle)
            if nargin<1
                amp=1;
            end
            if nargin<2
                fre=1;
            end
            if nargin<3
                duty_cycle=0.5;
            end
            obj.amp = amp;
            obj.fre = fre;
            obj.duty_cycle = duty_cycle;
        end

        function T_cal(obj) %计算周期
            obj.T=1/obj.fre;
        end

        function draw(obj)  %绘制图像
            fplot(@(x) obj.amp*((mod(x,obj.T)<obj.duty_cycle*obj.T)- ...
                (mod(x,obj.T)>obj.duty_cycle*obj.T)));
        end

%         function si=sine(obj)
%             si=sine(obj.amp,obj.fre);
%         end
    end
end