classdef square < wave_obj
    properties
        amp %幅值
        fre %频率
        duty_cycle  %占空比
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

%         function si=sine(obj)
%             si=sine(obj.amp,obj.fre);
%         end
    end
end