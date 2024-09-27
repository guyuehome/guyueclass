classdef sine < wave_obj
    properties
        amp
        fre
        T
    end

    methods(Static)
        function obj=loadobj(s)
            obj=sine(s.amp,s.fre);
        end
    end

    methods
        function obj = sine(amp,fre)
            if nargin<1
                amp=1;
            end
            if nargin<2
                fre=1;
            end
            obj.amp = amp;
            obj.fre = fre;
        end

        function T_cal(obj) %计算周期
            obj.T=1/obj.fre;
        end

        function draw(obj)  %绘制图像
            fplot(@(x) obj.amp*sin(2*pi*obj.fre*x));
        end

        function s=saveobj(obj)
            s.amp=obj.amp;
            s.fre=obj.fre;
        end
    end
end