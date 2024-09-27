classdef sine < wave_obj
    properties
        amp %幅值
        fre %频率
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
    end
end