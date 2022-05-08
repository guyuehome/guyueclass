classdef wave_obj < handle & matlab.mixin.Heterogeneous
    methods
        function obj = wave_obj
        end
    end

    methods(Abstract)   %抽象类，提供一个规范
        T_cal(obj);
        draw(obj);
    end
end