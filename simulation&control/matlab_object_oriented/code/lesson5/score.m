classdef score < matlab.mixin.Copyable
    properties
        data    %输入数据
        name    %学生名字
        m1  %均值
        m2  %中位数
    end

    methods
        function obj = score(data,name)
            if nargin<1
                data=0;
            end
            if nargin<2
                name='anonymity';
            end
            obj.data = data;
            obj.name=name;
            obj.mean_cal;
            obj.median_cal;
        end

        function mean_cal(obj)
            obj.m1 = mean(obj.data);
        end
        function median_cal(obj)
            obj.m2 = median(obj.data);
        end
    end
end