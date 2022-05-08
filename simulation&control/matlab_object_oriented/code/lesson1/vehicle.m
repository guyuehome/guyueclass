classdef vehicle < handle %句柄类
    properties
        make        %车辆的制造商
        year        %车辆的生产年份
        car_miles   %车辆的累计里程（英里）
    end

    methods
        function obj = vehicle(year,make)
            obj.make = make;
            obj.year = year;
        end

        function describe(obj)  %车辆信息的描述
            fprintf('%s made in %d.\n',obj.make,obj.year);
        end

        function run(obj,miles) %行驶一定的里程
            obj.car_miles = obj.car_miles+miles;
        end
    end
end