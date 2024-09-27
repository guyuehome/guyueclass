classdef vehicle < handle
    properties
        make
    end
    properties(SetAccess=protected) %受保护的属性，不能在外部修改
        car_miles=0;
    end
    properties(Hidden)  %隐藏的属性
        year
    end
    properties(Dependent)   %非独立属性
        car_km
    end

    methods(Static) %静态方法
        function miles=km_to_miles(km)
            miles=km/1.6;
        end
    end

    methods
        function obj = vehicle(year,make)
            if nargin<1     %缺省参数
                year=2020;
            end
            if nargin<2
                make='yiqi';
            end
            obj.make = make;
            obj.year = year;
        end

        function describe(obj)  %车辆信息的描述
            fprintf('%s made in %d.\n',obj.make,obj.year);
        end

        function run(obj,miles)     %行驶一定的里程
            obj.car_miles = obj.car_miles+miles;
        end

        function car_km=get.car_km(obj)     %非独立属性的获取
            car_km=obj.car_miles*1.6;
        end

%         function disp(obj)    %重载对象的显示方式
%             obj.describe;
%             fprintf('Its cumulative mileage is %f miles.\n',obj.car_miles);
%         end
    end
end