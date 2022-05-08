classdef e_vehicle < vehicle
    properties
        soc %子对象特有的属性，剩余电量
    end
    properties(Constant)        %常量属性
        soc_per_mile=1;     %每英里的耗电量
    end

    methods
        function obj = e_vehicle(soc,year,make)
            obj=obj@vehicle(year,make);     %调用父类的构造函数
            obj.soc = soc;
        end

        function run(obj,miles) %重写父类的方法
            run@vehicle(obj,miles);
            obj.soc=obj.soc-miles*obj.soc_per_mile;
        end
    end
end