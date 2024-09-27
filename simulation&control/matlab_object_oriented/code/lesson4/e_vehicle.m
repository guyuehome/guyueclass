classdef e_vehicle < vehicle    %电动车
    properties
        soc
    end
    properties(Constant)
        soc_per_mile=1;
    end

    methods
        function obj = e_vehicle(soc)
            if nargin<1
                soc=95;
            end
            obj.soc = soc;
        end

        function run(obj,miles)
            run@vehicle(obj,miles);
            obj.soc=obj.soc-miles*obj.soc_per_mile;
        end
    end
end