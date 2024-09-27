classdef e_vehicle_truck< e_vehicle & truck %电动货车（多重继承）
    methods
        function run(obj,miles)
            run@vehicle(obj,miles);
            obj.soc=obj.soc-miles*obj.soc_per_mile* ...
                (1+obj.loadage*0.01);
        end
    end
end