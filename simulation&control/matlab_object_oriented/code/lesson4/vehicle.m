classdef vehicle < handle
    properties
        make
    end
    properties(SetAccess=protected)
        car_miles=0;
    end
    properties(Hidden)
        year
    end
    properties(Dependent)
        car_km
    end

    methods(Static)
        function miles=km_to_miles(km)
            miles=km/1.6;
        end
    end

    methods
        function obj = vehicle(year,make)
            if nargin<1
                year=2020;
            end
            if nargin<2
                make='yiqi';
            end
            obj.make = make;
            obj.year = year;
        end

        function describe(obj)
            fprintf('%s made in %d.\n',obj.make,obj.year);
        end

        function run(obj,miles)
            obj.car_miles = obj.car_miles+miles;
        end

        function car_km=get.car_km(obj)
            car_km=obj.car_miles*1.6;
        end

%         function disp(obj)
%             obj.describe;
%             fprintf('Its cumulative mileage is %f miles.\n',obj.car_miles);
%         end
    end
end