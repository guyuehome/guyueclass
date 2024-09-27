classdef truck < vehicle    %货车
    properties
        loadage %载货量
    end

    methods
        function obj = truck(loadage)
            if nargin<1
                loadage=100;
            end
            obj.loadage = loadage;
        end
    end
end