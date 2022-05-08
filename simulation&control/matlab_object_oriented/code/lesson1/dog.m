classdef dog %实体类
    properties
        name    %小狗的名字
        age     %小狗的年龄
    end

    methods
        function obj = dog(name)
            obj.name = name;
        end

        function sit(obj)   %命令小狗坐下
            fprintf('%s is now sitting.\n',obj.name);
        end

        function how_old(obj)   %询问小狗的年龄
            if isempty(obj.age)
                fprintf('I do not know my age.\n')
            else
                fprintf('%d years old.\n',obj.age);
            end
        end
    end
end