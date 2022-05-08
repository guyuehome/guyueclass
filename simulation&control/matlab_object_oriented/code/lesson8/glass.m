classdef glass
    properties
        m
        which
        walked
    end

    methods
        function obj = glass(m)
            obj.m = m;
            %生成n个1或2的随机数，1表示左边为安全的玻璃，2则表示右边
            obj.which=randi(2,[1,obj.m]);
            %该序列表示每一块玻璃是否被走过，初始置0表示未被走过
            obj.walked=zeros(1,obj.m);
        end
    end
end