classdef gamer < handle
    properties
        x   %表示该游戏者已经走过了x块玻璃
        choice  %表示该游戏者做出选择，1表示选择左边，2表示选择右边
        fall   %该玩家掉落的位置（若大于玻璃数量则完成）
    end

    methods
        function obj = gamer
            obj.x = 0;
        end

        function choose(obj)
            obj.choice=randi(2);
        end

        function glass=go(obj,glass)
            for i=1:glass.m
                if glass.walked(i)==1
                    obj.x=obj.x+1;
                else
                    glass.walked(i)=1;
                    obj.choose;
                    if obj.choice==glass.which(i)
                        obj.x=obj.x+1;
                    else
                        break
                    end
                end
            end
            obj.fall=obj.x+1;
        end
    end
end