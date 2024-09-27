function plotTL(TL,ax)
% 本脚本用于画红绿灯
for i=1:length(TL)
    % 左转灯
    plot(TL(i).left.pos(1), TL(i).left.pos(2), 'Marker','o',...
        'MarkerFaceColor',TL(i).left.state,'MarkerSize',8, 'Parent',ax);
    
    % 直行灯
    plot(TL(i).stright.pos(1), TL(i).stright.pos(2), 'Marker','o',...
        'MarkerFaceColor',TL(i).stright.state,'MarkerSize',8, 'Parent',ax);
    
    % 右转灯
    plot(TL(i).right.pos(1), TL(i).right.pos(2), 'Marker','o',...
        'MarkerFaceColor',TL(i).right.state,'MarkerSize',8, 'Parent',ax);
end


