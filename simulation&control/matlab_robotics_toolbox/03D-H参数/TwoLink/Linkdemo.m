%% 参数说明
% 如果是旋转副，保持theta缺省
% 如果是移动副，保持 d缺省，同时指定上下限
% d：连杆偏移 即沿着连杆轴线运动
% theta：沿轴线的转动角
% a：连杆长度 即转动副轴线之间的距离
% alpha：扭转角，即转动副轴线之间的夹角

%% 构建一个简单的二连杆
% 参数 a=1 其余参数均为 0
Links(1) = Link( 'a', 0.5, 'alpha', 0,'d', 0);

Links(1).offset=pi/2; %通过Offset指定初值

Links(2) = Link( 'a', 1, 'alpha', 0,'d',0);

Robot=SerialLink(Links,'name','二连杆平面操作臂');

Robot.teach()