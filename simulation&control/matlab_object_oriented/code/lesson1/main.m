%构造一个小狗的实体
d=dog('Mary');
d.sit;                %命令其坐下Mary is now sitting.
d.how_old;      %询问其年龄I do not know my age.
d.age=3;          %通过赋值告诉其年龄
d.how_old;      %再次询问3 years old.

%%
%构造一个车辆的实体
v=vehicle(2020,'dazhong');
v.describe;             %获取描述信息dazhong made in 2020.
v.car_miles             %查询行驶里程（为空）
v.car_miles=0;      %初始化行驶里程为0
v.run(100);             %行驶100英里
v.car_miles             %再次查询里程（为100）