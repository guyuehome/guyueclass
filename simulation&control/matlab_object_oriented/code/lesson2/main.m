%% 关于继承
ev=e_vehicle(95,2020,'yiqi');
ev.run(10);
disp(ev.soc);

%% 句柄类和实体类的对比
%分别创建两个对象
dv=dog_v('Mary');
dv.age=5;
dh=dog_h('Lucy');
dh.age=5;
%简单复制
dv_copy=dv;%对于实体类，二者是相互独立的
dh_copy=dh;%对于句柄类，两个句柄指向同一块内存

%属性的修改
dv=dv.set_age(3);
dh.set_age(3);

%相等关系的判断
disp(isequal(dv,dv_copy));
disp(dh==dh_copy);
disp(isequal(dh,dh_copy));