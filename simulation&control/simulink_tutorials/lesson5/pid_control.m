m = [];
for i=[1:10,20:10:100]
    disp(i);
    set_param('PID_test/PID Controller', 'P',num2str(i));
    myout=sim('PID_test');
    m=[m,max(myout.pid_out.Data)];
end
plot([1:10,20:10:100],m);