function plot_realtime(state,i,t,dt)
% hold on
quiver3([0;0;0],[0;0;0],[0;0;0],[state(1,1);state(4,1);state(7,1)],[state(2,1);state(5,1);state(8,1)],[state(3,1);state(6,1);state(9,1)],"off",'k','LineWidth',2);
hold on
if t>=0&&t<=3
    quiver3([0;0;0],[0;0;0],[0;0;0],[state(1,i);state(4,i);state(7,i)],[state(2,i);state(5,i);state(8,i)],[state(3,i);state(6,i);state(9,i)],"off",'b','LineWidth',2);
    hold off
elseif t>3&&t<=6
    quiver3([0;0;0],[0;0;0],[0;0;0],[state(1,3/dt);state(4,3/dt);state(7,3/dt)],[state(2,3/dt);state(5,3/dt);state(8,3/dt)],[state(3,3/dt);state(6,3/dt);state(9,3/dt)],"off",'b','LineWidth',2);
    quiver3([0;0;0],[0;0;0],[0;0;0],[state(1,i);state(4,i);state(7,i)],[state(2,i);state(5,i);state(8,i)],[state(3,i);state(6,i);state(9,i)],"off",'g','LineWidth',2);
    hold off
elseif t>6&&t<9
    quiver3([0;0;0],[0;0;0],[0;0;0],[state(1,3/dt);state(4,3/dt);state(7,3/dt)],[state(2,3/dt);state(5,3/dt);state(8,3/dt)],[state(3,3/dt);state(6,3/dt);state(9,3/dt)],"off",'b','LineWidth',2);
    quiver3([0;0;0],[0;0;0],[0;0;0],[state(1,6/dt);state(4,6/dt);state(7,6/dt)],[state(2,6/dt);state(5,6/dt);state(8,6/dt)],[state(3,6/dt);state(6,6/dt);state(9,6/dt)],"off",'g','LineWidth',2);
    quiver3([0;0;0],[0;0;0],[0;0;0],[state(1,i);state(4,i);state(7,i)],[state(2,i);state(5,i);state(8,i)],[state(3,i);state(6,i);state(9,i)],"off",'r','LineWidth',2);
    hold off
else
    quiver3([0;0;0],[0;0;0],[0;0;0],[state(1,3/dt);state(4,3/dt);state(7,3/dt)],[state(2,3/dt);state(5,3/dt);state(8,3/dt)],[state(3,3/dt);state(6,3/dt);state(9,3/dt)],"off",'b','LineWidth',2);
    quiver3([0;0;0],[0;0;0],[0;0;0],[state(1,6/dt);state(4,6/dt);state(7,6/dt)],[state(2,6/dt);state(5,6/dt);state(8,6/dt)],[state(3,6/dt);state(6,6/dt);state(9,6/dt)],"off",'g','LineWidth',2);
    quiver3([0;0;0],[0;0;0],[0;0;0],[state(1,9/dt);state(4,9/dt);state(7,9/dt)],[state(2,9/dt);state(5,9/dt);state(8,9/dt)],[state(3,9/dt);state(6,9/dt);state(9,9/dt)],"off",'r','LineWidth',2);
    plot_circle_arc([0;0;0],state(1:3,1),state(1:3,3/dt),[0 0 1])
    plot_circle_arc([0;0;0],state(4:6,1),state(4:6,3/dt),[0 0 1])

    plot_circle_arc([0;0;0],state(4:6,3/dt),state(4:6,6/dt),[0 1 0])
    plot_circle_arc([0;0;0],state(7:9,3/dt),state(7:9,6/dt),[0 1 0])

    plot_circle_arc([0;0;0],state(1:3,6/dt),state(1:3,9/dt),[1 0 0])
    plot_circle_arc([0;0;0],state(4:6,6/dt),state(4:6,9/dt),[1 0 0])

    hold off
end
xlabel('$$X$$','Interpreter','latex','FontSize',15);
ylabel('$$Y$$','Interpreter','latex','FontSize',15);
zlabel('$$Z$$','Interpreter','latex','FontSize',15);
axis equal;
xlim([-1,1]);
ylim([-1,1]);
zlim([-1,1]);
view(144,30);
pause(0.0001);
end