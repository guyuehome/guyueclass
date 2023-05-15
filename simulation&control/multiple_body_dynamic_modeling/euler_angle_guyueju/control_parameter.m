function theta = control_parameter(t)
if t<=3
    theta   = [pi/6*(t/3);0;0];
end
if t>3&&t<=6
    theta   = [pi/6;pi/6*((t-3)/3);0];
end
if t>6&&t<=9
    theta   = [pi/6;pi/6;pi/6*((t-6)/3)];
end
if t>9
    theta   = [pi/6;pi/6;pi/6];
end

end