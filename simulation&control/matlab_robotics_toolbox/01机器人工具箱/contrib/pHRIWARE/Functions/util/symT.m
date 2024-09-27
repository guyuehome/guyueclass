function T = symT()

R = sym('R',3);
t = sym('t',[3 1]);
R = sym(R,'real');
t = sym(t,'real');
T = [R, t; 0 0 0 1];

end

