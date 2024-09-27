L1 =  Link([ 0,    0,       0,        0,      0], 'modified');
L2 =  Link([ 0,    0,       0,      -pi/2,    0], 'modified');
L3 =  Link([ 0,    0.149,   0.431,    0,      0], 'modified');
L4 =  Link([ 0,    0.433,   0.20,   -pi/2,    0], 'modified');
L5 =  Link([ 0,    0,      0,        pi/2,    0], 'modified');
L6 =  Link([ 0,    0,      0,       -pi/2,    0], 'modified');
robot=SerialLink([L1,L2,L3,L4,L5,L6]);   %SerialLink 类函数
robot.display();  %Link 类函数
robot.teach([0 0 0 0 0 0]);