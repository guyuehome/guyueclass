function P=compute_constraint_distance_2param_6eq_3unk(m1,m2)

% Copyright (C) <2007>  <Francesc Moreno-Noguer, Vincent Lepetit, Pascal Fua>
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the version 3 of the GNU General Public License
% as published by the Free Software Foundation.
% 
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
% General Public License for more details.       
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.
%
% Francesc Moreno-Noguer, CVLab-EPFL, September 2007.
% fmorenoguer@gmail.com, http://cvlab.epfl.ch/~fmoreno/ 

%redefine variables name, for compatibility with maple
m1_1=m1(1); 
m1_2=m1(2); 
m1_3=m1(3); 
m1_4=m1(4); 
m1_5=m1(5); 
m1_6=m1(6);
m1_7=m1(7); 
m1_8=m1(8); 
m1_9=m1(9); 
m1_10=m1(10); 
m1_11=m1(11); 
m1_12=m1(12);

m2_1=m2(1); 
m2_2=m2(2); 
m2_3=m2(3); 
m2_4=m2(4); 
m2_5=m2(5); 
m2_6=m2(6);
m2_7=m2(7); 
m2_8=m2(8); 
m2_9=m2(9); 
m2_10=m2(10); 
m2_11=m2(11); 
m2_12=m2(12);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t7 = (m1_6 ^ 2);
t8 = (m1_4 ^ 2);
t9 = (m1_1 ^ 2);
t10 = (m1_5 ^ 2);
t11 = (m1_2 ^ 2);
t12 = (m1_3 ^ 2);
t17 = m1_4 * m2_4;
t18 = m1_1 * m2_1;
t19 = m1_5 * m2_5;
t22 = m1_2 * m2_2;
t23 = m1_6 * m2_6;
t25 = m1_3 * m2_3;
t26 = (-m2_6 * m1_3 - m1_4 * m2_1 - m2_4 * m1_1 + t17 + t18 + t19 - m1_5 * m2_2 - m2_5 * m1_2 + t22 + t23 - m1_6 * m2_3 + t25);
t29 = (m2_3 ^ 2);
t34 = (m2_4 ^ 2);
t35 = (m2_1 ^ 2);
t36 = (m2_5 ^ 2);
t37 = (m2_2 ^ 2);
t38 = (m2_6 ^ 2);
t44 = (m1_7 ^ 2);
t45 = (m1_8 ^ 2);
t46 = (m1_9 ^ 2);
t55 = m1_8 * m2_8;
t56 = m1_9 * m2_9;
t58 = m1_7 * m2_7;
t59 = (-m1_9 * m2_3 - m2_8 * m1_2 - m2_9 * m1_3 - m1_7 * m2_1 - m2_7 * m1_1 + t55 + t22 + t56 + t18 - m1_8 * m2_2 + t25 + t58);
t64 = (m2_8 ^ 2);
t65 = (m2_9 ^ 2);
t68 = (m2_7 ^ 2);
t72 = (m1_11 ^ 2);
t73 = (m1_12 ^ 2);
t74 = (m1_10 ^ 2);
t85 = m1_10 * m2_10;
t86 = m1_11 * m2_11;
t88 = m1_12 * m2_12;
t89 = (-m1_10 * m2_1 - m2_10 * m1_1 - m1_12 * m2_3 - m2_11 * m1_2 - m1_11 * m2_2 + t18 + t22 + t25 + t85 + t86 - m2_12 * m1_3 + t88);
t92 = (m2_11 ^ 2);
t95 = (m2_12 ^ 2);
t96 = (m2_10 ^ 2);
t113 = (-m1_9 * m2_6 - m2_9 * m1_6 + t55 + t23 + t17 + t56 + t58 - m1_7 * m2_4 - m2_7 * m1_4 - m1_8 * m2_5 - m2_8 * m1_5 + t19);
t134 = (-m1_10 * m2_4 - m2_10 * m1_4 + t88 + t23 + t17 + t85 + t86 - m1_11 * m2_5 - m2_11 * m1_5 - m1_12 * m2_6 - m2_12 * m1_6 + t19);
t155 = (t58 + t88 - m2_10 * m1_7 - m2_11 * m1_8 + t56 - m1_10 * m2_7 + t55 + t85 + t86 - m1_12 * m2_9 - m2_12 * m1_9 - m1_11 * m2_8);
P(1,1) = -2 * m1_4 * m1_1 - 2 * m1_5 * m1_2 - 2 * m1_6 * m1_3 + t7 + t8 + t9 + t10 + t11 + t12;
P(1,2) = 2 * t26;
P(1,3) = -2 * m2_6 * m2_3 + t29 - 2 * m2_4 * m2_1 - 2 * m2_5 * m2_2 + t34 + t35 + t36 + t37 + t38;
P(2,1) = -2 * m1_7 * m1_1 + t12 - 2 * m1_9 * m1_3 + t44 + t45 + t46 - 2 * m1_8 * m1_2 + t9 + t11;
P(2,2) = 2 * t59;
P(2,3) = -2 * m2_8 * m2_2 - 2 * m2_9 * m2_3 + t64 + t65 - 2 * m2_7 * m2_1 + t29 + t68 + t37 + t35;
P(3,1) = t9 - 2 * m1_12 * m1_3 + t72 + t73 + t74 + t12 - 2 * m1_11 * m1_2 - 2 * m1_10 * m1_1 + t11;
P(3,2) = 2 * t89;
P(3,3) = -2 * m2_11 * m2_2 + t37 + t92 - 2 * m2_10 * m2_1 + t95 + t29 + t96 - 2 * m2_12 * m2_3 + t35;
P(4,1) = -2 * m1_9 * m1_6 + t8 + t10 + t7 - 2 * m1_7 * m1_4 + t44 + t45 + t46 - 2 * m1_8 * m1_5;
P(4,2) = 2 * t113;
P(4,3) = -2 * m2_9 * m2_6 + t68 + t64 - 2 * m2_7 * m2_4 - 2 * m2_8 * m2_5 + t34 + t36 + t38 + t65;
P(5,1) = t73 + t8 + t10 + t7 - 2 * m1_10 * m1_4 - 2 * m1_11 * m1_5 - 2 * m1_12 * m1_6 + t74 + t72;
P(5,2) = 2 * t134;
P(5,3) = -2 * m2_12 * m2_6 + t96 + t92 - 2 * m2_10 * m2_4 - 2 * m2_11 * m2_5 + t34 + t36 + t38 + t95;
P(6,1) = t46 + t73 + t44 + t45 + t74 + t72 - 2 * m1_11 * m1_8 - 2 * m1_10 * m1_7 - 2 * m1_12 * m1_9;
P(6,2) = 2 * t155;
P(6,3) = t65 - 2 * m2_10 * m2_7 + t96 + t92 + t95 - 2 * m2_11 * m2_8 + t68 + t64 - 2 * m2_12 * m2_9;