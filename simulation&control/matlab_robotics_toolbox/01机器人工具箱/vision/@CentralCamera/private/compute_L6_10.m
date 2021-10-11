function L=compute_L6_10(K)

% COMPUTE_L6_10  
%
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
% Francesc Moreno-Noguer, CVLab-EPFL, October 2007.
% fmorenoguer@gmail.com, http://cvlab.epfl.ch/~fmoreno/ 

L=zeros(6,10);

%extract vectors
v1=K(:,1);
v2=K(:,2);
v3=K(:,3);
v4=K(:,4);

%compute differences
dx112 = v1(1) - v1(4);  dy112 = v1(2) - v1(5);  dz112 = v1(3) - v1(6);
dx113 = v1(1) - v1(7);  dy113 = v1(2) - v1(8);  dz113 = v1(3) - v1(9);
dx114 = v1(1) - v1(10); dy114 = v1(2) - v1(11); dz114 = v1(3) - v1(12);
dx123 = v1(4) - v1(7);  dy123 = v1(5) - v1(8);  dz123 = v1(6) - v1(9);
dx124 = v1(4) - v1(10); dy124 = v1(5) - v1(11); dz124 = v1(6) - v1(12);
dx134 = v1(7) - v1(10); dy134 = v1(8) - v1(11); dz134 = v1(9) - v1(12);

dx212 = v2(1) - v2(4);  dy212 = v2(2) - v2(5);  dz212 = v2(3) - v2(6);
dx213 = v2(1) - v2(7);  dy213 = v2(2) - v2(8);  dz213 = v2(3) - v2(9);
dx214 = v2(1) - v2(10); dy214 = v2(2) - v2(11); dz214 = v2(3) - v2(12);
dx223 = v2(4) - v2(7);  dy223 = v2(5) - v2(8);  dz223 = v2(6) - v2(9);
dx224 = v2(4) - v2(10); dy224 = v2(5) - v2(11); dz224 = v2(6) - v2(12);
dx234 = v2(7) - v2(10); dy234 = v2(8) - v2(11); dz234 = v2(9) - v2(12);

dx312 = v3(1) - v3(4);  dy312 = v3(2) - v3(5);  dz312 = v3(3) - v3(6);
dx313 = v3(1) - v3(7);  dy313 = v3(2) - v3(8);  dz313 = v3(3) - v3(9);
dx314 = v3(1) - v3(10); dy314 = v3(2) - v3(11); dz314 = v3(3) - v3(12);
dx323 = v3(4) - v3(7);  dy323 = v3(5) - v3(8);  dz323 = v3(6) - v3(9);
dx324 = v3(4) - v3(10); dy324 = v3(5) - v3(11); dz324 = v3(6) - v3(12);
dx334 = v3(7) - v3(10); dy334 = v3(8) - v3(11); dz334 = v3(9) - v3(12);

dx412 = v4(1) - v4(4);  dy412 = v4(2) - v4(5);  dz412 = v4(3) - v4(6);
dx413 = v4(1) - v4(7);  dy413 = v4(2) - v4(8);  dz413 = v4(3) - v4(9);
dx414 = v4(1) - v4(10); dy414 = v4(2) - v4(11); dz414 = v4(3) - v4(12);
dx423 = v4(4) - v4(7);  dy423 = v4(5) - v4(8);  dz423 = v4(6) - v4(9);
dx424 = v4(4) - v4(10); dy424 = v4(5) - v4(11); dz424 = v4(6) - v4(12);
dx434 = v4(7) - v4(10); dy434 = v4(8) - v4(11); dz434 = v4(9) - v4(12);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
L(1,1) =        dx112 * dx112 + dy112 * dy112 + dz112 * dz112;      %b1*b1
L(1,2) = 2.0 *  (dx112 * dx212 + dy112 * dy212 + dz112 * dz212);    %b1*b2
L(1,3) =        dx212 * dx212 + dy212 * dy212 + dz212 * dz212;      %b2*b2
L(1,4) = 2.0 *  (dx112 * dx312 + dy112 * dy312 + dz112 * dz312);    %b1*b3
L(1,5) = 2.0 *  (dx212 * dx312 + dy212 * dy312 + dz212 * dz312);    %b2*b3
L(1,6) =        dx312 * dx312 + dy312 * dy312 + dz312 * dz312;      %b3*b3
L(1,7) = 2.0 *  (dx112 * dx412 + dy112 * dy412 + dz112 * dz412);    %b1*b4
L(1,8) = 2.0 *  (dx212 * dx412 + dy212 * dy412 + dz212 * dz412);    %b2*b4
L(1,9) = 2.0 *  (dx312 * dx412 + dy312 * dy412 + dz312 * dz412);    %b3*b4
L(1,10) =       dx412 * dx412 + dy412 * dy412 + dz412 * dz412;      %b4*b4


L(2,1) =        dx113 * dx113 + dy113 * dy113 + dz113 * dz113;
L(2,2) = 2.0 *  (dx113 * dx213 + dy113 * dy213 + dz113 * dz213);
L(2,3) =        dx213 * dx213 + dy213 * dy213 + dz213 * dz213;
L(2,4) = 2.0 *  (dx113 * dx313 + dy113 * dy313 + dz113 * dz313);
L(2,5) = 2.0 *  (dx213 * dx313 + dy213 * dy313 + dz213 * dz313);
L(2,6) =        dx313 * dx313 + dy313 * dy313 + dz313 * dz313;
L(2,7) = 2.0 *  (dx113 * dx413 + dy113 * dy413 + dz113 * dz413);
L(2,8) = 2.0 *  (dx213 * dx413 + dy213 * dy413 + dz213 * dz413);
L(2,9) = 2.0 *  (dx313 * dx413 + dy313 * dy413 + dz313 * dz413);
L(2,10) =       dx413 * dx413 + dy413 * dy413 + dz413 * dz413;


L(3,1) =        dx114 * dx114 + dy114 * dy114 + dz114 * dz114;
L(3,2) = 2.0 *  (dx114 * dx214 + dy114 * dy214 + dz114 * dz214);
L(3,3) =        dx214 * dx214 + dy214 * dy214 + dz214 * dz214;
L(3,4) = 2.0 *  (dx114 * dx314 + dy114 * dy314 + dz114 * dz314);
L(3,5) = 2.0 *  (dx214 * dx314 + dy214 * dy314 + dz214 * dz314);
L(3,6) =        dx314 * dx314 + dy314 * dy314 + dz314 * dz314;
L(3,7) = 2.0 *  (dx114 * dx414 + dy114 * dy414 + dz114 * dz414);
L(3,8) = 2.0 *  (dx214 * dx414 + dy214 * dy414 + dz214 * dz414);
L(3,9) = 2.0 *  (dx314 * dx414 + dy314 * dy414 + dz314 * dz414);
L(3,10) =       dx414 * dx414 + dy414 * dy414 + dz414 * dz414;


L(4,1) =        dx123 * dx123 + dy123 * dy123 + dz123 * dz123;
L(4,2) = 2.0 *  (dx123 * dx223 + dy123 * dy223 + dz123 * dz223);
L(4,3) =        dx223 * dx223 + dy223 * dy223 + dz223 * dz223;
L(4,4) = 2.0 *  (dx123 * dx323 + dy123 * dy323 + dz123 * dz323);
L(4,5) = 2.0 *  (dx223 * dx323 + dy223 * dy323 + dz223 * dz323);
L(4,6) =        dx323 * dx323 + dy323 * dy323 + dz323 * dz323;
L(4,7) = 2.0 *  (dx123 * dx423 + dy123 * dy423 + dz123 * dz423);
L(4,8) = 2.0 *  (dx223 * dx423 + dy223 * dy423 + dz223 * dz423);
L(4,9) = 2.0 *  (dx323 * dx423 + dy323 * dy423 + dz323 * dz423);
L(4,10) =       dx423 * dx423 + dy423 * dy423 + dz423 * dz423;


L(5,1) =        dx124 * dx124 + dy124 * dy124 + dz124 * dz124;
L(5,2) = 2.0 *  (dx124 * dx224 + dy124 * dy224 + dz124 * dz224);
L(5,3) =        dx224 * dx224 + dy224 * dy224 + dz224 * dz224;
L(5,4) = 2.0 * ( dx124 * dx324 + dy124 * dy324 + dz124 * dz324);
L(5,5) = 2.0 * (dx224 * dx324 + dy224 * dy324 + dz224 * dz324);
L(5,6) =        dx324 * dx324 + dy324 * dy324 + dz324 * dz324;
L(5,7) = 2.0 * ( dx124 * dx424 + dy124 * dy424 + dz124 * dz424);
L(5,8) = 2.0 * ( dx224 * dx424 + dy224 * dy424 + dz224 * dz424);
L(5,9) = 2.0 * ( dx324 * dx424 + dy324 * dy424 + dz324 * dz424);
L(5,10) =       dx424 * dx424 + dy424 * dy424 + dz424 * dz424;


L(6,1) =        dx134 * dx134 + dy134 * dy134 + dz134 * dz134;
L(6,2) = 2.0 * ( dx134 * dx234 + dy134 * dy234 + dz134 * dz234);
L(6,3) =        dx234 * dx234 + dy234 * dy234 + dz234 * dz234;
L(6,4) = 2.0 * ( dx134 * dx334 + dy134 * dy334 + dz134 * dz334);
L(6,5) = 2.0 * ( dx234 * dx334 + dy234 * dy334 + dz234 * dz334);
L(6,6) =        dx334 * dx334 + dy334 * dy334 + dz334 * dz334;
L(6,7) = 2.0 *  (dx134 * dx434 + dy134 * dy434 + dz134 * dz434);
L(6,8) = 2.0 *  (dx234 * dx434 + dy234 * dy434 + dz234 * dz434);
L(6,9) = 2.0 *  (dx334 * dx434 + dy334 * dy434 + dz334 * dz434);
L(6,10)=       dx434 * dx434 + dy434 * dy434 + dz434 * dz434;

