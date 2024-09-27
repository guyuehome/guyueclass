%
% Copyright (c) 2015, Yarpiz (www.yarpiz.com)
% All rights reserved. Please read the "license.txt" for license terms.
%
% Project Code: YPEA120
% Project Title: Non-dominated Sorting Genetic Algorithm II (NSGA-II)
% Publisher: Yarpiz (www.yarpiz.com)
% 
% Developer: S. Mostapha Kalami Heris (Member of Yarpiz Team)
% 
% Contact Info: sm.kalami@gmail.com, info@yarpiz.com
%

function z=MOP1(x)

    n=numel(x);
    
    z1=sum(sin(x));
    
    z2=sum(sin(x+pi/4));
    
    z3=sum(sin(x+pi/2));
    
    z=[z1 z2 z3]';

end