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

function PlotCosts(pop)

    Costs=[pop.Cost];
    
    plot(Costs(1,:),Costs(2,:),'r*','MarkerSize',8);
    xlabel('1^{st} Objective');
    ylabel('2^{nd} Objective');
    title('Non-dominated Solutions (F_{1})');
    grid on;

end