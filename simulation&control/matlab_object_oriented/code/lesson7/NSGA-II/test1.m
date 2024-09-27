function z=test1(x)

    n=numel(x);
    
    z1=1-exp(-sum((x-1/sqrt(n)).^2));
    
    z2=1-exp(-sum((x+1/sqrt(n)).^2));
    
    z=[z1 z2]';

end