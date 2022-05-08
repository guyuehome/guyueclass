function z=test3(x)

    n=numel(x);
    
    R=sqrt(sum(x.^2));
    z1=-sin(R)/R;
    z2=1;
    
    z=[z1 z2]';

end