ts=timeseries(sin([0:0.1:10]),[0:0.1:10]);

matrix=zeros(size(repmat([0:0.1:10]',[1 2])));
matrix(:,1)=[0:0.1:10]';
matrix(:,2)=sin([0:0.1:10])';

struc=struct('time',[0:0.1:10]','signals', ...
    struct('values',sin([0:0.1:10]'),'dimension',length([0:0.1:10])));