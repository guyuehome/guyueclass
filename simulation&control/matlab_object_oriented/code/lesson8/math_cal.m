n=16;
m=18;

p=zeros(n,m);
p(1,1)=0.5;
for j=2:m
    p(1,j)=0.5*p(1,j-1);
end

for i=2:n
    for j=1:m
        for k=1:(j-1)
            p(i,j)=p(i,j)+p(i-1,k)*0.5^(j-k);
        end
    end
end
%%
figure;
bar(1:n,1-sum(p,2));
xlabel('Player number');
ylabel('Probability of completion');

figure;
heatmap(p');
xlabel('Player number');
ylabel('Position of glass bridge');
title('Probability of falling at a certain location');