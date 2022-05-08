n=16;
m=18;

p=zeros(n,m);
sim=10000;
for s=1:sim
    if mod(s,1000)==0
        fprintf('已完成%d次模拟。\n',s)
    end
    g=glass(m);
    for i=1:n
        gamers(1,i)=gamer;
        g=gamers(i).go(g);
    end
    for i=1:n
        if gamers(i).fall<=m
            p(i,gamers(i).fall)=p(i,gamers(i).fall)+1;
        end
    end
end
p=p/sim;
