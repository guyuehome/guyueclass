s=0;
for i=1:100
    s=s+i;
    if s>=1000
        break
    end
end
fprintf('1到%d的和刚好超过1000，其和为%d\n',i,s);