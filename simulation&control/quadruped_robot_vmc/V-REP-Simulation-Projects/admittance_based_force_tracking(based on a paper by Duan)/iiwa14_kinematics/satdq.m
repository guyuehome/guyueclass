function modifieddq = satdq(dqlimit,q)
modifieddq = q;
count = length(q);
for i=1:count
    if modifieddq(i)<-dqlimit(i)
        modifieddq(i) = -dqlimit(i);
    end
    if modifieddq(i)> dqlimit(i)
        modifieddq(i) = dqlimit(i);
    end
end
end