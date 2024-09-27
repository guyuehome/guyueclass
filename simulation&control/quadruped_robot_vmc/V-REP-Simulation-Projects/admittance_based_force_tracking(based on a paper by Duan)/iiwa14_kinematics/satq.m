function modifiedq = satq(qmin,qmax,q)
modifiedq = q;
count = length(q);
for i=1:count
    if modifiedq(i)<qmin(i)
        modifiedq(i) = qmin(i);
    end
    if modifiedq(i)>qmax(i)
        modifiedq(i) = qmax(i);
    end
end
end