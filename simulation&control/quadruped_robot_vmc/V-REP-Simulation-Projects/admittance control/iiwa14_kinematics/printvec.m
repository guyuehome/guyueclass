function printvec(vec)
str='[ ';
count = length(vec);
for i=1:count
    str = [str,num2str(vec(i)),' '];
end
str = [str,' ]'];
disp(str);
end