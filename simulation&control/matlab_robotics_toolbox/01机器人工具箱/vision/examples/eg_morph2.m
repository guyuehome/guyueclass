bg = 0.7;
im = bg*ones(50,50);

im = ipaste(im, ones(20,20), [5,5]);
im = ipaste(im, ones(10,15), [30,30]);
c = kcircle(10);
c(c==0) = bg;
im = ipaste(im, c, [5,27]);
