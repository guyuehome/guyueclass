% set background to grey
bg = 0.7;
im = bg*ones(50, 40);

% create the four objects
im = ipaste(im, ones(10), [10,10]);
im = ipaste(im, ones(1,2), [8,15]);
im = ipaste(im, ones(2,1), [15,8]);
im = ipaste(im, ones(5), [12,25]);

%im = ipaste(im, ktriangle(10), [30,10]);
%im = ipaste(im, ktriangle(5), [25,30]);
%im = ipaste(im, flipud(ktriangle(5)), [35,30]);

% add the protrusions to big square
im = ipaste(im, ones(2,20), [5,40]);
im = ipaste(im, ones(20,2),[35,5]);

%im = ipaste(im, colorize2(ktriangle(10), [1 0 0]), [70,70]);

idisp(im);
