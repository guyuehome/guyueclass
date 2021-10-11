im = bg*ones(50,50);

im = ipaste(im, ones(20,20), [5,5]);
im = ipaste(im, ones(10,15), [30,30]);
c = kcircle(10);
c(c==0) = bg;
im = ipaste(im, c, [5,27]);
idisp(im, 'nogui', 'square', 'noaxes', 'cscale', [0 1])
iprint('morph_bound1');

eroded = imorph(im, [0 1 0; 1 1 1; 0 1 0], 'min');
idisp(im-eroded+bg,'nogui', 'square', 'noaxes', 'cscale', [0 1])
iprint('morph_bound2');
