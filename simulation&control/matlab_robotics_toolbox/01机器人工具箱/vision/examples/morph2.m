eg_morph1

% join the 2 squares
im(20:25,14)=1;
im=im';

S1 = ones(5);
e1 = imorph(im, S1, 'min');
d1 = imorph(e1, S1, 'max');

S2 = ones(7);
e2 = imorph(im, S2, 'min');
d2 = imorph(e2, S2, 'max');

S3 = ones(1,14);
e3 = imorph(im, S3, 'min');
d3 = imorph(e3, S3, 'max');

figure(1)
vsep = ones(numrows(im), 1);
hsep = ones(1,3*numcols(im)+3);
%idisp([e1 vsep e2 vsep e3; hsep; d1 vsep d2 vsep d3]);

p1 = bg*ones(numrows(vsep)+1, 25);
p1 = ipaste(p1, icolor(S1, [1 0 0]), [5,15]);
p2 = bg*ones(numrows(vsep)+1, 25);
p2 = ipaste(p2, icolor(S2, [1 0 0]), [5,15]);
p3 = bg*ones(numrows(vsep), 25);
p3 = ipaste(p3, icolor(S3, [1 0 0]), [5,15]);
panel = cat(1, p1, p2, p3);

results = icolor([im vsep e1 vsep d1 vsep; hsep;  im vsep e2 vsep d2 vsep; hsep; im vsep e3 vsep d3 vsep]);

idisp( cat(2, results, panel), 'noaxes', 'nogui', 'square')
pause(1)
iprint('morph_open' );

% now dilate

% regenerate the test pattern
eg_morph1

% make a hole in it
im(15,15) = bg;
im(15,14) = bg;
im(16,15) = bg;
im(16,14) = bg;
im=im';

d1 = imorph(im, S1, 'max');
e1 = imorph(d1, S1, 'min');

d2 = imorph(im, S2, 'max');
e2 = imorph(d2, S2, 'min');

d3 = imorph(im, S3, 'max');
e3 = imorph(d3, S3, 'min');

figure(1)
%idisp([e1 vsep e2 vsep e3; hsep; d1 vsep d2 vsep d3]);

results = icolor([im vsep d1 vsep e1 vsep; hsep;  im vsep d2 vsep e2 vsep; hsep; im vsep d3 vsep e3 vsep]);

idisp( cat(2, results, panel), 'noaxes', 'nogui', 'square');
iprint('morph_close');
