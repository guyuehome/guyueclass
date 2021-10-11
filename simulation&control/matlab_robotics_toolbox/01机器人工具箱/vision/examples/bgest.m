if ~exist('vid')
    vid = mmread('/Users/corkep/doc/book/src/imageproc/resources/LeftBag.mpg');
end

bg = cast(vid.frames(1).cdata(:,:,2), 'int32');

sigma = 1;
for i=2:vid.nrFramesTotal
    im = cast(vid.frames(i).cdata(:,:,2), 'int32');
    d = im-bg;
    % clip it
    d = min(d, sigma);
    d = max(d, -sigma);
    bg = bg + d;

    %idisp(im)
    idisp(im-bg, 'invsigned')
    drawnow

    if i > 250
        break
    end
end
iprint(im, 'foyer_frame')
iprint(bg, 'foyer_bg')
iprint(im-bg, 'foyer_diff', 'colormap', 'invsigned')
