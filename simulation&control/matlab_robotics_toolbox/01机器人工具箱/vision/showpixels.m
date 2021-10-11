%SHOWPIXELS Show low resolution image
%
% Displays a low resolution image in detail as a grid with colored lines 
% between pixels and numeric display of pixel values at each pixel.  Useful
% for illustrating principles in teaching.
%
% Options::
% 'fmt',F         Format string (defaults to %d or %.2f depending on image type)
% 'label'         Display axis labels (default true)
% 'color',C       Text color (default 'b')
% 'fontsize',S    Font size (default 12)
% 'pixval'        Display pixel numeric values (default true)
% 'tick'          Display axis tick marks (default true)
% 'cscale',C      Color map scaling [min max] (defaults [0 1] or [0 255])
% 'uv',UV         UV={u,v} vectors of u and v coordinates
%
% Notes::
% - This is meant for small images, say 10x10 pixels.

function h = showpixels(im, varargin)
    
    if size(im,1)>20 || size(im,2)>20
        warning('showpixels is meant for small images');
    end
    
        nr = size(im,1); nc = size(im,2);
    if isinteger(im)
        opt.fmt = '%d';
        opt.cscale = [0 255];
        
    else
        opt.fmt = '%.2f';
        opt.cscale = [0 1];
    end
    opt.label = true;
    opt.tick = true;
    opt.color = 'b';
    opt.fontsize = 12;
    opt.pixval = true;
    opt.uv = [];
    
    [opt,args] = tb_optparse(opt, varargin);
    
    im0 = im;
    im(isnan(im)) = 0.5;
    
    if ~isempty(opt.uv)
        args = ['xydata', {opt.uv}, args];
                ulab = opt.uv{1};
        vlab = opt.uv{2};
    else

        ulab = 1:nc;
        vlab = 1:nr;
    end
    idisp(im, 'nogui', 'square', 'cscale', opt.cscale, args{:})
    if ~opt.label
        xlabel('');
        ylabel('');
    end
    
    axis equal
    hold on

    umin = min(ulab); vmin = min(vlab);
    umax = max(ulab); vmax = max(vlab);
    
        % draw horizontal lines
    for row=vlab
        plot([umin-1 umax]+0.5, [row row]+0.5, '-y');
    end
    % draw vertical lines
    for col=ulab
        plot([col col]+0.5, [vmin-1 vmax]+0.5, '-y');
    end
    if opt.pixval
        
        for row=1:nr
            for col=1:nc
                if ~isnan(im0(row,col))
                    h = text(ulab(col), vlab(row), sprintf(opt.fmt, im(row,col)) );
                    set(h, 'Color', opt.color, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', opt.fontsize );
                end
            end
        end
    end
    
    xaxis(min(ulab)-0.5, max(ulab)+0.5)
    yaxis(min(vlab)-0.5, max(vlab)+0.5)
    if opt.tick
        set(gca, 'XTick', ulab);
        set(gca, 'YTick', vlab);
    else
        set(gca, 'XTick', []);
        set(gca, 'YTick', []);
    end
    
    if nargout > 0
        h = findall(gca, 'type', 'image');
    end
    
    
