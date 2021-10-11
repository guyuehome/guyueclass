%ICANNY	Canny edge detection
%
% E = ICANNY(IM, OPTIONS) is an edge image obtained using the Canny edge 
% detector algorithm.  Hysteresis filtering is applied to the gradient
% image: edge pixels > th1 are connected to adjacent pixels > th0, those
% below th0 are set to zero.
%
% Options::
%  'sd',S    set the standard deviation for smoothing (default 1)
%  'th0',T   set the lower hysteresis threshold (default 0.1 x strongest edge)
%  'th1',T   set the upper hysteresis threshold (default 0.5 x strongest edge)
%
% Reference::
% - "A Computational Approach To Edge Detection", 
%   J. Canny,
%   IEEE Trans. Pattern Analysis and Machine Intelligence, 8(6):679–698, 1986.
%
% Notes::
% - Produces a zero image with single pixel wide edges having non-zero values.
% - Larger values correspond to stronger edges.
% - If th1 is zero then no hysteresis filtering is performed.
% - A color image is automatically converted to greyscale first.
%
% Author::
% Oded Comay, Tel Aviv University, 1996-7.
%
% See also ISOBEL, KDGAUSS.

function E = icanny(I, varargin)

    % convert color image to greyscale
    if iscolor(I)
        I = imono(I);
    end

    opt.sd = 1;
    opt.th0 = 0.1;
    opt.th1 = 0.5;

    opt = tb_optparse(opt, varargin);

    x= -5*opt.sd:opt.sd*5; 
    g = exp(-0.5/opt.sd^2*x.^2); 		% Create a normalized Gaussian
    g = g(g>max(g)*.005); g = g/sum(g(:));
    dg = diff(g);				% Gaussian first derivative

    dx = abs(conv2(I, dg, 'same'));		% X/Y edges
    dy = abs(conv2(I, dg', 'same'));

    [ny, nx] = size(I);
                        % Find maxima 
    dy0 = [dy(2:ny,:); dy(ny,:)]; dy2 = [dy(1,:); dy(1:ny-1,:)];
    dx0 = [dx(:, 2:nx) dx(:,nx)]; dx2 = [dx(:,1) dx(:,1:nx-1)];
    peaks = find((dy>dy0 & dy>dy2) | (dx>dx0 & dx>dx2));
    e = zeros(size(I));
    e(peaks) = sqrt(dx(peaks).^2 + dy(peaks).^2); 

    e(:,2)    = zeros(ny,1);    e(2,:) = zeros(1,nx);	% Remove artificial edges
    e(:,nx-2) = zeros(ny,1); e(ny-2,:) = zeros(1,nx);
    e(:,1)    = zeros(ny,1);    e(1,:) = zeros(1,nx);
    e(:,nx)   = zeros(ny,1);   e(ny,:) = zeros(1,nx);
    e(:,nx-1) = zeros(ny,1); e(ny-1,:) = zeros(1,nx);
    e = e/max(e(:));

    if opt.th1  == 0, E = e; return; end			 % Perform hysteresis
    E(ny,nx) = 0;

    p = find(e >= opt.th1);
    while length(p) 
      E(p) = e(p);
      e(p) = zeros(size(p));
      n = [p+1 p-1 p+ny p-ny p-ny-1 p-ny+1 p+ny-1 p+ny+1]; % direct neighbors
      On = zeros(ny,nx); On(n) = n;
      p = find(e > opt.th0 & On);
    end
