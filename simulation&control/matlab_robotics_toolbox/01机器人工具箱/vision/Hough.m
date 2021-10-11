%Hough  Hough transform class
%
% The Hough transform is a technique for finding lines in an image using
% a voting scheme.  For every edge pixel in the input image a set of cells
% in the Hough accumulator (voting array) are incremented.
%
% In this version of the Hough transform lines are described 
% by:
%        d = y cos(theta) + x sin(theta)
% where theta is the angle the line makes to horizontal axis, and d is the 
% perpendicular distance between (0,0) and the line.  A horizontal  line has 
% theta = 0, a vertical line has theta = pi/2 or -pi/2.
%
% The voting array is 2-dimensional, with columns corresponding to theta and
% rows corresponding to offset (d). Theta spans the range -pi/2 to pi/2 in Ntheta 
% steps.  Offset is in the range -rho_max to rho_max where rho_max=max(W,H).
%
% Methods::
% plot      Overlay detected lines
% show      Display the Hough accumulator
% lines     Return line features
% char      Convert Hough parameters to string
% display   Display Hough parameters
% 
% Properties::
%
% Nrho          Number of bins in rho direction
% Ntheta        Number of bins in theta direction
% A             The Hough accumulator (Nrho x Ntheta)
% rho           rho values for the centre of each bin vertically
% theta         Theta values for the centre of each bin horizontally
% edgeThresh    Threshold on relative edge pixel strength
% houghThresh   Threshold on relative peak strength
% suppress      Radius of accumulator cells cleared around peak 
% interpWidth   Width of region used for peak interpolation
%
% Notes::
%  - Hough is a reference object.
%
% See also LineFeature.


% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Machine Vision Toolbox for Matlab (MVTB).
% 
% MVTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% MVTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with MVTB.  If not, see <http://www.gnu.org/licenses/>.

classdef Hough < handle

    properties
        Nrho
        Ntheta
        suppress
        edgeThresh
        interpWidth
        houghThresh

        A       % the Hough accumulator
        rho       % the rho values for the centre of each bin vertically
        theta   % the theta values for the centre of each bin horizontally
        rho_offset
        rho_scale
    end

    methods
        function h = Hough(IM, varargin)
        %Hough.Hough Create Hough transform object
        %
        % HT = Hough(E, OPTIONS) is the Hough transform of the edge image E.
        %
        % For every pixel in the edge image E (HxW) greater than a threshold 
        % the corresponding elements of the accumulator are incremented.  By
        % default the vote is incremented by the edge strength but votes
        % can be made equal with the option 'equal'.  The threshold is
        % determined from the maximum edge strength value x HT.edgeThresh.
        %
        % Options::
        % 'equal'            All edge pixels have equal weight, otherwise the
        %                    edge pixel value is the vote strength
        % 'points'           Pass set of points rather than an edge image, in 
        %                    this case E (2xN) is a set of N points, or E (3xN)
        %                    is a set of N points with corresponding vote strengths 
        %                    as the third row
        % 'interpwidth',W    Interpolation width (default 3)
        % 'houghthresh',T    Set HT.houghThresh (default 0.5)
        % 'edgethresh',T     Set HT.edgeThresh (default 0.1);
        % 'suppress',W       Set HT.suppress (default 0)
        % 'nbins',N          Set number of bins, if N is scalar set Nrho=Ntheta=N, else
        %                    N = [Ntheta, Nrho].  Default 400x401.
            opt.interpwidth = 3;
            opt.houghthresh = 0.5;
            opt.edgethresh = 0.1;
            opt.suppress = [];
            opt.nbins = [];
            opt.equal = false;
            opt.points = false;

            h.Nrho = 401;
            h.Ntheta = 400;

            opt.interpwidth = 3;
            opt.houghthresh = 0.5;
            opt.edgethresh = 0.1;
            opt.suppress = [];
            opt.nbins = [];

            opt = tb_optparse(opt, varargin);

            % copy options into the Hough object
            if ~isempty(opt.nbins)
                if length(opt.nbins) == 1
                    h.Ntheta = opt.nbins;
                    h.Nrho = opt.nbins;
                 elseif length(opt.nbins) == 2
                    h.Ntheta = opt.nbins(1);
                    h.Nrho = opt.nbins(2);
                else
                    error('1 or 2 elements for nbins');
                end
            end

            h.Nrho = bitor(h.Nrho, 1);            % Nrho must be odd
            h.Ntheta = floor(h.Ntheta/2)*2; % Ntheta must even

            h.interpWidth = opt.interpwidth;
            h.houghThresh = opt.houghthresh;
            h.edgeThresh = opt.edgethresh;
            h.suppress = opt.suppress;
            
            if isempty(opt.suppress)
                h.suppress = (h.interpWidth-1)/2;
            end

            if opt.points
                xyz = IM';
                nr = max(xyz(:,2));
                nc = max(xyz(:,1));
            else
                [nr,nc] = size(IM);
                
                % find the significant edge pixels
                IM = abs(IM);
                globalMax = max(IM(:));
                i = find(IM > (globalMax*h.edgeThresh));
                [r,c] = ind2sub(size(IM), i);
                
                if opt.equal
                    xyz = [c r];
                else
                    xyz = [c r IM(i)];
                end
            end

            % now pass the x/y/strength info to xyhough
            h.A = h.xyhough(xyz, norm([nr,nc]));
        end

        function show(h)
        %Hough.show Display the Hough accumulator as image
        %
        % S = HT.show() displays the Hough vote accumulator as an image using the
        % hot colormap, where 'heat' is proportional to the number of votes.
        %
        % See also COLORMAP, HOT.
            clf
            hi = image(h.theta, h.rho, h.A/max(max(h.A)));
            set(hi, 'CDataMapping', 'scaled');
            set(gca, 'YDir', 'normal');
            set(gca, 'Xcolor', [1 1 1]*0.5);
            set(gca, 'Ycolor', [1 1 1]*0.5);
            grid on
            xlabel('\theta (rad)');
            ylabel('\rho (pixels)');
            colormap(hot)
        end
            
        function handles = plot(h, varargin)
        %Hough.plot Plot line features
        %
        % HT.plot() overlays all detected lines on the current figure.
        %
        % HT.plot(N) overlays a maximum of N strongest lines on the current figure.
        %
        % HT.plot(N, LS) as above but the optional line style arguments LS are
        % passed to plot.
        %
        % H = HT.plot() as above but returns a vector of graphics handles for each
        % line.
        %
        % See also Hough.lines.

            holdon = ishold;
            hold on

            lines = h.lines();
    
            if (nargin > 1) && isnumeric(varargin{1})
                N = varargin{1};
                varargin = varargin(2:end);
                n = min(numel(lines), N);
                lines = lines(1:n);
            end

            % plot it
            lines.plot(varargin{:});

        end
        
        
        function out = lines(h, N)
        %Hough.lines Find lines
        %
        % L = HT.lines() is a vector of LineFeature objects that
        % represent the dominant lines in the Hough accumulator.
        %
        % L = HT.lines(N) as above but returns no more than N LineFeature
        % objects.
        %
        % Lines are the coordinates of peaks in the Hough accumulator.  
        % The highest peak is found, refined to subpixel precision, then 
        % all elements in an HT.suppress radius around are zeroed so as to eliminate
        % multiple close minima.  The process is repeated for all peaks.
        %
        % The peak detection loop breaks early if the remaining peak has a strength
        % less than HT.houghThresh times the maximum vote value.
        %
        % See also Hough.plot, LineFeature.
            if nargin < 2
                N = Inf;
            end

            if N < 1
                thresh = N;
                N = Inf;
            end

            [x,y] = meshgrid(1:h.Ntheta, 1:h.Nrho);

            nw2= floor((h.interpWidth-1)/2);
            nr2= floor((h.suppress-1)/2);

            [Wx,Wy] = meshgrid(-nw2:nw2,-nw2:nw2);
            globalMax = max(h.A(:));
            
            A = h.A;

            i = 0;
            while i<=N
                i = i + 1;
                
                % find the current peak
                [mx,where] = max(A(:));
                %[mx where]
                
                % is the remaining peak good enough?
                if mx < (globalMax*h.houghThresh)
                    break;
                end
                [rp,cp] = ind2sub(size(A), where);
                %fprintf('\npeak height %f at (%d,%d)\n', mx, cp, rp);
                if h.interpWidth == 0
                    d = H.rho(rp);
                    theta = H.theta(cp);
                    p(i,:) = [d theta mx/globalMax];
                else
                    % refine the peak to subelement accuracy

                    k = Hough.nhood2ind(A, ones(h.interpWidth,h.interpWidth), [cp,rp]);
                    Wh = A(k);
                    %Wh                    
                    rr = Wy .* Wh;
                    cc = Wx .* Wh;
                    ri = sum(rr(:)) / sum(Wh(:)) + rp;
                    ci = sum(cc(:)) / sum(Wh(:)) + cp;
              
                    
                    %fprintf('refined %f %f\n', ci, ri);

                    % interpolate the line parameter values
                    d = interp1(h.rho, ri);
                    theta = interp1(h.theta, ci, 'linear', 0);
                    %p(i,:) = [d theta mx/globalMax];
                    L(i) = LineFeature(d, theta, mx/globalMax);
                    %p(i,:)

                end
                
                % remove the region around the peak
                k = Hough.nhood2ind(A, ones(2*h.suppress+1,2*h.suppress+1), [cp,rp]);
                A(k) = 0;

            end
            if nargout == 1
                out = L;
            else
                h.show
                hold on
                plot([L.theta]', [L.rho]', 'go');
                hold off
            end
        end % peaks


        %XYHOUGH    XY Hough transform
        %
        %   H = XYHOUGH(XYZ, drange, Nth)
        %
        %   Compute the Hough transform of the XY data given as the first two 
        %   columns of XYZ.  The last column, if given, is the point strength, 
        %   and is used as the increment for the Hough accumulator for that point.
        %
        %   The accumulator array has theta across the columns and offset down 
        %   the rows.  Theta spans the range -pi/2 to pi/2 in Nth increments.
        %   The distance span is given by drange which is either
        %       [dmin dmax] in the range dmin to dmax in steps of 1, or
        %       [dmin dmax Nd] in the range dmin to dmax with Nd steps.
        %
        %   Clipping is applied so that only those points lying within the Hough 
        %   accumulator bounds are updated.
        %
        %   The output arguments TH and D give the theta and offset value vectors 
        %   for the accumulator columns and rows respectively.  With no output 
        %   arguments the Hough accumulator is displayed as a greyscale image.
        % 
        %   For this version of the Hough transform lines are described by
        %
        %       d = y cos(theta) + x sin(theta)
        %
        %   where theta is the angle the line makes to horizontal axis, and d is 
        %   the perpendicular distance between (0,0) and the line.  A horizontal 
        %   line has theta = 0, a vertical line has theta = pi/2 or -pi/2
        %
        % SEE ALSO: ihough mkline, mksq, isobel
        %
        function H = xyhough(h, XYZ, dmax, Nth)

            inc = 1;
            
            h.rho_offset = (h.Nrho+1)/2;
            h.rho_scale = (h.Nrho-1)/2 / dmax;
            
            if numcols(XYZ) == 2
                XYZ = [XYZ ones(numrows(XYZ),1)];
            end

            % compute the quantized theta values and the sin/cos
            nt2 = h.Ntheta/2;
            h.theta = [-nt2:(nt2-1)]'/nt2*pi/2;
            st = sin(h.theta);
            ct = cos(h.theta);

            H = zeros(h.Nrho, h.Ntheta);        % create the Hough accumulator

            % this is a fast `vectorized' algorithm

            % evaluate the index of the top of each column in the Hough array
            col0 = ([1:h.Ntheta]'-1)*h.Nrho;
            %col0_r = [(Nth-1):-1:0]'*Nrho + 1;

            for xyz = XYZ'
                x = xyz(1);     % determine (x, y) coordinate
                y = xyz(2);
                inc = xyz(3);
                inc =1 ;
                
                rho = y * ct + x * st;
                
                di = round( rho*h.rho_scale + h.rho_offset);    % in the range 1 .. Nrho
                % which elements are within the column
                %d(d<0) = -d(d<0);
                %inrange = d<Nrho;

                di = di + col0;     % convert array of d values to Hough indices
                H(di) = H(di) + inc;    % increment the accumulator cells
            end

            nd2 = (h.Nrho-1)/2;
            h.rho = [-nd2:nd2]'/h.rho_scale;
        end % xyhough
    
        function display(h)
        %Hough.display Display value
        %
        % HT.display() displays a compact human-readable string representation of the 
        % Hough transform parameters.
        %
        % Notes::
        % - This method is invoked implicitly at the command line when the result
        %   of an expression is a Hough object and the command has no trailing
        %   semicolon.
        %
        % See also Hough.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            if loose
                disp(' ');
            end
            disp(char(h))
            if loose
                disp(' ');
            end
        end

        function s = char(h)
        %Hough.char Convert to string
        %
        % S = HT.char() is a compact string representation of the Hough transform parameters.

            s = sprintf('Hough: nd=%d, ntheta=%d, interp=%dx%d, distance=%d', ...
                h.Nrho, h.Ntheta, h.interpWidth, h.interpWidth, h.suppress);
        end
        

    end % methods
    
    methods(Static)
        function idx = nhood2ind(im, SE, centre)
            [y,x] = find(SE);

            sw = (numcols(SE)-1)/2;
            sh = (numrows(SE)-1)/2;
            x = x + centre(1)-sw-1;
            y = y + centre(2)-sh-1;

            w = numcols(im);
            h = numrows(im);

            y(x<1) = h - y(x<1);
            x(x<1) = x(x<1) + w;
            
            y(x>w) = h - y(x>w);
            x(x>w) = x(x>w) - w;
            


            idx = sub2ind(size(im), y, x);
            idx = reshape(idx, size(SE));
        end
    end % static methods
end % Hough
