%LineFeature Line feature class
%
% This class represents a line feature.
%
% Methods::
% plot            Plot the line segment
% seglength       Determine length of line segment
% display         Display value
% char            Convert value to string
%
% Properties::
% rho         Offset of the line
% theta       Orientation of the line
% strength    Feature strength
% length      Length of the line
%
% Properties of a vector of LineFeature objects are returned as a vector.
% If L is a vector (Nx1) of LineFeature objects then L.rho is an Nx1 vector
% of the rho element of each feature.
%
% Note::
%  - LineFeature is a reference object.
%  - LineFeature objects can be used in vectors and arrays
%
% See also Hough, RegionFeature, PointFeature.


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
classdef LineFeature < handle

    properties
        rho_
        theta_
        strength_
        length_
    end

    methods
        function h = LineFeature(rho, theta, strength, length)
        %LineFeature.LineFeature Create a line feature object
        %
        % L = LineFeature() is a line feature object with null parameters.
        %
        % L = LineFeature(RHO, THETA, STRENGTH) is a line feature object with 
        % the specified properties.  LENGTH is undefined.
        % 
        % L = LineFeature(RHO, THETA, STRENGTH, LENGTH) is a line feature object 
        % with the specified properties.
        %
        % L = LineFeature(L2) is a deep copy of the line feature L2.

            if isa(rho, 'LineFeature')
                % clone the passed object
                obj = rho;
                h.rho_ = obj.rho_;
                h.theta_ = obj.theta_;
                h.strength_ = obj.strength_;
                h.length_ = obj.length_;
                return
            end
            if nargin > 0
                h.rho_ = rho;
                h.theta_ = theta;
                h.strength_ = strength;
                if nargin > 3
                    h.length_ = length;
                end
            end
        end

        function val = rho(lines)
            val = [lines.rho_];
        end

        function val = theta(lines)
            val = [lines.theta_];
        end

        function val = strength(lines)
            val = [lines.strength_];
        end

        function val = length(lines)
            val = [lines.length_];
        end

        function display(h)
        %LineFeature.display Display value
        %
        % L.display() displays a compact human-readable representation of the feature.
        % If L is a vector then the elements are printed one per line.
        %
        % Notes::
        % - This method is invoked implicitly at the command line when the result
        %   of an expression is a LineFeature object and the command has no trailing
        %   semicolon.
        %
        % See also LineFeature.char.

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

        function ss = char(lines)
        %LineFeature.char Convert to string
        %
        % S = L.char() is a compact string representation of the line feature.
        % If L is a vector then the string has multiple lines, one per element.

            ss = [];
            for line=lines
                s = sprintf('theta=%g, rho=%g, strength=%g', ...
                    line.theta_, line.rho_, line.strength_);
                if ~isempty(line.length)
                    s = strcat(s, sprintf(', length=%d', line.length_));
                end
                ss = strvcat(ss, s);
            end
        end
                %fprintf(' intercept     theta   strength\n');
                %disp(p);
        
        function handles = plot(lines, varargin)
        %LineFeature.plot Plot line
        %
        % L.plot() overlay the line on current plot.
        %
        % L.plot(LS) as above but the optional line style arguments LS are
        % passed to plot.
        %
        % Notes::
        % - If L is a vector then each element is plotted.

            holdon = ishold;
            hold on

            % figure the x-axis scaling
            scale = axis;
            x = [scale(1):scale(2)]';
            y = [scale(3):scale(4)]';
            hl = [];

            % plot it
            for line=lines

                %fprintf('theta = %f, d = %f\n', line.theta, line.rho);
                if abs(cos(line.theta_)) > 0.5,
                    % horizontalish lines
                    %disp('hoz');
                    h = plot(x, -x*tan(line.theta_) + line.rho_/cos(line.theta_), varargin{:});
                else
                    % verticalish lines
                    %disp('vert');
                    h = plot( -y/tan(line.theta_) + line.rho_/sin(line.theta_), y, varargin{:});
                end
                hl = [hl h];
            end

            if ~holdon,
                hold off
            end

            if nargout > 0,
                handles = hl;
            end
            figure(gcf);        % bring it to the top
        end

        function out = seglength(lines, im_edge, gap)
        %LineFeature.seglength Compute length of line segments
        %
        % The Hough transform identifies lines but cannot determine their length.
        % This method examines the edge pixels in the original image and determines
        % the longest stretch of non-zero pixels along the line.
        %
        % L2 = L.seglength(EDGE, GAP) is a copy of the line feature object with the 
        % property length updated to the length of the line (pixels).  Small gaps, 
        % less than GAP pixels are tolerated.
        %
        % L2 = L.seglength(EDGE) as above but the maximum allowable gap is
        % 5 pixels.
        %
        % See also ICANNY.

            if nargin < 3
                gap = 5;
            end

            out = [];
            for L=lines
                %fprintf('d=%f, theta=%f; ', L.rho, L.theta)

                % find it's extreme points in the image
                if abs(L.theta) < pi/4
                    xmin = 1; xmax = numcols(im_edge);
                    m = -tan(L.theta); c = L.rho/cos(L.theta);
                    ymin = round(xmin*m + c);
                    ymax = round(xmax*m + c);
                else
                    ymin = 1; ymax = numrows(im_edge);
                    m = -1/tan(L.theta); c = L.rho/sin(L.theta);
                    xmin = round(ymin*m + c);
                    xmax = round(ymax*m + c);
                end


                line = bresenham(xmin, ymin, xmax, ymax);

                line = line(line(:,2)>=1,:);
                line = line(line(:,2)<=numrows(im_edge),:);
                line = line(line(:,1)>=1,:);
                line = line(line(:,1)<=numcols(im_edge),:);

                contig = 0;
                contig_max = 0;
                total = 0;
                missing = 0;
                for pp=line'
                    pix = im_edge(pp(2), pp(1));
                    if pix == 0
                        missing = missing+1;
                        if missing > gap
                            contig_max = max(contig_max, contig);
                            contig = 0;
                        end
                    else
                        contig = contig+1;
                        total = total+1;
                        missing = 0;
                    end
                    %ee(pp(2), pp(1))=1;
                end
                contig_max = max(contig_max, contig);

                %fprintf('  strength=%f, len=%f, total=%f\n', L.strength, contig_max, total);
                o = LineFeature(L);     % clone the object
                o.length_ = contig_max;
                out = [out o];
            end
        end
        
        function P = points(lines, im_edge)
        %LineFeature.points Return points on line segments
        %
        % P = L.points(EDGE) is the set of points that lie along the 
        % line in the edge image EDGE are determined.
        %
        % See also ICANNY.
        
        % TODO
        %  refactor this code with segLength


            P = [];
            for L=lines
                %fprintf('d=%f, theta=%f; ', L.rho, L.theta)

                % find it's extreme points in the image
                if abs(L.theta) < pi/4
                    xmin = 1; xmax = numcols(im_edge);
                    m = -tan(L.theta); c = L.rho/cos(L.theta);
                    ymin = round(xmin*m + c);
                    ymax = round(xmax*m + c);
                else
                    ymin = 1; ymax = numrows(im_edge);
                    m = -1/tan(L.theta); c = L.rho/sin(L.theta);
                    xmin = round(ymin*m + c);
                    xmax = round(ymax*m + c);
                end


                line = bresenham(xmin, ymin, xmax, ymax);

                line = line(line(:,2)>=1,:);
                line = line(line(:,2)<=numrows(im_edge),:);
                line = line(line(:,1)>=1,:);
                line = line(line(:,1)<=numcols(im_edge),:);

                i = sub2ind(size(im_edge), line(:,2), line(:,1));
                k = im_edge(i);
                P = [P line(k,:)'];
            end
        end
        
    end % methods
end % Hough
