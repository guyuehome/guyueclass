%POLYGON Polygon class
%
% A general class for manipulating polygons and vectors of polygons.
%
% Methods::
%  plot           Plot polygon
%  area           Area of polygon
%  moments        Moments of polygon
%  centroid       Centroid of polygon
%  perimeter      Perimter of polygon
%  transform      Transform polygon
%  inside         Test if points are inside polygon
%  intersection   Intersection of two polygons
%  difference     Difference of two polygons
%  union          Union of two polygons
%  xor            Exclusive or of two polygons
%  display        print the polygon in human readable form
%  char           convert the polgyon to human readable string
%
% Properties::
%  vertices   List of polygon vertices, one per column
%  extent     Bounding box [minx maxx; miny maxy]
%  n          Number of vertices
%
% Notes::
% - This is reference class object
% - Polygon objects can be used in vectors and arrays
%
% Acknowledgement::
%
% The methods: inside, intersection, difference, union, and xor are based on code
% written by:
%  Kirill K. Pankratov, kirill@plume.mit.edu,
%  http://puddle.mit.edu/~glenn/kirill/saga.html
% and require a licence.  However the author does not respond to email regarding
% the licence, so use with care, and modify with acknowledgement.

% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

% TODO
%  split the code in two.  Simple polygon functions in Polgon class, subclass with
%  Pankratov code to Polygon2.
%  add method to detect empty polygon, overload isempty

classdef Polygon < handle
    
    properties
        vertices
        extent
    end
    
    properties (Dependent=true)
        n
        x
        y
    end
    
    methods
        
        function p = Polygon(v, wh)
            %Polygon.Polygon Polygon class constructor
            %
            % P = Polygon(V) is a polygon with vertices given by V, one column per
            % vertex.
            %
            % P = Polygon(C, WH) is a rectangle centred at C with dimensions
            % WH=[WIDTH, HEIGHT].
            
            if nargin == 0
                p.n = 0;
                p.vertices = [];
                return;
            end
            
            if nargin < 2
                if numrows(v) ~= 2
                    error('vertices must have two rows');
                end
                p.vertices = v;
            end
            if nargin == 2
                if length(v) ~= 2
                    error('first argument must be polygon centre');
                end
                if length(wh) ~= 2
                    error('second arugment must be width height');
                end
                
                p.vertices = [
                    v(1)-wh(1)/2  v(1)+wh(1)/2  v(1)+wh(1)/2  v(1)-wh(1)/2
                    v(2)-wh(2)/2  v(2)-wh(2)/2  v(2)+wh(2)/2 v(2)+wh(2)/2 ];
            end
            
            % compute the extent
            p.extent(1,1) = min(p.x);
            p.extent(1,2) = max(p.x);
            p.extent(2,1) = min(p.y);
            p.extent(2,2) = max(p.y);
        end
        
        function r = get.n(p)
            r = numcols(p.vertices);
        end
        
        function r = get.x(p)
            r = p.vertices(1,:)';
        end
        
        function r = get.y(p)
            r = p.vertices(2,:)';
        end
        
        function r = set.n(p)
            error('cant set property');
        end
        function r = set.x(p)
            error('cant set property');
        end
        function r = set.y(p)
            error('cant set property');
        end
        
        function s = char(p)
            %Polygon.char String representation
            %
            % S = P.char() is a compact representation of the polgyon in human
            % readable form.
            s = sprintf('%d vertices', p.n);
        end
        
        function display(p)
            %Polygon.display Display polygon
            %
            % P.display() displays the polygon in a compact human readable form.
            %
            % See also Polygon.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            if loose
                disp(' ');
            end
            disp(char(p))
            if loose
                disp(' ');
            end
        end
        
        function plot(plist, varargin)
            %Polygon.plot Draw polygon
            %
            % P.plot() draws the polygon P in the current plot.
            %
            % P.plot(LS) as above but pass the arguments LS to plot.
            %
            % Notes::
            % - The polygon is added to the current plot.
            
            opt.fill = [];
            [opt,args] = tb_optparse(opt, varargin);
            
            ish = ishold
            hold all
            
            for p=plist
                % for every polygon in the list
                
                % get the vertices
                X = p.vertices(1,:)';
                Y = p.vertices(2,:)';
                
                while true
                    
                    % look for NaNs which indicate disjoint vertex sets
                    k = find(isnan(X));
                    
                    if length(k) > 0
                        % if a NaN chop out the segment before and after
                        k = k(1);
                        x = X(1:k-1);
                        y = Y(1:k-1);
                        X = X(k+1:end);
                        Y = Y(k+1:end);
                    else
                        x = X; y = Y;
                    end

                    % close the polygon
                    x = [x; x(1)];
                    y = [y; y(1)];
                    
                    if opt.fill
                        patch(x, y, opt.fill);
                    else
                        plot(x, y, args{:});
                    end
                    
                    if length(k) == 0
                        break;
                    end
                end
            end
            hold(ish)
        end
        
        function a = area(p)
            %Polygon.area Area of polygon
            %
            % A = P.area() is the area of the polygon.
            %
            % See also Polygon.moments.
            a = p.moments(0, 0);
        end
        
        function m = moments(p, mp, mq)
            %Polygon.moments Moments of polygon
            %
            % A = P.moments(p, q) is the pq'th moment of the polygon.
            %
            % See also Polygon.area, Polygon.centroid, mpq_poly.
            m = mpq_poly(p.vertices, mp, mq);
        end
        
        function q = transform(p, T)
            %Polygon.transform Transform polygon vertices
            %
            % P2 = P.transform(T) is a new Polygon object whose vertices have
            % been transformed by the SE(2) homgoeneous transformation T (3x3).
            if length(T) == 3
                T = se2(T);
            end
            q = Polygon( homtrans(T, p.vertices) );
        end
        
        function f = inside(p, points)
            %Polygon.inside Test if points are inside polygon
            %
            % IN = P.inside(P) tests if points given by columns of P (2xN) are inside
            % the polygon.  The corresponding elements of IN (1xN) are either true or
            % false.
            IN = inpolygon(points(1,:), points(2,:), p.x, p.y)
        end
        
        function c = centroid(p)
            %Polygon.centroid Centroid of polygon
            %
            % X = P.centroid() is the centroid of the polygon.
            %
            % See also Polygon.moments.
            xc = p.moments(1,1) / p.area();
        end
        
        function r = perimeter(p)
            %Polygon.perimeter Perimeter of polygon
            %
            % L = P.perimeter() is the perimeter of the polygon.
            p = sum(sqrt(diff(p.x).^2+diff(p.y).^2));
        end
        
        function f = intersect(p, plist)
            %Polygon.intersect Intersection of polygon with list of polygons
            %
            % I = P.intersect(PLIST) indicates whether or not the Polygon P
            % intersects with
            %
            % i(j) = 1 if p intersects polylist(j), else 0.
            
            % Based on ISINTPL
            %  Copyright (c) 1995 by Kirill K. Pankratov,
            %       kirill@plume.mit.edu.
            %       06/20/95, 08/25/95
            f = [];
            for q=plist
                f = [f isintpl(p.x, p.y, q.x, q.y)];
            end
        end
        
        function f = intersect_line(p, l)
            %Polygon.intersect_line Intersection of polygon and line segment
            %
            % I = P.intersect_line(L) is the intersection points of a polygon P with
            % the line segment L=[x1 x2; y1 y2].  I (2xN) has one column per
            % intersection, each column is [x y]'.
            
            f = [];
            % find intersections
            for i=1:p.n
                in = mod(i, p.n)+1;
                
                xv = [p.x(i); p.x(in)];
                yv = [p.y(i); p.y(in)];
                intsec = iscross(xv, yv, l(1,:)', l(2,:)');
                if intsec
                    [x,y] = intsecl(xv, yv, l(1,:)', l(2,:)');
                    f = [f [x;y]];
                end
            end
        end
        
        function r = difference(p, q)
            %Polygon.difference Difference of polygons
            %
            % D = P.difference(Q) is polygon P minus polygon Q.
            %
            % Notes::
            % - If polygons P and Q are not intersecting, returns
            %   coordinates of P.
            % - If the result D is not simply connected or consists of
            %   several polygons, resulting vertex list will contain NaNs.
            
            % POLYDIFF  Difference of 2 polygons.
            %       [XO,YO] = POLYDIFF(X1,Y1,X2,Y2) Calculates polygon(s) P
            %   of difference of polygons P1 and P1 with coordinates
            %   X1, Y1 and X2, Y2.
            %   The resulting polygon(s) is a set of all points which belong
            %   to P1 but not to to P2: P = P1 & ~P2.
            %   The input polygons must be non-self-intersecting and
            %   simply connected.
            %
            %       If polygons P1, P2 are not intersecting, returns
            %   coordinates of the first polygon X1, X2.
            %   If the result P is not simply connected or consists of several
            %   polygons, resulting boundary consists of concatenated
            %   coordinates of these polygons, separated by NaN.
            %  Copyright (c) 1995 by Kirill K. Pankratov,
            %       kirill@plume.mit.edu.
            %       06/25/95
            
            % Call POLYBOOL with flag=3
            [xo,yo,ind] = polybool(p.x, p.y, q.x, q.y, 3);
            
            r = Polygon([xo(:) yo(:)]');
        end
        
        function r = intersection(p, q)
            %Polygon.intersection Intersection of polygons
            %
            % I = P.intersection(Q) is a Polygon representing the
            % intersection of polygons P and Q.
            %
            % Notes::
            % - If these polygons are not intersecting, returns empty polygon.
            % - If intersection consist of several disjoint polygons
            %   (for non-convex P or Q) then vertices of I is the concatenation
            %   of the vertices of these polygons.
            
            
            % POLYINTS  Intersection of 2 polygons.
            %   [XO,YO] = POLYINTS(X1,Y1,X2,Y2) Calculates polygon(s)
            %   if intersection of polygons with coordinates X1, Y1
            %   and X2, Y2.
            %       The resulting polygon(s) is a set of all points which
            %   belong to both P1 and P2: P = P1 & P2.
            %   These polygons must be non-self-intersecting and
            %   simply connected.
            %
            %   If these polygons are not intersecting, returns empty.
            %   If intersection consist of several disjoint polygons
            %   (for non-convex P1 or P2) output vectors XO, YO consist
            %   of concatenated cooddinates of these polygons,
            %  Copyright (c) 1995 by Kirill K. Pankratov,
            %       kirill@plume.mit.edu.
            %       06/25/95
            
            
            % Call POLYBOOL with flag=1
            [xo,yo,ind] = polybool(p.x, p.y, q.x, q.y, 1);
            
            
            r = Polygon([xo(:) yo(:)]');
        end
        
        function r = union(p, q)
            %Polygon.union Union of polygons
            %
            % I = P.union(Q) is a polygon representing the
            % union of polygons P and Q.
            %
            % Notes::
            % - If these polygons are not intersecting, returns a polygon with
            %   vertices of both polygons separated by NaNs.
            % - If the result P is not simply connected (such as a polygon
            %   with a "hole") the resulting contour consist of counter-
            %   clockwise "outer boundary" and one or more clock-wise
            %   "inner boundaries" around "holes".
            
            % POLYUNI  Union of 2 polygons.
            %       [XO,YO] = POLYINT(X1,Y1,X2,Y2) Calculates polygon(s) P
            %       which is (are) union of polygons P1 and P2 with coordinates
            %   X1, Y1 and X2, Y2.
            %   The resulting polygon(s) is a set of all points which belong
            %   either to P1 or to P2: P = P1 | P2.
            %   The input polygons must be non-self-intersecting and
            %   simply connected.
            %
            %       If polygons P1, P2 are not intersecting, returns
            %   coordinates of the both polygons separated by NaN.
            %   If both P1 and P2 are convex, their boundaries can have no
            %   more than 2 intersections. The result is also a convex
            %   polygon.
            %       If the result P is not simply connected (such as a polygon
            %   with a "hole") the resulting contour consist of counter-
            %   clockwise "outer boundary" and one or more clock-wise
            %   "inner boundaries" around "holes".
            %  Copyright (c) 1995 by Kirill K. Pankratov,
            %       kirill@plume.mit.edu.
            %       06/25/95
            
            
            % Call POLYBOOL with flag=2 ..........
            [xo,yo,ind] = polybool(p.x, p.y, q.x, q.y, 2);
            
            
            r = Polygon([xo(:) yo(:)]');
        end
        
        function r = xor(p, q)
            %Polygon.xor Exclusive or of polygons
            %
            % I = P.union(Q) is a polygon representing the
            % exclusive-or of polygons P and Q.
            %
            % Notes::
            % - If these polygons are not intersecting, returns a polygon with
            %   vertices of both polygons separated by NaNs.
            % - If the result P is not simply connected (such as a polygon
            %   with a "hole") the resulting contour consist of counter-
            %   clockwise "outer boundary" and one or more clock-wise
            %   "inner boundaries" around "holes".
            
            % POLYXOR  Exclusive OR of 2 polygons.
            %       [XO,YO] = POLYXOR(X1,Y1,X2,Y2) Calculates polygon(s) P
            %   of difference of polygons P1 and P1 with coordinates
            %   X1, Y1 and X2, Y2.
            %   The resulting polygon(s) is a set of all points which belong
            %   either to P1 or to P2 but not to both:
            %   P = (P1 & ~P2) | (P2 & ~P1).
            %   The input polygons must be non-self-intersecting and
            %   simply connected.
            %
            %       If polygons P1, P2 are not intersecting, returns
            %   coordinates of the both polygons separated by NaN.
            %   If the result P is not simply connected or consists of several
            %   polygons, resulting boundary consists of concatenated
            %   coordinates of these polygons, separated by NaN.
            
            %  Copyright (c) 1995 by Kirill K. Pankratov,
            %       kirill@plume.mit.edu.
            %       06/25/95
            
            
            % Call POLYBOOL twice with flag=3
            [xx,yy,ind] = polybool(p.x, p.y, q.x, q.y, 3);
            
            xo = [xx; NaN]; yo = [yy; NaN];
            [xx,yy,ind] = polybool(q.x, q.y, p.x, p.y, 3);
            
            xo = [xo; xx]; yo = [yo; yy];
            
            r = Polygon([xo(:) yo(:)]');
            
            
        end
    end % methods
end % classdef


function [is,in,un] = interval(x1,x2)
    
    % Intersection and union of 2 intervals.
    %   [IS,IN,UN] = INTERVAL(X1,X2) calculates pair-wise
    %   intersection IN and union UN of N pairs of
    %   intervals with coordinates X1 and X2 (both are
    %   2 by N vectors). Returns 1 by N boolean vector IS
    %   equal to 1 if intervals have non-empty intersection
    %   and 0 if they don't.
    
    %  Copyright (c) 1995 by Kirill K. Pankratov,
    %       kirill@plume.mit.edu.
    %       08/24/95
    
    % Handle input ...........................
    if nargin==0, help interval, return, end
    if nargin==1
        un = x1;
    else
        un = [x1; x2];
    end
    
    [in,un] = sort(un);     % Sort both intervals together
    un = un(1:2,:)-1;
    is = sum(floor(un/2));  % Check for [0 0 1 1] or [1 1 0 0]
    is = (is==1);
    ii = find(in(2,:)==in(3,:));
    is(ii) = .5*ones(size(ii));
    
    % Extract intersection and union from sorted coordinates
    if nargout>1
        un = in([1 4],:);
        in = in(2:3,:);
        in(:,~is) = flipud(in(:,~is));
    end
end

function [is,S] = iscross(x1,y1,x2,y2,tol)
    
    % ISCROSS  Finds whether pairs of lines cross each other
    %   [IS,S] = ISCROSS(X1,Y1,X2,Y2) where arguments X1, Y1,
    %   X2, Y2 are all 2 by N matrices are coordinates of
    %   ends of the pairs of line segments.
    %   Returns vector  IS (1 by N)  consisting of ones if
    %   corresponding pairs cross each other, zeros if they
    %   don't and .5 if an end of one line segment lies on
    %   another segment.
    %   Also returns a matrix  S (4 by N) with each row
    %   consisting of cross products (double areas of
    %   corresponding triangles) built on the following points:
    %   (X2(1,:),Y2(1,:)),(X1(1,:),Y1(1,:)),(X2(2,:),Y2(2,:)),
    %   (X2(1,:),Y2(1,:)),(X1(2,:),Y1(2,:)),(X2(2,:),Y2(2,:))
    %   (X1(1,:),Y1(1,:)),(X2(1,:),Y2(1,:)),(X1(2,:),Y1(2,:))
    %   (X1(1,:),Y1(1,:)),(X2(2,:),Y2(2,:)),(X1(2,:),Y1(2,:))
    %   The signs of these 4 areas can be used to determine
    %   whether these lines and their continuations cross each
    %   other.
    %   [IS,S] = ISCROSS(X1,Y1,X2,Y2,TOL) uses tolerance TOL
    %   for detecting the crossings (default is 0).
    
    %  Copyright (c) 1995 by Kirill K. Pankratov
    %       kirill@plume.mit.edu
    %       08/14/94, 05/18/95, 08/25/95
    
    % Defaults and parameters .......................
    tol_dflt = 0; % Tolerance for area calculation
    is_chk = 1;   % Check input arguments
    
    % Handle input ..................................
    if nargin==0, help iscross, return, end
    if nargin<4           % Check if all 4 entered
        error('  Not enough input arguments')
    end
    if nargin<5, tol = tol_dflt; end
    if tol < 0, is_chk = 0; tol = 0; end
    
    % Check the format of arguments .................
    if is_chk
        [x1,y1,x2,y2] = linechk(x1,y1,x2,y2);
    end
    
    len = size(x1,2);
    o2 = ones(2,1);
    
    % Find if the ranges of pairs of segments intersect
    [isx,S,A] = interval(x1,x2);
    scx = diff(A);
    [isy,S,A] = interval(y1,y2);
    scy = diff(A);
    is = isx & isy;
    
    % If S values are not needed, extract only those pairs
    % which have intersecting ranges ..............
    if nargout < 2
        isx = find(is);  % Indices of pairs to be checked
        % further
        x1 = x1(:,isx);
        x2 = x2(:,isx);
        y1 = y1(:,isx);
        y2 = y2(:,isx);
        is = is(isx);
        if isempty(is), is = zeros(1,len); return, end
        scx = scx(isx);
        scy = scy(isx);
    end
    
    % Rescale by ranges ...........................
    x1 = x1.*scx(o2,:);
    x2 = x2.*scx(o2,:);
    y1 = y1.*scy(o2,:);
    y2 = y2.*scy(o2,:);
    
    
    % Calculate areas .............................
    S = zeros(4,length(scx));
    S(1,:) = (x2(1,:)-x1(1,:)).*(y2(2,:)-y1(1,:));
    S(1,:) = S(1,:)-(x2(2,:)-x1(1,:)).*(y2(1,:)-y1(1,:));
    
    S(2,:) = (x2(1,:)-x1(2,:)).*(y2(2,:)-y1(2,:));
    S(2,:) = S(2,:)-(x2(2,:)-x1(2,:)).*(y2(1,:)-y1(2,:));
    
    S(3,:) = (x1(1,:)-x2(1,:)).*(y1(2,:)-y2(1,:));
    S(3,:) = S(3,:)-(x1(2,:)-x2(1,:)).*(y1(1,:)-y2(1,:));
    
    S(4,:) = (x1(1,:)-x2(2,:)).*(y1(2,:)-y2(2,:));
    S(4,:) = S(4,:)-(x1(2,:)-x2(2,:)).*(y1(1,:)-y2(2,:));
    
    
    % Find if they cross each other ...............
    is = (S(1,:).*S(2,:)<=0)&(S(3,:).*S(4,:)<=0);
    
    
    % Find very close to intersection
    isy = min(abs(S));
    ii = find(isy<=tol & is);
    is(ii) = .5*ones(size(ii));
    
    % Output
    if nargout < 2
        isy = zeros(1,len);
        isy(isx) = is;
        is = isy;
        
    else
        isy = scx.*scy;
        ii = find(~isy);
        isy(ii) = ones(size(ii));
        S = S./isy(ones(4,1),:);
        
    end
    
end

function [xo,yo,ind] = polybool(x1,y1,x2,y2,flag)
    
    %   [XO,YO] = POLYBOOL(X1,Y1,X2,Y2,FLAG)
    %   calulates results of Boolean operations on
    %   a pair of polygons.
    %   FLAG Specifies the type of the operation:
    %   1 - Intersection (P1 & P2)
    %   2 - Union (P1 | P2)
    %   3 - Difference (P1 & ~P2)
    
    %  Copyright (c) 1995 by Kirill K. Pankratov,
    %       kirill@plume.mit.edu.
    %       06/25/95, 09/07/95
    
    %   This program calls the following functions:
    %   AREA, ISINTPL, ISCROSS, INTSECL.
    
    % Algorithm:
    %  1. Check boundary contour directions (area).
    %     For intersection and union make all
    %     counter-clockwise. For difference make the second
    %    contour clock-wise.
    %  2. Calculate matrix of intersections (function ISINTPL).
    %     Quick exit if no intersections.
    %  3. For intersecting segments calculate intersection
    %     coordinates (function INTSECL).
    %  4. Sort intersections along both contours.
    %  5. Calculate sign of cross-product between intersectiong
    %     segments. This will give which contour goes "in" and
    %     "out" at intersections.
    %
    %  6. Start with first intersection:
    %     Determine direction to go ("in" for intersection,
    %     "out" for union).
    %     Move until next intersection, switch polygons at each
    %     intersection until coming to the initial point.
    %     If not all intersections are encountered, the
    %     resulting polygon is disjoint. Separate output
    %     coordinates by NaN and repeat procedure until all
    %     intersections are counted.
    
    % Default for flag
    flag_dflt = 1; % 1- intersec., 2-union, 3 - diff.
    
    % Handle input
    if nargin==0, help polybool, return, end
    if nargin < 4
        error(' Not enough input arguments')
    end
    if nargin<5, flag = flag_dflt; end
    
    x1 = x1(:); y1 = y1(:);
    x2 = x2(:); y2 = y2(:);
    l1 = length(x1);
    l2 = length(x2);
    
    % Check areas and reverse if negative
    nn1 = area(x1,y1);
    if nn1<0, x1 = flipud(x1); y1 = flipud(y1); end
    nn2 = area(x2,y2);
    if (nn2<0 & flag<3) | (nn2>0 & flag==3)
        x2 = flipud(x2); y2 = flipud(y2);
    end
    
    % If both polygons are identical ........
    if l1==l2
        if all(x1==x2) & all(y1==y2)
            if flag<3, xo = x1; yo = y1; ind = 1:l1;
            else, xo = []; yo = []; ind = []; end
            return
        end
    end
    
    % Calculate matrix of intersections .....
    [is,C] = isintpl(x1,y1,x2,y2);
    is = any(any(C));
    
    % Quick exit if no intersections ........
    if ~is
        if flag==1       % Intersection
            xo=[]; yo = [];
        elseif flag==2   % Union
            xo = [x1; nan; x2];
            yo = [y1; nan; y2];
        elseif flag==3   % Difference
            xo = x1; yo = y1;
        end
        return
    end
    
    % Mark intersections with unique numbers
    i1 = find(C);
    ni = length(i1);
    C(i1) = 1:ni;
    
    % Close polygon contours
    x1 = [x1; x1(1)]; y1 = [y1; y1(1)];
    x2 = [x2; x2(1)]; y2 = [y2; y2(1)];
    l1 = length(x1);  l2 = length(x2);
    
    % Calculate intersections themselves
    [i1,i2,id] = find(C);
    xs1 = [x1(i1) x1(i1+1)]'; ys1 = [y1(i1) y1(i1+1)]';
    xs2 = [x2(i2) x2(i2+1)]'; ys2 = [y2(i2) y2(i2+1)]';
    
    % Call INTSECL ............................
    [xint,yint] = intsecl(xs1,ys1,xs2,ys2);
    
    % For sements belonging to the same line
    % find interval of intersection ...........
    ii = find(xint==inf);
    if ~isempty(ii)
        [is,inx] = interval(xs1(:,ii),xs2(:,ii));
        [is,iny] = interval(ys1(:,ii),ys2(:,ii));
        xint(ii) = mean(inx);
        yint(ii) = mean(iny);
    end
    
    % Coordinate differences of intersecting segments
    xs1 = diff(xs1); ys1 = diff(ys1);
    xs2 = diff(xs2); ys2 = diff(ys2);
    
    % Calculate cross-products
    cp = xs1.*ys2-xs2.*ys1;
    cp = cp>0;
    if flag==2, cp=~cp; end % Reverse if union
    cp(ii) = 2*ones(size(ii));
    
    % Sort intersections along the contours
    ind = (xint-x1(i1)').^2+(yint-y1(i1)').^2;
    ind = ind./(xs1.^2+ys1.^2);
    cnd = min(ind(ind>0));
    ind = ind+i1'+i2'/(ni+1)*cnd*0;
    [xo,ii] = sort(ind);
    xs1 = id(ii);
    [xo,ind] = sort(xs1);
    ind = rem(ind,ni)+1;
    xs1 = xs1(ind);
    
    ind = (xint-x2(i2)').^2+(yint-y2(i2)').^2;
    ind = ind./(xs2.^2+ys2.^2);
    cnd = min(ind(ind>0));
    [xo,ii] = sort(i2'+ind+i1'/(ni+1)*cnd*0);
    xs2 = id(ii);
    [xo,ind] = sort(xs2);
    ind = rem(ind,ni)+1;
    xs2 = xs2(ind);
    
    % Combine coordinates in one vector
    x1 = [x1; x2]; y1 = [y1; y2];
    
    % Find max. possible length of a chain
    xo = find(any(C'));
    xo = diff([xo xo(1)+l1]);
    mlen(1) = max(xo);
    xo = find(any(C));
    xo = diff([xo xo(1)+l2]);
    mlen(2) = max(xo);
    
    % Check if multiple intersections in one segment
    xo = diff([i1 i2]);
    is_1 = ~all(all(xo));
    
    % Begin counting intersections *********************
    
    % Initialization ..................
    int = zeros(size(xint));
    nn = 1;   % First intersection
    nn1 = i1(nn); nn2 = i2(nn);
    b = cp(nn);
    is2 = b==2;
    xo = []; yo = []; ind = [];
    closed = 0;
    
    % Proceed until all intersections are counted
    while ~closed  % begin counting `````````````````````0
        
        % If contour closes, find new starting point
        if int(nn) & ~all(int)
            ii = find(int);
            C(id(ii)) = zeros(size(ii));
            nn = min(find(~int));  % Next intersection
            nn1 = i1(nn);
            nn2 = i2(nn);
            xo = [xo; nan];        % Separate by NaN
            yo = [yo; nan];
            ind = [ind; nan];
            % Choose direction ......
            b = cp(nn);
        end
        
        % Add current intersection ......
        xo = [xo; xint(nn)];
        yo = [yo; yint(nn)];
        ind = [ind; 0];
        int(nn) = 1;
        closed = all(int);
        
        % Find next segment
        % Indices for next intersection
        if ~b, nn = xs1(nn);
        else,  nn = xs2(nn);
        end
        if ~b, pt0 = nn1; else,  pt0 = nn2; end
        
        nn1 = i1(nn);
        nn2 = i2(nn);
        
        if b, pt = nn2; else, pt = nn1; end
        
        if b, pt0 = pt0+l1; pt = pt+l1; end
        ii = (pt0+1:pt);
        
        
        % Go through the beginning ..............
        cnd = pt<pt0 | (pt==pt0 & is_1 & flag>1);
        if cnd
            if ~b,  ii = [pt0+1:l1 1:pt];
            else,   ii = [pt0+1:l1+l2 l1+1:pt];
            end
        end
        len = length(ii);
        cnd = b & len>mlen(2);
        cnd = cnd | (~b & len>mlen(1));
        if is2 | cnd, ii=[]; end
        
        
        % Add new segment
        xo = [xo; x1(ii)];
        yo = [yo; y1(ii)];
        ind = [ind; ii'];
        
        % Switch direction
        if cp(nn)==2, b = ~b; is2 = 1;
        else, b = cp(nn); is2 = 0;
        end
        
    end    % End while (all intersections) '''''''''''''''0
    
    % Remove coincident successive points
    ii = find(~diff(xo) & ~diff(yo));
    xo(ii) = []; yo(ii) = []; ind(ii) = [];
    
    % Remove points which are
    ii = find(isnan(xo));
    if ~isempty(ii)
        i2 = ones(size(xo));
        ii = [ii; length(xo)+1];
        
        i1 = find(diff(ii)==3);
        i1 = ii(i1);
        i1 = [i1; i1+1; i1+2];
        i2(i1) = zeros(size(i1));
        
        i1 = find(diff(ii)==2);
        i1 = ii(i1);
        i1 = [i1; i1+1];
        i2(i1) = zeros(size(i1));
        
        xo = xo(i2); yo = yo(i2); ind = ind(i2);
    end
end

function  [xo,yo] = intsecpl(xv,yv,xl,yl,trace)
    
    % INTSECPL Intersection of a polygon and a line.
    %   [XI,YI] = INTSECPL(XV,YV,XL,YL) calculates
    %   intersections XI, YI of a polygon with vertices XV,
    %   YV and a line specified by pairs of end coordinates
    %   XL = [XL0 XL1], YL = [YL0 YL1]. Line is assumed to
    %   continue beyond the range of end points.
    %   INTSECPL(XV,YV,[A B]) uses another specification for
    %   a line: Y = A*X+B.
    %
    %   If a line does not intersect polygon, returns empty
    %   XI, YI.
    %   For convex polygon maximum number of intersections is
    %   2, for non-convex polygons multiple intersections are
    %   possible.
    %
    %   INTSECPL(XV,YV,XL,YL)  by itself or
    %   [XI,YI] = INTSECPL(XV,YV,XL,YL,1) plots polygon,
    %   a line segment and intersection segment(s)
    %   (part(s) of the same line inside the polygon).
    
    %  Copyright (c) 1995 by Kirill K. Pankratov,
    %       kirill@plume.mit.edu.
    %       06/25/95, 08/27/95, 09/27/95
    
    %  Calls ISCROSS, INTSECL programs.
    
    
    % Defaults and parameters .................................
    tol = 1e-14;  % Tolerance
    marg = tol;   % Margins for polygon frame
    is_ab = 0;    % Default A*X+B  mode
    
    % Handle input ............................................
    if nargin==0, help intsecpl, return, end
    if nargin < 3
        error(' Not enough input arguments')
    end
    if nargin<5, trace = 0; end
    if nargin==4  % Check if 4-th arg is trace
        if max(size(yl))==1, trace = yl;  is_ab = 1; end
    end
    if nargin==3, is_ab = 1; end
    trace = trace | nargin<2;
    if length(xv)~=length(yv)
        error(' Vectors X, Y must have the same size')
    end
    
    % Auxillary ...........
    xv = [xv(:); xv(1)];
    yv = [yv(:); yv(1)];
    ii = find(abs(diff(xv))<tol & abs(diff(yv))<tol);
    xv(ii) = []; yv(ii) = [];
    nv = length(xv);
    ov = ones(nv-1,1);
    
    
    % Polygon frame
    lim = [min(xv)-marg max(xv)+marg min(yv)-marg max(yv)+marg];
    % Estimate for diameter
    d = sqrt((lim(2)-lim(1)).^2+(lim(4)-lim(3)).^2);
    
    
    % Form line segment depending on how line is specified
    if is_ab       % A*X+B  mode ...................
        
        xl = xl(:);
        if length(xl)<2
            error(' Line is specified by at least two parameters')
        end
        
        a = xl(1); b = xl(2);
        xl = [lim(1)-1 lim(2)+1];
        yl = a*xl+b;
        
    else          % [X1 X2],[Y1 Y2]  mode ..........
        
        x0 = (xl(1)+xl(2))/2;
        y0 = (yl(1)+yl(2))/2;
        dx = xl(2)-x0;
        dy = yl(2)-y0;
        dl = max(abs([lim(1:2)-x0 lim(3:4)-y0]));
        d = max(d,dl);
        dl = sqrt(dx.^2+dy.^2);
        dl = max(d,d/dl);
        dx = dx*dl; dy = dy*dl;
        xl = [x0-dx x0+dx];
        yl = [y0-dy y0+dy];
        
    end
    
    
    % Find intersecting segments ..............................
    is = iscross([xv(1:nv-1) xv(2:nv)]',[yv(1:nv-1) yv(2:nv)]',...
        xl(ov,:)',yl(ov,:)',0);
    
    
    % Quick exit if no intersections .........................
    if ~any(is)
        if trace
            % Intersection with polygon frame
            [xl,yl] = intsecpl(lim([1 2 2 1]),lim([3 3 4 4]),xl,yl);
            %plot(xv,yv,xl,yl) % Plotting itself
        end
        xo = []; yo = [];
        return
    end
    
    
    % For segments touching the line (is==.5) find whether pairs of
    % successive segments are on the same side
    ii = find(is==.5)';
    if ii~=[]
        xo = [ii-1 ii+1];
        xo = xo+(nv-1)*(xo<1);
        yo = iscross([xv(xo(:,1)) xv(xo(:,2))]',[yv(xo(:,1)) yv(xo(:,2))]',...
            xl,yl,tol);
        ii = ii(find(yo==1));
        is(ii) = zeros(size(ii));
    end
    
    
    % Calculate intersection coordinates ......................
    ii = find(is);
    oi = ones(size(ii));
    [xo,yo] = intsecl([xv(ii) xv(ii+1)]',[yv(ii) yv(ii+1)]',...
        xl(oi,:)',yl(oi,:)');
    
    dx = find(~finite(xo));
    xo(dx) = []; yo(dx) = [];
    ii(dx) = []; oi(dx) = [];
    
    
    % Sort intersections along the line ..........
    xo = xo(:); yo = yo(:);
    if any(diff(xo))
        [xo,ii] = sort(xo);
        yo = yo(ii);
        
    else
        
        [yo,ii] = sort(yo);
        xo = xo(ii);
    end
    
    % Exclude repeated points (degenerate cases)
    % ///////// It seems that this is not needed, figure this
    % later ///////////////////
    if length(ii)>1 & 0  % Do not execute
        xx = [xo yo];
        yy = diff(xx)';
        ii = [1 find(any(abs(yy)>tol))+1];
        xo = xx(ii,1); yo = xx(ii,2);
        oi = ones(size(xo));
    end
    
    
    % Plotting ................................................
    if trace
        oi(3:2:length(oi)) = oi(3:2:length(oi))+1;
        oi = cumsum(oi);
        len = max(oi);
        xp = nan*ones(len,1); yp = xp;
        xp(oi) = xo;
        yp(oi) = yo;
        
        % Intersection with polygon frame
        [xl,yl] = intsecpl(lim([1 2 2 1]),lim([3 3 4 4]),xl,yl);
        
        plot(xv,yv,xl,yl,xp,yp) % Plotting itself
    end
end

function [is,S] = isintpl(x1,y1,x2,y2)
    
    % ISINTPL Check for intersection of polygons.
    %   [IS,S] = ISINTPL(X1,Y1,X2,Y2) Accepts coordinates
    %   X1,Y1 and X2, Y2 of two polygons. Returns scalar
    %   IS equal to 1 if these polygons intersect each other
    %   and 0 if they do not.
    %   Also returns Boolean matrix S of the size length(X1)
    %   by length(X2) so that {ij} element of which is 1 if
    %   line segments i to i+1 of the first polygon and
    %   j to j+1 of the second polygon intersect each other,
    %   0 if they don't and .5 if they "touch" each other.
    
    %  Copyright (c) 1995 by Kirill K. Pankratov,
    %       kirill@plume.mit.edu.
    %       06/20/95, 08/25/95
    
    
    % Handle input ...................................
    if nargin==0, help isintpl, return, end
    if nargin<4
        error(' Not enough input arguments ')
    end
    
    % Make column vectors and check sizes ............
    x1 = x1(:); y1 = y1(:);
    x2 = x2(:); y2 = y2(:);
    l1 = length(x1);
    l2 = length(x2);
    if length(y1)~=l1 | length(y2)~=l2
        error('(X1,Y1) and (X2,Y2) must pair-wise have the same length.')
    end
    
    % Quick exit if empty
    if l1<1 | l2<1, is = []; S = []; return, end
    
    % Check if their ranges are intersecting .........
    lim1 = [min(x1) max(x1)]';
    lim2 = [min(x2) max(x2)]';
    isx = interval(lim1,lim2);  % X-ranges
    lim1 = [min(y1) max(y1)]';
    lim2 = [min(y2) max(y2)]';
    isy = interval(lim1,lim2);  % Y-ranges
    is = isx & isy;
    S = zeros(l2,l1);
    
    if ~is, return, end  % Early exit if disparate limits
    
    % Indexing .......................................
    [i11,i12] = meshgrid(1:l1,1:l2);
    [i21,i22] = meshgrid([2:l1 1],[2:l2 1]);
    i11 = i11(:); i12 = i12(:);
    i21 = i21(:); i22 = i22(:);
    
    % Calculate matrix of intersections ..............
    S(:) = iscross([x1(i11) x1(i21)]',[y1(i11) y1(i21)]',...
        [x2(i12) x2(i22)]',[y2(i12) y2(i22)]')';
    
    S = S';
    is  = any(any(S));
end

function [xo,yo] = intsecl(x1,y1,x2,y2,tol)
    
    % INTSECL Intersection coordinates of two line segments.
    %       [XI,YI] = INTSECL(X1,Y1,X2,Y2) where all 4
    %   arguments are 2 by N matrices with coordinates
    %   of ends of N pairs of line segments (so that
    %       the command such as PLOT(X1,Y1,X2,Y2) will plot
    %       these pairs of lines).
    %       Returns 1 by N vectors XI and YI consisting of
    %       coordinates of intersection points of each of N
    %   pairs of lines.
    %
    %   Special cases:
    %   When a line segment is degenerate into a point
    %   and does not lie on line through the other segment
    %   of a pair returns XI=NaN while YI has the following
    %   values: 1 - when the first segment in a pair is
    %   degenerate, 2 - second segment, 0 - both segments
    %   are degenerate.
    %   When a pair of line segments is parallel, returns
    %   XI = Inf while YI is 1 for coincident lines,
    %   0 - for parallel non-coincident ones.
    %   INTSECL(X1,Y1,X2,Y2,TOL) also specifies tolerance
    %   in detecting coincident points in different line
    %   segments.
    
    %  Copyright (c) 1995 by Kirill K. Pankratov
    %       kirill@plume.mit.edu
    %       04/15/94, 08/14/94, 05/10/95, 08/23/95
    
    
    % Defaults and parameters .........................
    tol_dflt = 0; % Tolerance for coincident points
    is_chk = 1;   % Check input arguments
    
    % Handle input ....................................
    if nargin==0, help intsecl, return, end
    if nargin<4           % Check if all 4 entered
        error('  Not enough input arguments')
    end
    if nargin<5, tol = tol_dflt; end
    if tol < 0, is_chk = 0; tol = 0; end
    
    % Check the format of arguments .......
    if is_chk
        [x1,y1,x2,y2] = linechk(x1,y1,x2,y2);
    end
    
    
    % Auxillary
    o2 = ones(2,1);
    i_pt1 = []; i_pt2 = []; i_pt12 = [];
    
    % Make first points origins ...........
    xo = x1(1,:);
    yo = y1(1,:);
    x2 = x2-xo(o2,:);
    y2 = y2-yo(o2,:);
    
    % Differences of first segments .......
    a = x1(2,:)-x1(1,:);
    b = y1(2,:)-y1(1,:);
    s = sqrt(a.^2+b.^2);  % Lengths of first segments
    i_pt1 = find(~s);
    s(i_pt1) = ones(size(i_pt1));
    rr = rand(size(i_pt1));
    a(i_pt1) = cos(rr);
    b(i_pt1) = sin(rr);
    
    % Normalize by length .................
    a = a./s; b = b./s;
    
    % Rotate coordinates of the second segment
    tmp = x2.*a(o2,:)+y2.*b(o2,:);
    y2 = -x2.*b(o2,:)+y2.*a(o2,:);
    x2 = tmp;
    
    % Calculate differences in second segments
    s = x2(2,:)-x2(1,:);
    tmp = y2(2,:)-y2(1,:);
    cc = tmp(i_pt1);
    
    % Find some degenerate cases .......................
    
    % Find zeros in differences
    izy2 = find(~tmp);
    tmp(izy2) = ones(size(izy2));
    
    % Find degenerate and parallel segments
    bool = ~s(izy2);
    i_par = izy2(~bool);
    i_pt2 = izy2(bool);
    
    bool = abs(y2(1,i_pt2))<=tol;
    i_pt2_off = i_pt2(~bool);
    i_pt2_on = i_pt2(bool);
    
    if ~isempty(i_par)
        bool = abs(y2(1,i_par))<=tol;
        i_par_off = i_par(~bool);
        i_par_on = i_par(bool);
    end
    
    % Calculate intercept with rotated x-axis ..........
    tmp = s./tmp;   % Slope
    tmp = x2(1,:)-y2(1,:).*tmp;
    
    
    % Rotate and translate back to original coordinates
    xo = tmp.*a+xo;
    yo = tmp.*b+yo;
    
    % Mark special cases ...................................
    % First segments are degenerate to points
    if ~isempty(i_pt1)
        bool = ~s(i_pt1) & ~cc;
        i_pt12 = i_pt1(bool);
        i_pt1 = i_pt1(~bool);
        
        bool = abs(tmp(i_pt1))<=tol;
        i_pt1_on = i_pt1(bool);
        i_pt1_off = i_pt1(~bool);
        
        xo(i_pt1_on) = x1(1,i_pt1_on);
        yo(i_pt1_on) = y1(1,i_pt1_on);
        
        oo = ones(size(i_pt1_off));
        xo(i_pt1_off) = nan*oo;
        yo(i_pt1_off) = oo;
    end
    
    % Second segments are degenerate to points ...
    if ~isempty(i_pt2)
        oo = ones(size(i_pt2_off));
        xo(i_pt2_off) = nan*oo;
        yo(i_pt2_off) = 2*oo;
    end
    
    % Both segments are degenerate ...............
    if ~isempty(i_pt12)
        bool = x1(i_pt12)==xo(i_pt12);
        i_pt12_on = i_pt12(bool);
        i_pt12_off = i_pt12(~bool);
        
        xo(i_pt12_on) = x1(1,i_pt12_on);
        yo(i_pt12_on) = y1(1,i_pt12_on);
        
        oo = ones(size(i_pt12_off));
        xo(i_pt12_off) = nan*oo;
        yo(i_pt12_off) = 0*oo;
    end
    
    % Parallel segments .........................
    if ~isempty(i_par)
        oo = ones(size(i_par_on));
        xo(i_par_on) = inf*oo;
        yo(i_par_on) = oo;
        
        oo = ones(size(i_par_off));
        xo(i_par_off) = inf*oo;
        yo(i_par_off) = 0*oo;
    end
    
    
    
end

function [x1,y1,x2,y2] = linechk(x1,y1,x2,y2)
    
    % LINECHK Input checking for line segments.
    
    %  Copyright (c) 1995 by Kirill K. Pankratov
    %       kirill@plume.mit.edu
    %       08/22/95,
    
    % String for transposing
    str = ['x1=x1'';'; 'y1=y1'';'; 'x2=x2'';'; 'y2=y2'';'];
    
    % Sizes
    sz = [size(x1); size(y1); size(x2); size(y2)]';
    psz = prod(sz);
    
    % Check x1, y1
    if psz(1)~=psz(2)
        error('  Arguments  x1 and y1 must have the same size')
    end
    
    % Check x2, y2
    if psz(3)~=psz(3)
        error('  Arguments  x2 and y2 must have the same size')
    end
    
    % Check if any arguments are less than 2 by 1
    if any(max(sz)<2)
        error('  Arguments  x1, y1, x2, y2 must be at least 2 by 1 vectors')
    end
    
    % Check if no size is equal to 2
    if any(all(sz~=2))
        error('  Arguments  x1, y1, x2, y2 must be 2 by 1 vectors')
    end
    
    % Find aruments to be transposed .............................
    ii = find(sz(1,:)~=2);
    for jj = 1:length(ii)
        eval(str(ii(jj),:)); % Transpose if neccessary
    end
    sz(:,ii) = flipud(sz(:,ii));
    
    % If vectors, extend to 2 by n matrices ......................
    n = max(sz(2,:));
    on = ones(1,n);
    if sz(2,1)<n, x1 = x1(:,on); end
    if sz(2,2)<n, y1 = y1(:,on); end
    if sz(2,3)<n, x2 = x2(:,on); end
    if sz(2,4)<n, y2 = y2(:,on); end
end

