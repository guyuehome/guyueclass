%PGraph Graph class
%
%   g = PGraph()        create a 2D, planar embedded, directed graph
%   g = PGraph(n)       create an n-d, embedded, directed graph
%
% Provides support for graphs that:
%   - are directed
%   - are embedded in a coordinate system
%   - have symmetric cost edges (A to B is same cost as B to A)
%   - have no loops (edges from A to A)
%   - have vertices that are represented by integers VID
%   - have edges that are represented by integers EID
%
% Methods::
%
% Constructing the graph::
%   g.add_node(coord)      add vertex, return vid
%   g.add_edge(v1, v2)     add edge from v1 to v2, return eid
%   g.setcost(e, c)        set cost for edge e
%   g.setdata(v, u)        set user data for vertex v
%   g.data(v)              get user data for vertex v
%   g.clear()              remove all vertices and edges from the graph
%
% Information from graph::
%   g.edges(v)             list of edges for vertex v
%   g.cost(e)              cost of edge e
%   g.neighbours(v)        neighbours of vertex v
%   g.component(v)         component id for vertex v
%   g.connectivity()       number of edges for all vertices
%
% Display::
%
%   g.plot()                   set goal vertex for path planning
%   g.highlight_node(v)        highlight vertex v
%   g.highlight_edge(e)        highlight edge e
%   g.highlight_component(c)   highlight all nodes in component c
%   g.highlight_path(p)        highlight nodes and edge along path p
%
%   g.pick(coord)              vertex closest to coord
%
%   g.char()                   convert graph to string
%   g.display()                display summary of graph
%
% Matrix representations::
%   g.adjacency()          adjacency matrix
%   g.incidence()          incidence matrix
%   g.degree()             degree matrix
%   g.laplacian()          Laplacian  matrix
%
% Planning paths through the graph::
%   g.Astar(s, g)          shortest path from s to g
%   g.goal(v)              set goal vertex, and plan paths
%   g.path(v)              list of vertices from v to goal
%
% Graph and world points::
%   g.coord(v)             coordinate of vertex v
%   g.distance(v1, v2)     distance between v1 and v2
%   g.distances(coord)     return sorted distances from coord to all vertices
%   g.closest(coord)       vertex closest to coord
%
% Object properties (read only)::
%   g.n            number of vertices
%   g.ne           number of edges
%   g.nc           number of components
%
% Examples::
%         g = PGraph();
%         g.add_node([1 2]');  % add node 1
%         g.add_node([3 4]');  % add node 1
%         g.add_node([1 3]');  % add node 1
%         g.add_edge(1, 2);    % add edge 1-2
%         g.add_edge(2, 3);    % add edge 2-3
%         g.add_edge(1, 3);    % add edge 1-3
%         g.plot()
%
% Notes::
% - Graph connectivity is maintained by a labeling algorithm and this
%   is updated every time an edge is added.
% - Nodes and edges cannot be deleted.
% - Support for edge direction is rudimentary.

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

% Peter Corke 8/2009.

% TODO: 
%       be able to delete nodes, must update connectivity

classdef PGraph < handle

    properties (SetAccess=private, GetAccess=private)
        vertexlist      % vertex coordinates, columnwise, vertex number is the column number
        edgelist        % 2xNe matrix, each column is vertex index of edge start and end
        edgelen         % length (cost) of this edge    

        curLabel        % current label
        ncomponents     % number of components
        labels          % label of each vertex (1xN)
        labelset        % set of all labels (1xNc)

        goaldist        % distance from goal, after planning
        
        userdata        % per vertex data, cell array
        ndims           % number of coordinate dimensions, height of vertices matrix
        verbose
        measure         % distance measure: 'Euclidean', 'SE2'
    end

    properties (Dependent)
        n               % number of nodes/vertices 
        ne              % number of edges
        nc              % number of components
    end

    methods

        function g = PGraph(ndims, varargin)
        %PGraph.PGraph Graph class constructor
        %
        % G=PGraph(D, OPTIONS) is a graph object embedded in D dimensions.
        %
        % Options::
        %  'distance',M   Use the distance metric M for path planning which is either
        %                 'Euclidean' (default) or 'SE2'.
        %  'verbose'      Specify verbose operation
        %
        % Notes::
        % - Number of dimensions is not limited to 2 or 3.
        % - The distance metric 'SE2' is the sum of the squares of the difference 
        %   in position and angle modulo 2pi.
        % - To use a different distance metric create a subclass of PGraph and 
        %   override the method distance_metric().

            if nargin < 1
                ndims = 2;  % planar by default
            end
            g.ndims = ndims;
            opt.distance = 'Euclidean';
            opt = tb_optparse(opt, varargin);
            
            g.clear();
            g.verbose = opt.verbose;
            g.measure = opt.distance;
            g.userdata = {};
        end

        function n = get.n(g)
        %Pgraph.n Number of vertices
        %
        % G.n is the number of vertices in the graph.
        %
        % See also PGraph.ne.
            n = numcols(g.vertexlist);
        end

        function ne = get.ne(g)
        %Pgraph.ne Number of edges
        %
        % G.ne is the number of edges in the graph.
        %
        % See also PGraph.n.
            ne = numcols(g.edgelist);
        end

        function ne = get.nc(g)
        %Pgraph.nc Number of components
        %
        % G.nc is the number of components in the graph.
        %
        % See also PGraph.component.

            ne = g.ncomponents;
        end

        function clear(g)
        %PGraph.clear Clear the graph
        %
        % G.clear() removes all vertices, edges and components.

            g.labelset = zeros(1, 0);
            g.labels = zeros(1, 0);
            g.edgelist = zeros(2, 0);
            g.edgelen = zeros(1, 0);
            g.vertexlist = zeros(g.ndims, 0);

            g.ncomponents = 0;
            g.curLabel = 0;
        end

        function v = add_node(g, coord, varargin)
        %PGraph.add_node Add a node
        %
        % V = G.add_node(X) adds a node/vertex with coordinate X (Dx1) and
        % returns the integer node id V.
        %
        % V = G.add_node(X, V2) as above but connected by a directed edge from vertex V
        % to vertex V2 with cost equal to the distance between the vertices.
        %
        % V = G.add_node(X, V2, C) as above but the added edge has cost C.
        %
        % Notes::
        % - Distance is computed according to the metric specified in the
        %   constructor.
        %
        % See also PGraph.add_edge, PGraph.data, PGraph.getdata.

            if length(coord) ~= g.ndims
                error('coordinate length different to graph coordinate dimensions');
            end
            
            % append the coordinate as a column in the vertex matrix
            g.vertexlist = [g.vertexlist coord(:)];
            v = numcols(g.vertexlist);
            g.labels(v) = g.newlabel();

            if g.verbose
                fprintf('add node (%d) = ', v);
                fprintf('%f ', coord);
                fprintf('\n');
            end

            % optionally add an edge
            if nargin > 2
                g.add_edge(v, varargin{:});
            end
        end

        function u = setdata(g, v, u)
        %PGraph.setdata Set user data for node
        %
        % G.setdata(V, U) sets the user data of vertex V to U which can be of any 
        % type such as a number, struct, object or cell array.
        %
        % See also PGraph.data.

            g.userdata{v} = u;
        end

        function u = data(g, v)
        %PGraph.data Get user data for node
        %
        % U = G.data(V) gets the user data of vertex V which can be of any 
        % type such as a number, struct, object or cell array.
        %
        % See also PGraph.setdata.
            u = g.userdata{v};
        end

        
        function add_edge(g, v1, v2, d)
        %PGraph.add_edge Add an edge
        %
        % E = G.add_edge(V1, V2) adds a directed edge from vertex id V1 to vertex id V2, and
        % returns the edge id E.  The edge cost is the distance between the vertices.
        %
        % E = G.add_edge(V1, V2, C) as above but the edge cost is C.
        %
        % Notes::
        % - Distance is computed according to the metric specified in the
        %   constructor.
        % - Graph connectivity is maintained by a labeling algorithm and this
        %   is updated every time an edge is added.
        %
        % See also PGraph.add_node, PGraph.edgedir.
            if g.verbose
                fprintf('add edge %d -> %d\n', v1, v2);
            end
            for vv=v2(:)'
                g.edgelist = [g.edgelist [v1; vv]];
                if (nargin < 4) || isempty(d)
                    d = g.distance(v1, vv);
                end
                g.edgelen = [g.edgelen d];
                if g.labels(vv) ~= g.labels(v1)
                    g.merge(g.labels(vv), g.labels(v1));
                end
            end
        end

        function c = component(g, v)
        %PGraph.component Graph component
        %
        % C = G.component(V) is the id of the graph component that contains vertex
        % V.
            c = [];
            for vv=v
                tf = ismember(g.labelset, g.labels(vv));
                c = [c find(tf)];
            end
        end

        % which edges contain v
        %  elist = g.edges(v)
        function e = edges(g, v)
        %PGraph.edges Find edges given vertex
        %
        % E = G.edges(V) is a vector containing the id of all edges connected to vertex id V.
        %
        % See also PGraph.edgedir.
            e = [find(g.edgelist(1,:) == v) find(g.edgelist(2,:) == v)];
        end
        
         
        function dir = edgedir(g, v1, v2)
        %PGraph.edgedir Find edge direction
        %
        % D = G.edgedir(V1, V2) is the direction of the edge from vertex id V1
        % to vertex id V2.
        %
        % If we add an edge from vertex 3 to vertex 4 
        %         g.add_edge(3, 4)
        % then
        %         g.edgedir(3, 4)
        % is positive, and
        %         g.edgedir(4, 3)
        % is negative.
        %
        % See also PGraph.add_node, PGraph.add_edge.
            n = g.edges(v1);
            if any(ismember( g.edgelist(2, n), v2))
                dir = 1;
            elseif any(ismember( g.edgelist(1, n), v2))
                dir = -1;
            else
                dir = 0;
            end
        end

        function v = vertices(g, e)
        %PGraph.vertices Find vertices given edge
        %
        % V = G.vertices(E) return the id of the vertices that define edge E.
            v = g.edgelist(:,e);
        end


        function [n,c] = neighbours(g, v)
        %PGraph.neighbours Neighbours of a vertex
        %
        % N = G.neighbours(V) is a vector of ids for all vertices which are
        % directly connected neighbours of vertex V.
        %
        % [N,C] = G.neighbours(V) as above but also returns a vector C whose elements
        % are the edge costs of the paths corresponding to the vertex ids in N.
            e = g.edges(v);
            n = g.edgelist(:,e);
            n = n(:)';
            n(n==v) = [];   % remove references to self
            if nargout > 1
                c = g.cost(e);
            end
        end
        
        

        function [n,c] = neighbours_d(g, v)
        %PGraph.neighbours_d Directed neighbours of a vertex
        %
        % N = G.neighbours_d(V) is a vector of ids for all vertices which are
        % directly connected neighbours of vertex V.  Elements are positive
        % if there is a link from V to the node, and negative if the link
        % is from the node to V.
        %
        % [N,C] = G.neighbours_d(V) as above but also returns a vector C whose elements
        % are the edge costs of the paths corresponding to the vertex ids in N.
            e = g.edges(v);
            n = [-g.edgelist(1,e) g.edgelist(2,e)];
            n(abs(n)==v) = [];   % remove references to self
            if nargout > 1
                c = g.cost(e);
            end
        end

        function d = cost(g, e)
        %PGraph.cost Cost of edge
        %
        % C = G.cost(E) is the cost of edge id E.
            d = g.edgelen(e);
        end

        function d = setcost(g, e, c)
        %PGraph.cost Set cost of edge
        %
        % G.setcost(E, C) set cost of edge id E to C.
            g.edgelen(e) = c;
        end

        function p = coord(g, v)
        %PGraph.coord Coordinate of node
        %
        % X = G.coord(V) is the coordinate vector (Dx1) of vertex id V.

            p = g.vertexlist(:,v);
        end


        function c = connectivity(g)
        %PGraph.connectivity Graph connectivity
        %
        % C = G.connectivity() is a vector (Nx1) with the number of edges per
        % vertex.
        %
        % The average vertex connectivity is
        %         mean(g.connectivity())
        %
        % and the minimum vertex connectivity is
        %         min(g.connectivity())

            for k=1:g.n
                c(k) = length(g.edges(k));
            end
        end

        function plot(g, varargin)
        %PGraph.plot Plot the graph
        %
        % G.plot(OPT) plots the graph in the current figure.  Nodes
        % are shown as colored circles.
        %
        % Options::
        %  'labels'              Display vertex id (default false)
        %  'edges'               Display edges (default true)
        %  'edgelabels'          Display edge id (default false)
        %  'NodeSize',S          Size of vertex circle (default 8)
        %  'NodeFaceColor',C     Node circle color (default blue)
        %  'NodeEdgeColor',C     Node circle edge color (default blue)
        %  'NodeLabelSize',S     Node label text sizer (default 16)
        %  'NodeLabelColor',C    Node label text color (default blue)
        %  'EdgeColor',C         Edge color (default black)
        %  'EdgeLabelSize',S     Edge label text size (default black)
        %  'EdgeLabelColor',C    Edge label text color (default black)
        %  'componentcolor'      Node color is a function of graph component

            colorlist = 'bgmyc';

            % show vertices
            holdon = ishold;
            hold on

            % parse options
            opt.componentcolor = false;
            opt.labels = false;
            opt.edges = true;
            opt.edgelabels = false;
            opt.NodeSize = 8;
            opt.NodeFaceColor = 'b';
            opt.NodeEdgeColor = 'b';
            opt.NodeLabelSize = 16;
            opt.NodeLabelColor = 'b';
            opt.EdgeColor = 'k';
            opt.EdgeLabelSize = 8;
            opt.EdgeLabelColor = 'k';

            [opt,args] = tb_optparse(opt, varargin);
            
            % set default color if none specified
            if ~isempty(args)
                mcolor = args{1};
            else
                mcolor = 'b';
            end

            % show the vertices as filled circles
            for i=1:g.n
                % for each node
                if opt.componentcolor
                    j = mod( g.component(i)-1, length(colorlist) ) + 1;
                    c = colorlist(j);
                else
                    c = mcolor;
                end
                args = {'LineStyle', 'None', ...
                    'Marker', 'o', ...
                    'MarkerFaceColor', opt.NodeFaceColor, ...
                    'MarkerSize', opt.NodeSize, ...
                    'MarkerEdgeColor', opt.NodeEdgeColor };
                if g.ndims == 3
                    plot3(g.vertexlist(1,i), g.vertexlist(2,i), g.vertexlist(3,i), args{:});
                else
                    plot(g.vertexlist(1,i), g.vertexlist(2,i), args{:});
                end
            end
            % show edges
            if opt.edges
                for e=g.edgelist
                    v1 = g.vertexlist(:,e(1));
                    v2 = g.vertexlist(:,e(2));
                    if g.ndims == 3
                        plot3([v1(1) v2(1)], [v1(2) v2(2)], [v1(3) v2(3)], ...
                            'Color', opt.EdgeColor);
                    else
                        plot([v1(1) v2(1)], [v1(2) v2(2)], ...
                            'Color', opt.EdgeColor);
                    end
                end
            end
            % show the edge labels
            if opt.edgelabels
                for i=1:numcols(g.edgelist)
                    e = g.edgelist(:,i);
                    v1 = g.vertexlist(:,e(1));
                    v2 = g.vertexlist(:,e(2));
                    
                    text('String', sprintf('  %g', g.cost(i)), ...
                        'Position', (v1 + v2)/2, ...
                        'HorizontalAlignment', 'left', ...
                        'VerticalAlignment', 'middle', ...
                        'FontUnits', 'pixels', ...
                        'FontSize', opt.EdgeLabelSize, ...
                        'Color', opt.EdgeLabelColor);
                end
            end
            % show the labels
            if opt.labels
                for i=1:numcols(g.vertexlist)
                    text('String', sprintf('  %d', i), ...
                        'Position', g.vertexlist(:,i), ... 
                        'HorizontalAlignment', 'left', ... 
                        'VerticalAlignment', 'middle', ... 
                        'FontUnits', 'pixels', ... 
                        'FontSize', opt.NodeLabelSize, ... 
                        'Color', opt.NodeLabelColor);
                end
            end
            if ~holdon
                hold off
            end
        end

        function v = pick(g)
        %PGraph.pick Graphically select a vertex
        %
        % V = G.pick() is the id of the vertex closest to the point clicked
        % by the user on a plot of the graph.
        %
        % See also PGraph.plot.
            [x,y] = ginput(1);
            v = g.closest([x; y]);
        end

        function goal(g, vg)
        %PGraph.goal Set goal node
        %
        % G.goal(VG) computes the cost of reaching every vertex in the graph connected 
        % to the goal vertex VG.
        %
        % Notes::
        % - Combined with G.path performs a breadth-first search for paths to the goal.
        %
        % See also PGraph.path, PGraph.Astar, Astar.

            % cost is total distance from goal
            g.goaldist = Inf*ones(1, numcols(g.vertexlist));

            g.goaldist(vg) = 0;
            g.descend(vg);
        end


        function p = path(g, v)
        %PGraph.path Find path to goal node
        %
        % P = G.path(VS) is a vector of vertex ids that form a path from
        % the starting vertex VS to the previously specified goal.  The path
        % includes the start and goal vertex id.
        %
        % To compute path to goal vertex 5
        %        g.goal(5);
        % then the path, starting from vertex 1 is
        %        p1 = g.path(1);
        % and the path starting from vertex 2 is
        %        p2 = g.path(2);
        %
        % Notes::
        % - Pgraph.goal must have been invoked first.
        % - Can be used repeatedly to find paths from different starting points
        %   to the goal specified to Pgraph.goal().
        %
        % See also PGraph.goal, PGraph.Astar.
            p = [v];

            while g.goaldist(v) ~= 0
                v = g.next(v);
                p = [p v];
            end
        end

        
        function d = distance(g, v1, v2)
        %PGraph.distance Distance between vertices
        %
        % D = G.distance(V1, V2) is the geometric distance between
        % the vertices V1 and V2.
        %
        % See also PGraph.distances.
            
            d = g.distance_metric( g.vertexlist(:,v1), g.vertexlist(:,v2));
            
        end

        function [d,k] = distances(g, p)
        %PGraph.distances Distances from point to vertices
        %
        % D = G.distances(X) is a vector (1xN) of geometric distance from the point
        % X (Dx1) to every other vertex sorted into increasing order.
        %
        % [D,W] = G.distances(P) as above but also returns W (1xN) with the 
        % corresponding vertex id.
        %
        % Notes::
        % - Distance is computed according to the metric specified in the
        %   constructor.
        %
        % See also PGraph.closest.
            
            d = g.distance_metric(p(:), g.vertexlist);
            [d,k] = sort(d, 'ascend');
        end

        function [c,dn] = closest(g, p)
        %PGraph.closest Find closest vertex
        %
        % V = G.closest(X) is the vertex geometrically closest to coordinate X.
        %
        % [V,D] = G.closest(X) as above but also returns the distance D.
        %
        % See also PGraph.distances.
            d = g.distance_metric(p(:), g.vertexlist);
            [mn,c] = min(d);

            if nargin > 1
                dn = mn;
            end
        end

        function display(g)
        %PGraph.display Display graph
        %
        % G.display() displays a compact human readable representation of the
        % state of the graph including the number of vertices, edges and components.
        %
        % See also PGraph.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(g) );
        end % display()

        function s = char(g)
        %PGraph.char Convert graph to string
        %
        % S = G.char() is a compact human readable representation of the
        % state of the graph including the number of vertices, edges and components.

            s = '';
            s = strvcat(s, sprintf('  %d dimensions', g.ndims));
            s = strvcat(s, sprintf('  %d vertices', g.n));
            s = strvcat(s, sprintf('  %d edges', numcols(g.edgelist)));
            s = strvcat(s, sprintf('  %d components', g.ncomponents));
        end

        %% convert graphs to matrix representations

        function L = laplacian(g)
        %Pgraph.laplacian Laplacian matrix of graph
        %
        % L = G.laplacian() is the Laplacian matrix (NxN) of the graph.
        %
        % Notes::
        % - L is always positive-semidefinite.
        % - L has at least one zero eigenvalue.
        % - The number of zero eigenvalues is the number of connected components 
        %   in the graph.
        %
        % See also PGraph.adjacency, PGraph.incidence, PGraph.degree.

            L = g.degree() - (g.adjacency() > 0);
        end

        function D = degree(g)
        %Pgraph.degree Degree matrix of graph
        %
        % D = G.degree() is a diagonal matrix (NxN) where element D(i,i) is the number
        % of edges connected to vertex id i.
        %
        % See also PGraph.adjacency, PGraph.incidence, PGraph.laplacian.

            D = diag( g.connectivity() );
        end

        function A = adjacency(g)
        %Pgraph.adjacency Adjacency matrix of graph
        %
        % A = G.adjacency() is a matrix (NxN) where element A(i,j) is the cost
        % of moving from vertex i to vertex j.
        %
        % Notes::
        % - Matrix is symmetric.
        % - Eigenvalues of A are real and are known as the spectrum of the graph.
        % - The element A(I,J) can be considered the number of walks of one
        %   edge from vertex I to vertex J (either zero or one).  The element (I,J)
        %   of A^N are the number of walks of length N from vertex I to vertex J.
        %
        % See also PGraph.degree, PGraph.incidence, PGraph.laplacian.

            A = zeros(g.n, g.n);
            for i=1:g.n
                [n,c] = g.neighbours(i);
                for j=1:numel(n)
                    A(i,n(j)) = c(j);
                    A(n(j),i) = c(j);
                end
            end
        end

        function I = incidence(g)
        %Pgraph.degree Incidence matrix of graph
        %
        % IN = G.incidence() is a matrix (NxNE) where element IN(i,j) is
        % non-zero if vertex id i is connected to edge id j.
        %
        % See also PGraph.adjacency, PGraph.degree, PGraph.laplacian.
            I = zeros(g.n, numcols(g.edgelist));
            for i=1:g.n
                for n=g.edges(i)
                    I(i,n) = 1;
                end
            end
        end

        %% these are problematic, dont advertise them
        %
        % removing an edge may divide the graph into 2 components, this is expensive
        % to check and currently not implemented
        function delete_edge(g, e)
            g.edgelist(:,e) = [];
            % really need to check if the two halves are connected, is expensive
            % could use path planner
        end

        function delete_node(g, v)
            el = g.edges(v);
            el
            % make the column invalid, really should remove it but this
            % requires changing all the edgelist entries, and the vertex
            % numbers will change...
            g.vertexlist(:,v) = [NaN; NaN];
            g.delete_edge(el);
            g.n = g.n - 1;
        end

        
        function highlight_node(g, verts, varargin)
        %PGraph.highlight_node Highlight a node
        %
        % G.highlight_node(V, OPTIONS) highlights the vertex V with a yellow marker.
        % If V is a list of vertices then all are highlighted.
        %
        % Options::
        %  'NodeSize',S          Size of vertex circle (default 12)
        %  'NodeFaceColor',C     Node circle color (default yellow)
        %  'NodeEdgeColor',C     Node circle edge color (default blue)
        %
        % See also PGraph.highlight_edge, PGraph.highlight_path, PGraph.highlight_component.
            
            hold on

            % parse options
            opt.NodeSize = 12;
            opt.NodeFaceColor = 'y';
            opt.NodeEdgeColor = 'b';

            [opt,args] = tb_optparse(opt, varargin);
            markerprops = {'LineStyle', 'None', ...
                'Marker', 'o', ...
                'MarkerFaceColor', opt.NodeFaceColor, ...
                'MarkerSize', opt.NodeSize, ...
                'MarkerEdgeColor', opt.NodeEdgeColor };

            for v=verts
                if g.ndims == 3
                    plot3(g.vertexlist(1,v), g.vertexlist(2,v), g.vertexlist(3,v), ...
                        markerprops{:});
                else
                    plot(g.vertexlist(1,v), g.vertexlist(2,v), markerprops{:});
                end
            end
        end

        function highlight_component(g, c, varargin)
        %PGraph.highlight_component Highlight a graph component
        %
        % G.highlight_component(C, OPTIONS) highlights the vertices that belong to 
        % graph component C.
        %
        % Options::
        %  'NodeSize',S          Size of vertex circle (default 12)
        %  'NodeFaceColor',C     Node circle color (default yellow)
        %  'NodeEdgeColor',C     Node circle edge color (default blue)
        %
        % See also PGraph.highlight_node, PGraph.highlight_edge, PGraph.highlight_component.
            nodes = find(g.labels == g.labelset(c));
            for v=nodes
                g.highlight_node(v, varargin{:});
            end
        end

        function highlight_edge(g, e, varargin)
        %PGraph.highlight_node Highlight a node
        %
        % G.highlight_edge(V1, V2) highlights the edge between vertices V1 and V2.
        %
        % G.highlight_edge(E) highlights the edge with id E.
        %
        % Options::
        % 'EdgeColor',C         Edge edge color (default black)
        % 'EdgeThickness',T     Edge thickness (default 1.5)
        %
        % See also PGraph.highlight_node, PGraph.highlight_path, PGraph.highlight_component.
            
            % parse options
            opt.EdgeColor = 'k';
            opt.EdgeThickness = 1.5;

            [opt,args] = tb_optparse(opt, varargin);

            hold on
            if (length(args) > 0) && isnumeric(args{1})
                % highlight_edge(V1, V2)
                v1 = e;
                v2 = args{1};

                v1 = g.vertexlist(:,v1);
                v2 = g.vertexlist(:,v2);
             else
                % highlight_edge(E)
                e = g.edgelist(:,e);
                v1 = g.vertexlist(:,e(1));
                v2 = g.vertexlist(:,e(2));
            end

            % create the line properties for the edges
            lineprops = {
                'Color', opt.EdgeColor, ...
                'LineWidth', opt.EdgeThickness };

            if g.ndims == 3
                plot3([v1(1) v2(1)], [v1(2) v2(2)], [v1(3) v2(3)], lineprops{:});
            else
                plot([v1(1) v2(1)], [v1(2) v2(2)], lineprops{:});
            end
        end

        function highlight_path(g, path)
        %PGraph.highlight_path Highlight path
        %
        % G.highlight_path(P, OPTIONS) highlights the path defined by vector P
        % which is a list of vertex ids comprising the path.
        %
        % Options::
        %  'NodeSize',S          Size of vertex circle (default 12)
        %  'NodeFaceColor',C     Node circle color (default yellow)
        %  'NodeEdgeColor',C     Node circle edge color (default blue)
        %  'EdgeColor',C         Node circle edge color (default black)
        %
        % See also PGraph.highlight_node, PGraph.highlight_edge, PGraph.highlight_component.
            g.highlight_node(path);

            % highlight the edges
            for i=1:numel(path)-1
                v1 = path(i);
                v2 = path(i+1);
                g.highlight_edge(v1, v2);
            end
        end

        function [path,cost] = Astar(g, vstart, vgoal)
            %PGraph.Astar path finding
            %
            % PATH = G.Astar(V1, V2) is the lowest cost path from vertex V1 to
            % vertex V2.  PATH is a list of vertices starting with V1 and ending
            % V2.
            %
            % [PATH,C] = G.Astar(V1, V2) as above but also returns the total cost
            % of traversing PATH.
            %
            % Notes::
            % - Uses the efficient A* search algorithm.
            %
            % References::
            % - Correction to "A Formal Basis for the Heuristic Determination of Minimum Cost Paths".
            %   Hart, P. E.; Nilsson, N. J.; Raphael, B.
            %   SIGART Newsletter 37: 28-29, 1972.
            %
            % See also PGraph.goal, PGraph.path.

             % The set of vertices already evaluated.
             closedset = [];  

             % The set of tentative vertices to be evaluated, initially containing the start node
             openset = [vstart];    
             came_from = [];    % The map of navigated vertices.
         
             g_score(vstart) = 0;    % Cost from start along best known path.
             h_score(vstart) = g.distance(vstart, vgoal);
             % Estimated total cost from start to goal through y.
             f_score(vstart) = g_score(vstart) + h_score(vstart);
         
             while ~isempty(openset)
                 % current := the node in openset having the lowest f_score[] value
                 [mn,k] = min(f_score(openset));
                 vcurrent = openset(k);

                 if vcurrent == vgoal
                     path = [];
                     p = vgoal;
                     while true
                        path = [p path];
                        p = came_from(p);
                        if p == 0
                            break;
                        end
                     end
                     if nargout > 1
                        cost = f_score(vgoal);
                     end
                     return
                 end

                 %remove current from openset
                 openset = setdiff(openset, vcurrent);
                 %add current to closedset
                 closedset = union(closedset, vcurrent);

                 for neighbour = g.neighbours(vcurrent)
                     if ismember(neighbour, closedset)
                         continue;
                     end
                     tentative_g_score = g_score(vcurrent) + ...
                        g.distance(vcurrent,neighbour);
         
                     if ~ismember(neighbour, openset)
                         %add neighbor to openset
                         openset = union(openset, neighbour);
                         h_score(neighbour) = g.distance(neighbour, vgoal);
                         tentative_is_better = true;
                     elseif tentative_g_score < g_score(neighbour)
                         tentative_is_better = true;
                     else
                         tentative_is_better = false;
                     end
                     if tentative_is_better
                         came_from(neighbour) = vcurrent;
                         g_score(neighbour) = tentative_g_score;
                         f_score(neighbour) = g_score(neighbour) + h_score(neighbour);
                     end
                 end
             end
             path = [];
        end

        
    end % method

    methods (Access='protected')
    % private methods

        % depth first
        function descend(g, vg)

            % get neighbours and their distance
            for nc = g.neighbours2(vg);
                vn = nc(1);
                d = nc(2);
                newcost = g.goaldist(vg) + d;
                if isinf(g.goaldist(vn))
                    % no cost yet assigned, give it this one
                    g.goaldist(vn) = newcost;
                    %fprintf('1: cost %d <- %f\n', vn, newcost);
                    descend(g, vn);
                else
                    % it already has a cost
                    if g.goaldist(vn) <= newcost
                        continue;
                    else
                        g.goaldist(vn) = newcost;
                        %fprintf('2: cost %d <- %f\n', vn, newcost);
                        descend(g, vn);
                    end
                end
            end
        end

        % breadth first
        function descend2(g, vg)

            % get neighbours and their distance
            for vn = g.neighbours2(vg);
                vn = nc(1);
                d = nc(2);
                newcost = g.goaldist(vg) + d;
                if isinf(g.goaldist(vn))
                    % no cost yet assigned, give it this one
                    g.goaldist(vn) = newcost;
                    fprintf('1: cost %d <- %f\n', vn, newcost);
                    descend(g, vn);
                elseif g.goaldist(vn) > newcost
                    % it already has a cost
                        g.goaldist(vn) = newcost;
                end
            end
            for vn = g.neighbours(vg);
                descend(g, vn);
            end
        end

        function l = newlabel(g)
            g.curLabel = g.curLabel + 1;
            l = g.curLabel;
            g.ncomponents = g.ncomponents + 1;
            g.labelset = union(g.labelset, l);
        end

        function merge(g, l1, l2)
            
            % merge label1 and label2, lowest label dominates

            % get the dominant and submissive labels
            ldom = min(l1, l2);
            lsub = max(l1, l2);

            % change all instances of submissive label to dominant one
            g.labels(g.labels==lsub) = ldom;

            % reduce the number of components
            g.ncomponents = g.ncomponents - 1;
            % and remove the submissive label from the set of all labels
            g.labelset = setdiff(g.labelset, lsub);
        end

        function nc = neighbours2(g, v)
            e = g.edges(v);
            n = g.edgelist(:,e);
            n = n(:)';
            n(n==v) = [];   % remove references to self
            c = g.cost(e);
            nc = [n; c];
        end
        
        function d = distance_metric(g, x1, x2)

            % distance between coordinates x1 and x2 using the relevant metric
            % x2 can be multiple points represented by multiple columns
            switch g.measure
                case 'Euclidean'
                   d = colnorm( bsxfun(@minus, x1, x2) );

                case 'SE2'
                    d = bsxfun(@minus, x1, x2);
                    d(3,:) = angdiff(d(3,:));
                    d = colnorm( d );
                otherwise
                    error('unknown distance measure', g.measure);
            end
        end

        function vn = next(g, v)

            % V = G.next(VS) return the id of a node connected to node id VS
            % that is closer to the goal.
            n = g.neighbours(v);
            [mn,k] = min( g.goaldist(n) );
            vn = n(k);
        end
        
    end % private methods
end % classdef
