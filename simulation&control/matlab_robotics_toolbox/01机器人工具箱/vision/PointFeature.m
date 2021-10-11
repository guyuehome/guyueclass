%PointFeature  PointCorner feature object
%
% A superclass for image corner features.
%
% Methods::
% plot         Plot feature position
% distance     Descriptor distance
% ncc          Descriptor similarity
% uv           Return feature coordinate
% display      Display value
% char         Convert value to string
%
% Properties::
% u             horizontal coordinate
% v             vertical coordinate
% strength      feature strength
% descriptor    feature descriptor (vector)
%
% Properties of a vector of PointFeature objects are returned as a vector.
% If F is a vector (Nx1) of PointFeature objects then F.u is a 2xN matrix
% with each column the corresponding point coordinate.
%
% See also ScalePointFeature, SurfPointFeature, SiftPointFeature.

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

classdef PointFeature < handle

    properties %(GetAccess=protected)%, Hidden=true)
        u_           % feature x-coordinates
        v_           % feature y-coordinates
        strength_
        descriptor_
    end % properties

    methods

        function f = PointFeature(u, v, strength)
        %PointFeature.PointFeature Create a point feature object
        %
        % F = PointFeature() is a point feature object with null parameters.
        %
        % F = PointFeature(U, V) is a point feature object with specified
        % coordinates.
        %
        % F = PointFeature(U, V, STRENGTH) as above but with specified strength.

            if nargin == 0
                return;
            end
            if nargin >= 2
                f.u_ = u;
                f.v_ = v;
            end
            if nargin == 3
                f.strength_ = strength;
            end
        end

        function val = uv(features)
            val = [[features.u]; [features.v]];
        end

        function plot(features, varargin)
        %PointFeature.plot Plot feature
        %
        % F.plot() overlay a white square marker at the feature position.
        %
        % F.plot(LS) as above but the optional line style arguments LS are
        % passed to plot.
        %
        % If F is a vector then each element is plotted.

            holdon = ishold;
            hold on

            if nargin == 1
                varargin = {'ws'};
            end

            for i=1:length(features)
                plot(features(i).u_, features(i).v_, varargin{:});
            end

            if ~holdon
                hold off
            end
        end % plot

        function s = distance(f1, f2)
        %PointFeature.distance Distance between feature descriptors
        %
        % D = F.distance(F1) is the distance between feature descriptors, the norm
        % of the Euclidean distance.
        %
        % If F is a vector then D is a vector whose elements are the distance between
        % the corresponding element of F and F1.
            for i=1:length(f2)
                s(i) = norm(f1.descriptor-f2(i).descriptor);
            end
        end

        function s = ncc(f1, f2)
        %PointFeature.ncc Feature descriptor similarity
        %
        % S = F.ncc(F1) is the similarty between feature descriptors which is a
        % scalar in the interval -1 to 1, where 1 is perfect match.
        %
        % If F is a vector then D is a vector whose elements are the distance between
        % the corresponding element of F and F1.
            for i=1:length(f2)
                s(i) = dot(f1.descriptor,f2(i).descriptor);
            end
        end

        function [m,corresp] = match(f1, f2)
        %PointFeature.match Match point features
        %   
        % M = F.match(F2, OPTIONS) is a vector of FeatureMatch objects that 
        % describe candidate matches between the two vectors of point
        % features F and F2.
        %
        % [M,C] = F.match(F2, OPTIONS) as above but returns a correspodence
        % matrix where each row contains the indices of corresponding features
        % in F and F2  respectively.
        %
        % Options::
        % 'thresh',T    Match threshold (default 0.05)
        % 'median'      Threshold at the median distance
        %
        % See also FeatureMatch.


            [matches,dist,dist2] = closest([f1.descriptor], [f2.descriptor]);
            matches = [1:length(f1); matches];
   
            % delete matches where distance of closest match is greater than 
            % 0.7 of second closest match
            k = dist > 0.7 * dist2;
   
            matches(:,k) = [];
            dist(k) = [];
   
            % dist is a 1xM matrix of distance between the matched features, low is good.
   
            % matches is a 2xM matrix, one column per match, each column 
            % is the index of the matching feature in images 1 and 2
   
            % sort into increasing distance
            [z,k] = sort(dist, 'ascend');
            matches = matches(:,k);
            dist = dist(:,k);

            m = [];
            cor = [];

            for i=1:numcols(matches),
                k1 = matches(1,i);
                k2 = matches(2,i);
                mm = FeatureMatch(f1(k1), f2(k2), dist(i));
                m = [m mm];
                cor(:,i) = [k1 k2]';
            end            

            if nargout > 1
                corresp = cor;
            end
        end

        % methods to provide convenient access to properties of object vectors
        function val = u(f)
            val = [f.u_];
        end

        function val = v(f)
            val = [f.v_];
        end

        function val = p(f)
            val = [[f.u_]; [f.v_]];
        end

        function val = strength(f)
            val = [f.strength_];
        end

        function val = descriptor(f)
            val = [f.descriptor_];
        end

        function display(f)
        %PointFeature.display Display value
        %
        % F.display() displays a compact human-readable representation of the feature.
        % If F is a vector then the elements are printed one per line.
        %
        % Notes::
        % - This method is invoked implicitly at the command line when the result
        %   of an expression is a PointFeature object and the command has no trailing
        %   semicolon.
        %
        % See also PointFeature.char.

            disp(' ');
            disp([inputname(1), ' = '])
            disp(' ');
            if length(f) > 20
                fprintf('%d features (listing suppressed)\n  Properties:', length(f));
                for property=fieldnames(f)'
                    fprintf(' %s', property{1}(1:end-1));
                end
                fprintf('\n');
            else
                disp( char(f) );
            end
        end % display()

        function ss = char(features)
        %PointFeature.char Convert to string
        %
        % S = F.char() is a compact string representation of the point feature.
        % If F is a vector then the string has multiple lines, one per element.

            ss = [];
            for i=1:length(features)
                f = features(i);
                % display the coordinate
                s = sprintf('  (%g,%g)', f.u_, f.v_);

                % display the other properties
                for property=fieldnames(f)'
                    prop = property{1}; % convert from cell array
                    switch prop
                    case {'u_', 'v_', 'descriptor_'}
                        continue;
                    otherwise
                        val = getfield(f, prop);
                        if ~isempty(val)
                            s = strcat(s, [sprintf(', %s=', prop(1:end-1)), num2str(val, ' %g')]);
                        end
                    end
                end

                % do the descriptor last
                val = getfield(f, 'descriptor_');
                if ~isempty(val)
                    if length(val) == 1
                        % only list scalars or shortish vectors
                        s = strcat(s, [', descrip=', num2str(val, ' %g')]);
                    elseif length(val) < 4
                        % only list scalars or shortish vectors
                        s = strcat(s, [', descrip=(', num2str(val', ' %g'), ')']);
                    else
                        s = strcat(s, ', descrip= ..');
                    end
                end
                ss = strvcat(ss, s);
            end
        end


    end % methods
end % classdef

