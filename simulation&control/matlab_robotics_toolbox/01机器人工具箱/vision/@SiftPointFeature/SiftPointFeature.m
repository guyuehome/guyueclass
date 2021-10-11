%SiftCornerFeature SIFT point corner feature object
%
% A subclass of OrientedScalePointFeature for SIFT features.
%
% Methods::
% plot         Plot feature position
% plot_scale   Plot feature scale
% distance     Descriptor distance
% ncc          Descriptor similarity
% match        Match features
% ncc          Descriptor similarity
% uv           Return feature coordinate
% display      Display value
% char         Convert value to string
%
% Properties::
% u             horizontal coordinate
% v             vertical coordinate
% strength      feature strength
% theta         feature orientation [rad]
% scale         feature scale
% descriptor    feature descriptor (vector)
% image_id      index of image containing feature
%
% Properties of a vector of SiftCornerFeature objects are returned as a vector.
% If F is a vector (Nx1) of SiftCornerFeature objects then F.u is a 2xN matrix
% with each column the corresponding u coordinate.
%
% Notes::
% - SiftCornerFeature is a reference object.
% - SiftCornerFeature objects can be used in vectors and arrays
% - The SIFT algorithm is patented and not distributed with this toolbox.
%   You can download a SIFT implementation which this class can utilize.
%   See README.SIFT.
%
% References::
%
% "Distinctive image features from scale-invariant keypoints",
% D.Lowe, 
% Int. Journal on Computer Vision, vol.60, pp.91-110, Nov. 2004.
%
% See also ISIFT, PointFeature, ScalePointFeature, OrientedScalePointFeature, SurfPointFeature.

classdef SiftPointFeature < OrientedScalePointFeature

    properties
        image_id_
    end % properties

    methods
        function f = SiftPointFeature(varargin)
        %SiftPointFeature.SiftPointFeature Create a SIFT point feature object
        %   
        % F = SiftPointFeature() is a point feature object with null parameters.
        %   
        % F = SiftPointFeature(U, V) is a point feature object with specified
        % coordinates.
        %   
        % F = SiftPointFeature(U, V, STRENGTH) as above but with specified strength.
        %
        % F = SiftPointFeature(U, V, STRENGTH, SCALE) as above but with specified 
        % feature scale.
        %
        % F = SiftPointFeature(U, V, STRENGTH, SCALE, THETA) as above but with specified 
        % feature orientation.
        %
        % See also isift.

            f = f@OrientedScalePointFeature(varargin{:});  % invoke the superclass constructor
        end

        function val = image_id(features)
            val = [features.image_id_];
        end

        function [m,corresp] = match(f1, f2)
        %SiftPointFeature.match Match SIFT point features
        %   
        % M = F.match(F2, OPTIONS) is a vector of FeatureMatch objects that 
        % describe candidate matches between the two vectors of SIFT 
        % features F and F2.  Correspondence is based on descriptor
        % similarity.

        %
        % [M,C] = F.match(F2, OPTIONS) as above but returns a correspodence
        % matrix where each row contains the indices of corresponding features
        % in F and F2  respectively.
        %
        % See also FeatureMatch.

        % TODO
        % Options::
        % 'thresh',T    Match threshold (default 0.05)
        % 'median'      Threshold at the median distance
        % ambiguity threshold, defaults to 1.5 in siftmatch
        % use distance
        %

            [matches,dist] = siftmatch([f1.descriptor], [f2.descriptor]);

            % matches is a 2xM matrix, one column per match, each column is the index of the
            % matching features in image 1 and 2 respectively
            % dist is a 1xM matrix of distance between the matched features, low is good.

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


    end % methods

    methods(Static)

        % the MEX functions live in a private subdirectory, so these static methods
        % provide convenient access to them

        function [k,d] = sift(varargin)
            try
            [k,d] = sift(varargin{:});
            catch me
                if strcmp(me.identifier, 'MATLAB:UndefinedFunction')
                    error('MVTB:SiftPointFeature:notinstalled', 'Contributed software for SIFT features is not installed')
                end
            end
        end
    end

end % classdef
