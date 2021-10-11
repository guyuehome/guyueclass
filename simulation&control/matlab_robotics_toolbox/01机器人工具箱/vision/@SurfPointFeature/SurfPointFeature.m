%SurfPointFeature  SURF point corner feature object
%
% A subclass of OrientedScalePointFeature for SURF features.
%
% Methods::
% plot         Plot feature position
% plot_scale   Plot feature scale
% distance     Descriptor distance
% ncc          Descriptor similarity
% match        Match features
% uv           Return feature coordinate
% display      Display value
% char         Convert value to string
%
% Properties::
% u             horizontal coordinate
% v             vertical coordinate
% strength      feature strength
% scale         feature scale
% theta         feature orientation [rad]
% descriptor    feature descriptor (vector)
% image_id      index of image containing feature
%
% Properties of a vector of SurfCornerFeature objects are returned as a vector.
% If F is a vector (Nx1) of SurfCornerFeature objects then F.u is a 2xN matrix
% with each column the corresponding u coordinate.
%
% Notes::
% - SurfCornerFeature is a reference object.
% - SurfCornerFeature objects can be used in vectors and arrays
%
% Reference::
% "SURF: Speeded Up Robust Features", 
% Herbert Bay, Andreas Ess, Tinne Tuytelaars, Luc Van Gool,
% Computer Vision and Image Understanding (CVIU), 
% Vol. 110, No. 3, pp. 346--359, 2008
%
% See also ISURF, PointFeature, ScalePointFeature, OrientedScalePointFeature, SiftPointFeature.

classdef SurfPointFeature < OrientedScalePointFeature

    properties
        image_id_
    end % properties

    methods
        function f = SurfPointFeature(varargin)
        %SurfPointFeature.SurfPointFeature Create a SURF point feature object
        %   
        % F = SurfPointFeature() is a point feature object with null parameters.
        %   
        % F = SurfPointFeature(U, V) is a point feature object with specified
        % coordinates.
        %   
        % F = SurfPointFeature(U, V, STRENGTH) as above but with specified strength.
        %
        % F = SurfScalePointFeature(U, V, STRENGTH, SCALE) as above but with specified 
        % feature scale.
        %
        % F = SurfPointFeature(U, V, STRENGTH, SCALE, THETA) as above but with specified 
        % feature orientation.
        %
        % See also isurf, OrientedScalePointFeature.

            f = f@OrientedScalePointFeature(varargin{:});  % invoke the superclass constructor
        end

        function val = image_id(features)
            val = [features.image_id_];
        end

        function [m,corresp] = match(f1, f2, varargin)
        %SurfPointFeature.match Match SURF point features
        %   
        % M = F.match(F2, OPTIONS) is a vector of FeatureMatch objects that 
        % describe candidate matches between the two vectors of SURF 
        % features F and F2.  Correspondence is based on descriptor
        % similarity.
        %
        % [M,C] = F.match(F2, OPTIONS) as above but returns a correspodence
        % matrix where each row contains the indices of corresponding features
        % in F and F2  respectively.
        %
        % Options::
        % 'thresh',T    Match threshold (default 0.05)
        % 'median'      Threshold at the median distance
        %
        % Notes::
        % - for no threshold set to [].
        %
        % See also FeatureMatch.
        
            if isempty(f2)
                m = [];
                corresp = [];
                return;
            end

            opt.thresh = 0.05;
            opt.median = false;
            opt = tb_optparse(opt, varargin);

            % Put the landmark descriptors in a matrix
            D1 = f1.descriptor;
            D2 = f2.descriptor;

            % Find the best matches
            err=zeros(1,length(f1));
            cor1=1:length(f1); 
            cor2=zeros(1,length(f1));
            for i=1:length(f1),
                distance = sum((D2-repmat(D1(:,i),[1 length(f2)])).^2,1);
                [err(i),cor2(i)] = min(distance);
            end

            % Sort matches on vector distance
            [err, ind] = sort(err); 
            cor1=cor1(ind); 
            cor2=cor2(ind);

            % Build a list of FeatureMatch objects
            m = [];
            cor = [];
            for i=1:length(f1)
                k1 = cor1(i);
                k2 = cor2(i);
                mm = FeatureMatch(f1(k1), f2(k2), err(i));
                m = [m mm];
                cor(:,i) = [k1 k2]';
            end            

            % get the threshold, either given or the median of all errors
            if opt.median
                thresh = median(err);
            else
                thresh = opt.thresh;
            end

            % remove those matches over threshold
            if ~isempty(thresh)
                k = err > thresh;
                cor(:,k) = [];
                m(k) = [];
            end

            if nargout > 1
                corresp = cor;
            end
        end


    end % methods

    methods(Static)

        % the MEX functions live in a private subdirectory, so these static methods
        % provide convenient access to them

        function Ipts = surf(im, opt)
            if exist('surfpoints') == 3
                fprintf('MEX\n');
                % do the OpenCV/MEX version
                % put the results into the same return format as OpenSurf
                params.extended = 0;
                params.nOctaves = opt.octaves;
                if ~isempty(opt.thresh)
                    params.hessianThreshold = opt.thresh;
                end

                try
                    [p,d,l,info] = surfpoints(iint(im), params);
                catch me
                    if strcmp(me.identifier, 'MATLAB:UndefinedFunction')
                        error('MVTB:SurfPointFeature:notinstalled', 'Contributed software for SURF features is not installed')
                    end
                end

                % returns
                % p    point coordinates, one per column
                % d    SURF descriptor, one per column
                % l    sign of the Laplacian (light or dark feature)
                % info other parameters, per row: scale, strength,
                %      orientation
                
                % put the data into a vector of structs format to 
                % match OpenSurf
                Ipts = struct('x', num2cell(p(1,:)), ...
                              'y', num2cell(p(2,:)), ...
                              'scale', num2cell(info(1,:)), ...
                              'strength', num2cell(info(2,:)), ...
                              'orientation', num2cell(info(3,:)), ...
                              'descriptor', num2cell(d,1)   );
                              
            else
                params.octaves = opt.octaves;   % for OpenSurf
                if ~isempty(opt.thresh)
                    params.tresh = opt.thresh;      % for OpenSurf, (sic)
                end

                Ipts = OpenSurf(im, params);
            end
        end
    end

end % classdef
