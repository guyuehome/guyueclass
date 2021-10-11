%ScalePointFeature  ScalePointCorner feature object
%
% A subclass of PointFeature for features with scale.
%
% Methods::
% plot         Plot feature position
% plot_scale   Plot feature scale
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
% scale         feature scale
% descriptor    feature descriptor (vector)
%
% Properties of a vector of ScalePointFeature objects are returned as a vector.
% If F is a vector (Nx1) of ScalePointFeature objects then F.u is a 2xN matrix
% with each column the corresponding point coordinate.
%
% See also PointFeature, OrientedScalePointFeature, SurfPointFeature, SiftPointFeature.
classdef OrientedScalePointFeature < ScalePointFeature
    
    properties
        theta_
    end % properties
    
    methods
        
        function f = OrientedScalePointFeature(varargin)
        %OrientedScalePointFeature.ScalePointFeature Create a scale point feature object
        %
        % F = OrientedScalePointFeature() is a point feature object with null parameters.
        %
        % F = OrientedScalePointFeature(U, V) is a point feature object with specified
        % coordinates.
        %
        % F = OrientedScalePointFeature(U, V, STRENGTH) as above but with specified strength.
        %
        % F = OrientedScalePointFeature(U, V, STRENGTH, SCALE) as above but with specified 
        % feature scale.
        %
        % F = OrientedScalePointFeature(U, V, STRENGTH, SCALE, THETA) as above but with specified 
        % feature orientation.
            f = f@ScalePointFeature(varargin{:});  % invoke the superclass constructor
        
            if nargin > 4
                f.theta_ = varargin{5};
            end
        end
        
        
        function val = theta(features)
            val = [features.theta_];
        end
        
        function plot(features, varargin)
        %OrientedScalePointFeature.plot Plot feature
        %   
        % F.plot(OPTIONS) overlay a marker to indicate feature point position and
        % scale.
        %
        % F.plot(OPTIONS, LS) as above but the optional line style arguments LS are
        % passed to plot.
        %   
        % If F is a vector then each element is plotted.
        %   
        % Options::
        % 'circle'    Indicate scale by a circle (default)
        % 'clock'     Indicate scale by circle with one radial line for orientation
        % 'arrow'     Indicate scale and orientation by an arrow
        % 'disk'      Indicate scale by a translucent disk
        % 'color',C   Color of circle or disk (default green)
        % 'alpha',A   Transparency of disk, 1=opaque, 0=transparent (default 0.2)
        % 'scale',S   Scale factor for drawing circles and arrows.
        %
        % Examples::
        %
        % Mark the feature coordinates with a white asterisk
        %          f.plot('w*')
        % Mark each feature with a blue translucent disk
        %          f.plot('disk', 'color', 'b', 'alpha', 0.3);
        % Mark each feature with a green circle with a radial line to
        % indicate orientation and with exagerated scale
        %          f.plot('clock', 'color', 'g', 'scale', 2)
        %
        % See also ScalePointFeature.plot, PointFeature.plot, plot.

            opt.display = {'', 'circle', 'clock', 'arrow', 'disk'};
            opt.color = 'g';
            opt.alpha = 0.2;
            opt.scale = 6;
            [opt,args] = tb_optparse(opt, varargin);

            if length(args) == 1 && isstr(args{1})
                opt.color = args{1};
                args = {};
            end

            holdon = ishold;
            hold on

            %s = 20/sqrt(pi);    % circle of same area as 20s x 20s square support region
            
            switch (opt.display)
                
                case 'clock'
                    plot_circle([ [features.u_]; [features.v_] ], ...
                        opt.scale*[features.scale_]', ...
                        'color', opt.color, args{:});
                    % plot radial lines
                    for f=features
                        plot([f.u_, f.u_+opt.scale*f.scale_*cos(f.theta_)], ...
                            [f.v_, f.v_+opt.scale*f.scale_*sin(f.theta_)], ...
                            'color', opt.color, args{:});
                    end
                    
                case 'arrow'
                    for f=features
                        quiver(f.u_, f.v_, ...
                            opt.scale*f.scale_.*cos(f.theta_), ...
                            opt.scale*f.scale_.*sin(f.theta_), ...
                            'color', opt.color, args{:});
                    end
                    
                otherwise
                    plot@ScalePointFeature(features, varargin{:});
            end
            if ~holdon
                hold off
            end
        end % plot

    end % methods
end % classdef
