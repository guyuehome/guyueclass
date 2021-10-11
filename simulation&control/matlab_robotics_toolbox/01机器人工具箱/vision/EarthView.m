%EarthView Image from Google maps
%
% A concrete subclass of ImageSource that acquires images from Google
% maps.
%
% Methods::
% grab    Grab a frame from Google maps
% size    Size of image
% close   Close the image source
% char    Convert the object parameters to human readable string
%
% Examples::
% Create an EarthView camera
%          ev = EarthView();
% Zoom into QUT campus in Brisbane
%          ev.grab(-27.475722,153.0285, 17);
% Show aerial view of Brisbane in satellite and map view
%          ev.grab('brisbane', 14)
%          ev.grab('brisbane', 14, 'map')
%
% Notes::
% - Google limit the number of map queries limit to 1000 unique (different) 
%   image requests per viewer per day.  A 403 error is returned if the daily 
%   quota is exceeded.
% - Maximum size is 640x640 for free access, business users can get more.
% - There are lots of conditions on what you can do with the images, 
%   particularly with respect to publication.  See the Google web site for 
%   details.
%
% Author::
%  Peter Corke, with some lines of code from from get_google_map by Val
%  Schmidt.
%
% See also ImageSource.

%TODO:
% map features/element control
% street view
% roadmap, satellite, terrain, hybrid

classdef EarthView < ImageSource

% e = EarthView() is an object that returns images of the Earth's surface
% obtained from Google Maps.
%
% e = EarthView(googlekey) as above but the google key string is passed in.
%
% e.grab(lat, lon, zoom, options)

    properties
        key
        type
        scale
    end

    methods
        function ev = EarthView(varargin)
        %EarthView.EarthView Create EarthView object
        %
        % EV = EarthView(OPTIONS)
        %
        % Options::
        % 'satellite'    Retrieve satellite image
        % 'map'          Retrieve map image
        % 'hybrid'       Retrieve satellite image with map overlay
        % 'scale'        Google map scale (default 18)
        % 'width',W      Set image width to W (default 640)
        % 'height',H     Set image height to H (default 640)
        % 'key',S        The Google maps key string
        %
        % see also options for ImageSource.
        %
        % Notes::
        % - A key is required before you can use the Google Static Maps API.  
        %   The key is a long string that can be passed to the constructor or 
        %   saved as an environment variable GOOGLE_KEY.  You need a Google 
        %   account before you can register for a key.
        %
        % Notes::
        % - Scale is 1 for the whole world, ~20 is about as high a resolution
        %   as you can get.
        %
        % See also ImageSource, EarthView.grab.
        
        %TODO
        %  clone this for new StreetView API
        %  method to return lat/long or NE matrices corresp to pixels
        
            ev = ev@ImageSource(varargin{:});
            
            opt.type = {'satellite', 'map', 'hybrid'};
            opt.scale = 18;
            opt.key = '';
            
            [opt,args] = tb_optparse(opt, varargin);
            
            ev.type = opt.type;
            ev.scale = 1;
          

            % set default size params if not set
            if isempty(ev.width)
                ev.width = 640;
                ev.height = 640;
            end

            if isempty(opt.key)
                opt.key = getenv('GOOGLE_KEY');
            end
            ev.key = opt.key;
            
        end

        function [im,E,N] = grab(ev, varargin)
            % EarthView.grab Grab an aerial image
            %
            % IM = EV.grab(LAT, LONG, OPTIONS) is an image of the Earth
            % centred at the geographic coordinate (lat, long).
            %
            % IM = EarthView.grab(LAT, LONG, ZOOM, OPTIONS) as above with the 
            % specified zoom.  ZOOM is an integer between 1 (zoom right out)
            % to a maximum of 18-20 depending on where in the world you
            % are looking.
            %
            % [IM,E,N] = EarthView.grab(LAT, LONG, OPTIONS) as above but also
            % returns the estimated easting E and northing N.  E and N are both 
            % matrices, the same size as IM, whose corresponding elements are 
            % the easting and northing are the coordinates of the pixel.
            %
            % [IM,E,N] = EarthView.grab(NAME, OPTIONS) as above but uses a
            % geocoding web site to resolve the name to a location.
            % 
            % Options::
            % 'satellite'      Retrieve satellite image
            % 'map'            Retrieve map image
            % 'hybrid'         Retrieve satellite image with map overlay
            % 'roadmap'        Retrieve a binary image that shows only roads, no labels.
            %                  Roads are white, everything else is black.
            % 'noplacenames'   Don't show placenames.
            % 'noroadnames'    Don't show roadnames.
            %
            %
            % Examples::
            % Zoom into QUT campus in Brisbane
            %          ev.grab(-27.475722,153.0285, 17);
            % Show aerial view of Brisbane in satellite and map view
            %          ev.grab('brisbane', 14)
            %          ev.grab('brisbane', 14, 'map')
            %
            % Notes::
            % - If northing/easting outputs are requested the function
            %   deg2utm is required (from MATLAB Central)
            % - The easting/northing is somewhat approximate, see
            %   get_google_map on MATLAB Central.
            % - If no output argument is given the image is displayed using idisp.

            opt.type = {'satellite', 'map', 'hybrid', 'roadmap'};
            opt.scale = ev.scale;
            opt.roadnames = true;
            opt.placenames = true;
            opt.landscape = false;
            opt.roadmap = false;
            opt.onlyroads = false;

            [opt,args] = tb_optparse(opt, varargin);

            if strcmp(opt.type, 'roadmap')
                opt.type = 'map';
                opt.onlyroads = true;
            end

            % build the URL
            if ischar(args{1})
                % given a string name, do a geocode lookup
                place = args{1};
                zoom = args{2};

                % build the URL, and load the XML document
                url = sprintf('http://maps.googleapis.com/maps/api/geocode/xml?address=%s&sensor=false', place);
                doc = xmlread(url);

                % walk the XML document
                locations = doc.getElementsByTagName('location')

                if locations.getLength > 1
                    fprintf('%d places called %s found\n', locations.getLength, place);
                end

                location = locations.item(0);   % take the first return

                node_lat = location.getElementsByTagName('lat');
                el = node_lat.item(0);
                lat = str2num( el.getFirstChild.getData );

                node_lon = location.getElementsByTagName('lng');
                el = node_lon.item(0);
                lon = str2num( el.getFirstChild.getData );
            else
                lat = args{1};
                lon = args{2};
                if length(args) == 3
                    zoom = args{3};
                else
                    zoom = 18;
                end
                
            end
            % now read the map
            baseurl = sprintf('http://maps.google.com/maps/api/staticmap?center=%.6f,%.6f&zoom=%d&size=%dx%d&scale=%d&format=png&maptype=%s&key=%s&sensor=false', lat, lon, zoom, ev.width, ev.height, opt.scale, opt.type, ev.key);

            opturl = '';
            
            if ~opt.roadnames
                opturl = strcat(opturl, '&style=feature:road|element:labels|visibility:off');
            end
            if ~opt.placenames
                opturl = strcat(opturl, '&style=feature:administrative|element:labels.text|visibility:off&style=feature:poi|visibility:off');
            end
            
            if opt.onlyroads
                opturl = strcat(opturl, ['&style=feature:landscape|element:geometry.fill|color:0x000000|visibility:on'...
                    '&style=feature:landscape|element:labels|visibility:off'...
                    '&style=feature:administrative|visibility:off'...
                    '&style=feature:road|element:geometry.fill|color:0xffffff|visibility:on'...
                    '&style=feature:road|element:labels|visibility:off'...
                    '&style=feature:poi|element:all|visibility:off'...
                    '&style=feature:transit|element:all|visibility:off'...
                    '&style=feature:water|element:all|visibility:off'...
                    ]);
            end
            
            %opturl = urlencode(opturl)
            opturl = strrep(opturl, '|', '%7C');
            
            opturl

            [idx,cmap] = imread([baseurl opturl], 'png');
            cmap = iint(cmap);

            % apply the color map
            view = cmap(idx(:)+1,:);

            % knock it into shape
            view = shiftdim( reshape(view', [3 size(idx)]), 1);
            view = ev.convert(view);

            if nargout == 0
                idisp(view);
            else
                im = view;
            end
            
            if nargout > 1
                % compute the northing/easting at each pixel.
                %
                % the following lines of code from get_google_map by Val Schmidt 
                % ESTIMATE BOUNDS OF IMAGE:
                %
                % Normally one must specify a center (lat,lon) and zoom level for a map.
                % Zoom Notes:
                % ZL: 15, hxw = 640x640, image dim: 2224.91 x 2224.21 (mean 2224.56)
                % ZL: 16, hxw = 640x640, image dim: 1128.01m x 1111.25m (mean 1119.63)
                % This gives an equation of roughly (ZL-15)*3.4759 m/pix * pixels
                % So for an image at ZL 16, the LHS bound is 
                % LHS = centerlonineastings - (zl-15) * 3.4759 * 640/2;
                [lonutm latutm zone] = deg2utm(lat,lon);
                Hdim = (2^15/2^zoom) * 3.4759 * ev.width;
                Vdim = (2^15/2^zoom) * 3.4759 * ev.height;

                ell = lonutm - Hdim/2;
                nll = latutm - Vdim/2;
                eur = lonutm + Hdim/2;
                nur = latutm + Vdim/2;

                Nvec = linspace(nur,nll,ev.height); % lat is highest at image row 1
                Evec = linspace(ell,eur,ev.width);
                [E,N] = meshgrid(Evec, Nvec);
            end
        end

        function close()
        end

        function s = char(m)
        %EarthView.char Convert to string
        %
        % EV.char() is a string representing the state of the EarthView object in 
        % human readable form.
        %
        % See also EarthView.display.

            s = sprintf('EarthView: %d x %d', ...
                m.width, m.height);
            s2 = char@ImageSource(m);
            if ~isempty(s2)
                s = strvcat(s, strcat(' - ', s2));
            end
        end

        function paramSet(varargin)
        end

    end
end
