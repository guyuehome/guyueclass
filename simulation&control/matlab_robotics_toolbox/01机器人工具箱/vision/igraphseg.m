%IGRAPHSEG Graph-based image segmentation
%
% L = IGRAPHSEG(IM, K, MIN) is a graph-based segmentation of the color 
% image IM (HxWx3).  L (HxW) is an image where each element is the label 
% assigned to the corresponding pixel in IM.  K is the scale parameter, 
% and a larger value indicates a preference for larger regions, MIN is the
% minimum region size (pixels).
%
% L = IGRAPHSEG(IM, K, MIN, SIGMA) as above and SIGMA is the width of 
% a Gaussian which is used to initially smooth the image (default 0.5).
%
% [L,NREG] = IGRAPHSEG(IM, K, MIN, SIGMA) as above but NREG is the number of
% regions found.
%
% Example::
%     im = iread('58060.jpg');
%     [labels,maxval] = igraphseg(im, 1500, 100, 0.5);
%     idisp(labels)
%
% Reference::
%  "Efficient graph-based image segmentation",
%  P. Felzenszwalb and D. Huttenlocher, 
%  Int. Journal on Computer Vision,
%  vol. 59, pp. 167–181, Sept. 2004.
%
% Notes::
% - Requires a color uint8 image.
% - The hardwork is done by a MEX file in contrib/graphseg.
% - With zero smoothing the number of regions can be massive and can crash 
%   MATLAB.
%
% Author::
%  Pedro Felzenszwalb, 2006.
%
% See also ITHRESH, IMSER.

% wrapper function by Peter Corke

function [L_,M_] = igraphseg(im, K, mn, sigma)
    if size(im, 3) ~= 3
        error('MVTB:igraphseg:badarg', 'Input image must be color');
    end
    
    if ~isa(im, 'uint8')
        error('MVTB:igraphseg:badarg', 'image must be of uint8 type');
    end
    
    if nargin < 3
        error('MVTB:igraphseg:badarg', 'must specify image, K, min');
    end  
    
    if nargin < 4
        sigma = 0.5;
    end
    
    [L,M] = graphseg(im, K, mn, sigma);
    
    % labels are not sequential but sparsely distributed over a large
    % numeric range
    
    uniqLabels = count_unique(L(:));
    
    % create a map
    map=containers.Map(uniqLabels, 1:M);
    L = cell2mat( values(map, num2cell(L) ) );  % map the values
    
    if nargout >= 1
        L_ = L;
    end
    if nargout > 1
        M_ = M;
    end