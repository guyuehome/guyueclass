%IDISPLABEL  Display an image with mask
%
% IDISPLABEL(IM, LABELIMAGE, LABELS) displays only those image pixels which 
% belong to a specific class.  IM is a greyscale (HxW) or color (HxWx3) image, 
% and LABELIMAGE (HxW) contains integer pixel class labels for the
% corresponding pixels in IM.  The pixel classes to be displayed are given by
% LABELS which is either a scalar or a vector of class labels.  
% Non-selected pixels are displayed as white by default.
%
% IDISPLABEL(IM, LABELIMAGE, LABELS, BG) as above but the grey level of the 
% non-selected pixels is specified by BG in the range 0 to 1 for a float
% image or 0 to 255 for a uint8 image..
%
% Example::
%
% We will segment the image flowers into 7 color classes
%        cls = colorkemans(flowers, 7);
% where the matrix cls is the same size as flowers and the elements are
% the corresponding pixel class, a value in the range 1 to 7.  To display
% pixels of class 5 we use
%        idisplabel(flowers, cls, 5)
% and to display pixels belong to class 1 or 5 we use
%        idisplabel(flowers, cls, [1 5])
%
% See also IBLOBS, ICOLORIZE, COLORSEG.

function idisplabel(im, label, select, bg)

    if isscalar(select)
        mask = label == select;
    else
        mask = zeros(size(label));
        for s=select(:)',
            mask = mask | (label == s);
        end
    end
    
    if nargin < 4
        if isfloat(im)
            bg = 1;
        else
            bg = 255;
        end
    end
    
    if ndims(im) == 3
        mask = cat(3, mask, mask, mask);
    end
    
    im(~mask) = bg;
    idisp(im, 'nogui');
    shg
