%SiftPointFeature.support Support region of feature
%
% OUT = F.support(IM, W) is an image of the support region of the 
% feature F, extracted from the image IM in which the feature appears.
% The support region is scaled to WxW and rotated so that the feature's
% orientation axis is upward.
%
% OUT = F.support(IMAGES, W) as above but if the features were extracted
% from an image sequence IMAGES then the feature is extracted from the 
% appropriate image in the same sequence.
%
% [OUT,T] = F.support(IMAGES, W) as above but returns the pose of the feature
% as a 3x3 homogeneous transform in SE(2) that comprises the feature position
% and orientation.
%
% F.support(IM, W) as above but the support region is displayed.
%
% See also SiftPointFeature.

function [out,TT] = support(sf, images, N)

    if nargin < 3
        N = 50;
    end

    im = images(:,:,sf.image_id_);

    d = 2*sf.scale_;

    [Uo,Vo] = imeshgrid(N, N);

    T = se2(sf.u_, sf.v_, sf.theta_) * diag([d/N,d/N,1]) * se2(-N/2, -N/2);

    UV = transformp(T, [Uo(:) Vo(:)]');
    U = reshape(UV(1,:), size(Uo));
    V = reshape(UV(2,:), size(Vo));

    [Ui,Vi] = imeshgrid(im);

    im2 = interp2(Ui, Vi, idouble(im), U, V);

    if nargout == 0
        idisp(im2)
    elseif nargout == 1
        out = im2;
    elseif nargout == 2
        out = im2;
        TT = T;
    end
