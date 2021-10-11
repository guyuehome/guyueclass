%IRECTIFY Rectify stereo image pair
%
% [OUT1,OUT2] = IRECTIFY(F, M, IM1, IM2) is a rectified pair of images
% corresponding to IM1 and IM2.  F (3x3) is the fundamental matrix relating 
% the two views and M is a FeatureMatch object containing point correspondences 
% between the images.
%
% [OUT1,OUT2,H1,H2] = IRECTIFY(F, M, IM1, IM2) as above but also returns
% the homographies H1 and H2 that warp IM1 to OUT1 and IM2 to OUT2 respectively.
%
% Notes::
% - The resulting image pair are epipolar aligned, equivalent to the view
%   if the two original camera axes were parallel.
% - Rectified images are required for dense stereo matching.
% - The effect of lense distortion is not removed, use the camera calibration
%   toolbox to unwarp each image prior to rectification.
% - The resulting images may have negative disparity.
% - Some output pixels may have no corresponding input pixels and will be
%   set to NaN.
%
% See also FeatureMatch, ISTEREO, HOMWARP, CentralCamera.


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

function [Img1_new, Img2_new, H12,H21] = irectify(F, m, Img1, Img2)
% http://se.cs.ait.ac.th/cvwiki/matlab:tutorial:rectification

F12 = F';

[rows,cols,depth] = size(Img1);

% Get homographies.

x1 = e2h( m.p1 );
x2 = e2h( m.p2 );

[H12,H21,bSwap] = rectify_homographies( F12, x1, x2, rows, cols );

[w1,off1] = homwarp(H12, Img1, 'full');
[w2,off2] = homwarp(H21, Img2, 'full');

% fix the vertical alignment of the images by padding
dy = off1(2) - off2(2);
if dy < 0
    w1 = ipad(w1, 'b', -dy);
    w2 = ipad(w2, 't', -dy);
else
    w1 = ipad(w1, 't', dy);
    w2 = ipad(w2, 'b', dy);
end

[w1,w2] = itrim(w1, w2);

if nargout == 0
    stview(w1, w2)
else
    Img1_new = w1;
    Img2_new = w2;
end



%-----------------------------------------------------------------------------

function [H1,H2,bSwap] = rectify_homographies( F, x1, x2, rows, cols )

  % F: a fundamental matrix

  % x1 and x2: corresponding points such that x1_i' * F * x2_i = 0

  % Initialize

  H1 = [];
  H2 = [];
  bSwap = 0;

  % Center of image

  cy = round( rows/2 );
  cx = round( cols/2 );

  % Fix F to be rank 2 to numerical accuracy

  [U,D,V] = svd( F );
  D(3,3) = 0;
  F = U*D*V';

  % Get epipole.  e12 is the epipole in image 1 for camera 2.

  e12 = null( F' );             % Epipole in image 1 for camera 2
  e21 = null( F );              % Epipole in image 2 for camera 1

  % Put epipoles in front of camera

  if e12 < 0, e12 = -e12; end;
  if e21 < 0, e21 = -e21; end;

  % Make sure the epipoles are inside the images

  check_epipoles_in_image( e12, e21, rows, cols );

  % Check that image 1 is to the left of image 2

%   if e12(1)/e12(3) < cx
%     fprintf( 1, 'Swapping left and right images...\n' );
%     tmp = e12;
%     e12 = e21;
%     e21 = tmp;
%     F = F';
%     bSwap = 1;
%   end;

  % Now we have
  % F' * e12 = 0, 
  % F  * e21 = 0,

  % Let's get the rectifying homography Hprime for image 1 first

  Hprime = map_to_infinity( e12, cx, cy );
  e12_new = Hprime * e12;
  % Normalize Hprime so that Hprime*eprime = (1,0,0)'
  Hprime = Hprime / e12_new(1);
  e12_new = Hprime * e12;
  fprintf( 1, 'Epipole 1/2 mapped to infinity: (%g, %g, %g)\n', e12_new );

  % Get canonical camera matrices for F12 and compute H0, one possible
  % rectification homography for image 2

  [P,Pprime] = get_canonical_cameras( F );
  M = Pprime(:,1:3);
  H0 = Hprime * M;

  % Test that F12 is a valid F for P,Pprime

  test_p_f( P, Pprime, F );

  % Now we need to find H so that the epipolar lines match
  % each other, i.e., inv(H)' * l = inv(Hprime)' * lprime
  % and the disparity is minimized, i.e.,
  % min \sum_i d(H x_i, Hprime xprime_i)^2

  % Transform data initially according to Hprime (img 1) and H0 (img 2)

  x1hat = Hprime * x1;
  x1hat = x1hat ./ repmat( x1hat(3,:), 3, 1 );
  x2hat = H0 * x2;
  x2hat = x2hat ./ repmat( x2hat(3,:), 3, 1 );
  rmse_x = sqrt( mean( (x1hat(1,:) - x2hat(1,:) ).^2 ));
  rmse_y = sqrt( mean( (x1hat(2,:) - x2hat(2,:) ).^2 ));
  fprintf( 1, 'Before Ha, RMSE for corresponding points in Y: %g X: %g\n', ...
           rmse_y, rmse_x );

  % Estimate [ a b c ; 0 1 0 ; 0 0 1 ] aligning H, Hprime

  n = size(x1,2);
  A = [ x2hat(1,:)', x2hat(2,:)', ones(n,1) ];
  b = x1hat(1,:)';
  abc = A\b;
  HA = [ abc' ; 0 1 0 ; 0 0 1 ];
  H = HA*H0;
  x2hat = H * x2;
  x2hat = x2hat ./ repmat( x2hat(3,:), 3, 1 );
  rmse_x = sqrt( mean(( x1hat(1,:) - x2hat(1,:) ).^2 ));
  rmse_y = sqrt( mean(( x1hat(2,:) - x2hat(2,:) ).^2 ));
  fprintf( 1, 'After Ha, RMSE for corresponding points in Y: %g X: %g\n', ...
           rmse_y, rmse_x );

  % Return the homographies as appropriate

  if bSwap
    H1 = H;
    H2 = Hprime;
  else
    H1 = Hprime;
    H2 = H;
  end;

%-----------------------------------------------------------------------------

function check_epipoles_in_image( e1, e2, rows, cols )

  % Check whether given epipoles are in the image or not

  if abs( e1(3) ) < 1e-6 & abs( e2(3) ) < 1e-6, return; end;

  e1 = e1 / e1(3);
  e2 = e2 / e2(3);
  if ( e1(1) <= cols & e1(1) >= 1 & e1(2) <= rows & e1(2) >= 1 ) | ...
     ( e2(1) <= cols & e2(1) >= 1 & e2(2) <= rows & e2(2) >= 1 )
    err_msg = sprintf( 'epipole (%g,%g) or (%g,%g) is inside image', ...
                       e1(1:2), e2(1:2) );
    error( [ err_msg, ' -- homography does not work in this case!' ] );
  end;

%-----------------------------------------------------------------------------

function [P,Pprime] = get_canonical_cameras( F )

  % Get the "canonical" cameras for given fundamental matrix
  % according to Hartley and Zisserman (2004), p256, Result 9.14

  % But ensure that the left 3x3 submatrix of Pprime is nonsingular
  % using Result 9.15, that the general form is
  % [ skewsym( e12 ) * F + e12 * v', k * e12 ] where v is an arbitrary
  % 3-vector and k is an arbitrary scalar

  P = [ 1 0 0 0
        0 1 0 0
        0 0 1 0 ];

  e12 = null( F' );
  M = skew( e12 ) * F + e12 * [1 1 1];
  Pprime = [ M, e12 ];

%-----------------------------------------------------------------------------

function test_p_f( P, Pprime, F )

  % Test that camera matrices Pprime and P are consistent with
  % fundamental matrix F
  % Meaning  (Pprime*X)' * F * (P*X) = 0,  for all X in 3space

  % Get the epipole in camera 1 for camera 2

  C2 = null( P );
  eprime = Pprime * C2;

  % Construct F from Pprime, P, and eprime

  Fhat = skew( eprime ) * Pprime * pinv( P );

  % Check that it's close to F

  alpha = Fhat(:)\F(:);
  if norm( alpha*Fhat-F ) > 1e-10
    fprintf( 1, 'Warning: supplied camera matrices are inconsistent with F\n' );
  else
    fprintf( 1, 'Supplied camera matrices OK\n' );
  end;

%-----------------------------------------------------------------------------

function H = map_to_infinity( x, cx, cy )

  % Given a point and the desired origin (point of minimum projective
  % distortion), compute a homograph H = G*R*T taking the point to the
  % origin, rotating it to align with the X axis, then mapping it to
  % infinity.

  % First map cx,cy to the origin

  T = [ 1 0 -cx
        0 1 -cy
        0 0 1 ];
  x = T * x;

  % Now rotate the translated x to align with the X axis.

  cur_angle = atan2( x(2), x(1) );
  R = [ cos( -cur_angle ), -sin( -cur_angle ), 0
        sin( -cur_angle ),  cos( -cur_angle ), 0
                        0,                  0, 1 ];
  x = R * x;

  % Now the transformation G mapping x to infinity

  if abs( x(3)/norm(x) ) < 1e-6
      % It's already at infinity
      G = eye(3)
  else
      f = x(1)/x(3);
      G = [    1   0  0
               0   1  0
             -1/f  0  1 ];
  end;

  H = G*R*T;
