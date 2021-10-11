%DISTANCE Euclidean distances between sets of points
%
% D = DISTANCE(A,B) is the Euclidean distances between L-dimensional points
% described by the matrices A (LxM) and B (LxN) respectively.  The distance 
% D is MxN and element D(I,J) is the distance between points A(I) and D(J).
%
% Example:: 
%    A = rand(400,100); B = rand(400,200);
%    d = distance(A,B);
%
% Notes::
% - This fully vectorized (VERY FAST!)
% - It computes the Euclidean distance between two vectors by:
%         ||A-B|| = sqrt ( ||A||^2 + ||B||^2 - 2*A.B )
%
% Author::
%  Roland Bunschoten,
%  University of Amsterdam,
%  Intelligent Autonomous Systems (IAS) group,
%  Kruislaan 403  1098 SJ Amsterdam,
%  tel.(+31)20-5257524,
%  bunschot@wins.uva.nl
%  Last Rev: Oct 29 16:35:48 MET DST 1999,
%  Tested: PC Matlab v5.2 and Solaris Matlab v5.3,
%  Thanx: Nikos Vlassis.
%
% See also CLOSEST.

function d = distance(a,b)

if (nargin ~= 2)
   error('Not enough input arguments');
end

if (size(a,1) ~= size(b,1))
   error('A and B should be of same dimensionality');
end

aa=sum(a.*a,1); bb=sum(b.*b,1); ab=a'*b; 
d = sqrt(abs(repmat(aa',[1 size(bb,2)]) + repmat(bb,[size(aa,2) 1]) - 2*ab));
