%Matlab MEX file to compute 2-D histogram.
%
%[h,vx,vy] = hist2d(x,y)
%
%or
%
%[h,vx,vy] = hist2d(x,y,[x0 dx nx],[y0 dy ny])
%
%Inputs:
%        x,y     data points.  {x(i),y(i)} is a single data point.
%        x0      lowest x bin's lower edge
%        dx      x bin width
%        nx      number of x bins
%        y0      lowest y bin's lower edge
%        dy      y bin width
%        ny      number of y bins
%        [x0,dx,nx] and [y0,dy,ny] default = [0,1,256]
%Outputs:
%        h       histogram matrix.  h(i,j) = number of data points
%                satisfying vx(j) <= x < vx(j+1) and vy(i) <= y < vy(i+1).
%        vx      bin lower x-ordinates (one for each column of h)
%        vy      bin lower y-ordinates (one for each row of h)
%
% Notes::
% - Data vectors x and y must be double
%
% Author::
%Michael Maurer, 7 October 1994. 
%Copyright 1994 by Michael Maurer.
