#include <math.h>
#include "mex.h"

#define MAX(x,y)     (((x) > (y)) ? (x) : (y))
#define MIN(x,y)     (((x) < (y)) ? (x) : (y))
#define IDX(i,j,m)   ((m)*(j)+(i))

static void
hist2d(double *x,
       double *y,
       unsigned int n,
       double x0,
       double dx,
       unsigned int ndx,
       double y0,
       double dy,
       unsigned int ndy,
       double *h)
{
   unsigned int i,ix,iy;

   for (i=0; i<ndx*ndy; i++)
      h[i]=0;
   for (i=0; i<n; i++) {
      ix = (x[i] - x0) / dx;
      iy = (y[i] - y0) / dy;
      if (ix < ndx && iy < ndy)
         h[IDX(iy,ix,ndy)]++;
   }
}

#define IN_X prhs[0]
#define IN_Y prhs[1]
#define IN_XBIN prhs[2]
#define IN_YBIN prhs[3]
#define OUT_H plhs[0]
#define OUT_VX plhs[1]
#define OUT_VY plhs[2]

// TODO
//  handle uint8 images
//  allow second argument to be standard MATLAB range a:b:c
void
mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
   double *px,*py,*ph,*pvx,*pvy,*p,x0,dx,y0,dy;
   unsigned int mx,nx,my,ny,mh,nh,m,n,i,ndx,ndy;

   if (nrhs < 2)
      mexErrMsgTxt("hist2d: Two input arguments required.");
   if ( !(mxIsDouble(IN_X) && mxIsDouble(IN_Y)) )
      mexErrMsgTxt("hist2d: Arguments must be double.");
   
   px = mxGetPr(IN_X);
   mx = mxGetM(IN_X);
   nx = mxGetN(IN_X);

   py = mxGetPr(IN_Y);
   my = mxGetM(IN_Y);
   ny = mxGetN(IN_Y);

   if (mx!=my || nx!=ny)
      mexErrMsgTxt("X and Y must be same size.");

   x0 = 1;
   dx = 1;
   ndx = 256;
   if (nrhs >= 3) {
      p = mxGetPr(IN_XBIN);
      m = mxGetM(IN_XBIN);
      n = mxGetN(IN_XBIN);
      if (m*n != 3)
         mexErrMsgTxt("Bin specification must be a 3 element vector.");
      x0 = p[0];
      dx = p[1];
      ndx = p[2];
   }
   
   y0 = 1;
   dy = 1;
   ndy = 256;
   if (nrhs >= 4) {
      p = mxGetPr(IN_YBIN);
      m = mxGetM(IN_YBIN);
      n = mxGetN(IN_YBIN);
      if (m*n != 3)
         mexErrMsgTxt("Bin specification must be a 3 element vector.");
      y0 = p[0];
      dy = p[1];
      ndy = p[2];
   }
   
   mh = ndy;
   nh = ndx;
   OUT_H = mxCreateDoubleMatrix(mh,nh,mxREAL);
   OUT_VX = mxCreateDoubleMatrix(1,ndx,mxREAL);
   OUT_VY = mxCreateDoubleMatrix(ndy,1,mxREAL);
   ph = mxGetPr(OUT_H);
   pvx = mxGetPr(OUT_VX);
   pvy = mxGetPr(OUT_VY);
   for (i=0; i<ndx; i++)
      pvx[i] = x0 + i*dx;
   for (i=0; i<ndy; i++)
      pvy[i] = y0 + i*dy;

   hist2d(px,py,mx*nx,
          x0,dx,ndx,
          y0,dy,ndy,
          ph);
}
