// patch all functions as per ZNCC
// replace mean calculation with an integral filter

#ifdef STANDALONE
# include <stdio.h>
# include <stdlib.h>
# include "area.h"
# define CALLOC calloc
#else
# include "mex.h"
# define CALLOC mxCalloc
#endif

#define TIMING

#ifdef __LCC__
#undef TIMING
#endif

#ifdef _W64
#undef TIMING
#endif

#ifdef TIMING
#include <sys/time.h>
#endif
#include <math.h>
#include <string.h>

typedef unsigned char PIX;
double time_diff(struct timeval *tv1, struct timeval *tv2);
static void compute_means( float *image_l, float *image_r, int wx, int wy, int width, int height,
    float *mean_l, float *mean_r);

#ifdef STANDALONE
#define INFINITY    FP_INFINITE
#define NAN         FP_NAN
#endif

#ifdef __LCC__
#define INFINITY    mxGetInf()
#define NAN         mxGetNaN()
#endif

#ifdef _W64
#define INFINITY    mxGetInf()
#define NAN         mxGetNaN()
#endif
/* matrix reference macros Matlab element order (column wise) */
#define REF(p,x,y)  p[(y)*width+x]
#define REF_ML(p,x,y)  p[(x)*height+(y)]
#define REF3(p,x,y,z)  p[((z)*width+(x))*height+(y)]

void 
stereo_matching_sad(
    float *image_l, 
    float *image_r, 
    double *score, 
    int width, 
    int height, 
    int wx, 
    int wy, 
    int disparity_min, 
    int disparity_max)
{
	unsigned int    x;
	int             i, j, y, left, right, wx2, wy2;
	float  *p_l, *p_r;
	float          pix_l, pix_r;
	double          sum;
	int             disp, right_lim, left_lim, ndisp;

    ndisp = disparity_max - disparity_min;
    if (ndisp < 0)
        ndisp = -ndisp;
    ndisp++;

	wx2 = (wx - 1) / 2;
	wy2 = (wy - 1) / 2;
	left = wx2;
	right = width - wx2;

	for (disp = disparity_min; disp < disparity_max; disp++) {

		for (y = wy2; y < (height-wy2); y++) {
			if (disp < 0) {
				p_l = image_l + y * width + wx2;
				p_r = image_r + y * width - disp + wx2;
			}
			else {
				p_l = image_l + y * width + disp + wx2;
				p_r = image_r + y * width + wx2;
			}

			right_lim = (disp < 0) ? right + disp : right;
			left_lim = (disp < 0) ? left : left + disp;

			for (x = left_lim; x < right_lim; x++) {

				sum = 0;

                /* inner matching loop */
				for (i = -wx2; i <= wx2; i++) {      // x
					for (j = -wy2; j <= wy2; j++) {  // y
						pix_l = REF(p_l, i,j);
						pix_r = REF(p_r, i,j);

                        if (pix_l > pix_r)
                            sum += pix_l - pix_r;
                        else
                            sum += pix_r - pix_l;
					}
                }

                REF3(score, x,y,disp) = sum;

				p_l++;
				p_r++;
			}	/* for x */
		}		/* for y */
	}			/* for disparity */
}

void 
stereo_matching_ssd(
    float *image_l, 
    float *image_r, 
    double *score, 
    int width, 
    int height, 
    int wx, 
    int wy, 
    int disparity_min, 
    int disparity_max)
{
	unsigned int    x;
	int             i, j, y, left, right, wx2, wy2;
	float  *p_l, *p_r;
	float          pix_l, pix_r;
	double          sum;
	int             disp, right_lim, left_lim, ndisp;

    ndisp = disparity_max - disparity_min;
    if (ndisp < 0)
        ndisp = -ndisp;
    ndisp++;

	wx2 = (wx - 1) / 2;
	wy2 = (wy - 1) / 2;
	left = wx2;
	right = width - wx2;

	for (disp = disparity_min; disp < disparity_max; disp++) {

		for (y = wy2; y < (height-wy2); y++) {
			if (disp < 0) {
				p_l = image_l + y * width + wx2;
				p_r = image_r + y * width - disp + wx2;
			}
			else {
				p_l = image_l + y * width + disp + wx2;
				p_r = image_r + y * width + wx2;
			}

			right_lim = (disp < 0) ? right + disp : right;
			left_lim = (disp < 0) ? left : left + disp;

			for (x = left_lim; x < right_lim; x++) {

				sum = 0;

                /* inner matching loop */
				for (i = -wx2; i <= wx2; i++) {      // x
					for (j = -wy2; j <= wy2; j++) {  // y
						pix_l = REF(p_l, i,j);
						pix_r = REF(p_r, i,j);

						sum += (pix_l - pix_r) * (pix_l - pix_r);
					}
                }

                REF3(score, x,y,disp) = sum;

				p_l++;
				p_r++;
			}	/* for x */
		}		/* for y */
	}			/* for disparity */
}

void 
stereo_matching_ncc(
    float *image_l, 
    float *image_r, 
    double *score, 
    int width, 
    int height, 
    int wx, 
    int wy, 
    int disparity_min, 
    int disparity_max)
{
	unsigned int    x;
	int             i, j, y, left, right, wx2, wy2;
	float  *p_l, *p_r;
	float          pix_l, pix_r;
	double          ncc, den;
	double          sum, sum_l, sum_r;
	int             disp, right_lim, left_lim, ndisp;

    ndisp = disparity_max - disparity_min;
    if (ndisp < 0)
        ndisp = -ndisp;
    ndisp++;

	wx2 = (wx - 1) / 2;
	wy2 = (wy - 1) / 2;
	left = wx2;
	right = width - wx2;

	for (disp = disparity_min; disp < disparity_max; disp++) {

		for (y = wy2; y < (height-wy2); y++) {
			if (disp < 0) {
				p_l = image_l + y * width + wx2;
				p_r = image_r + y * width - disp + wx2;
			}
			else {
				p_l = image_l + y * width + disp + wx2;
				p_r = image_r + y * width + wx2;
			}

			right_lim = (disp < 0) ? right + disp : right;
			left_lim = (disp < 0) ? left : left + disp;

			for (x = left_lim; x < right_lim; x++) {

				sum = 0;
				sum_l = 0;
				sum_r = 0;

                /* inner matching loop */
				for (i = -wx2; i <= wx2; i++) {      // x
					for (j = -wy2; j <= wy2; j++) {  // y
						pix_l = REF(p_l, i,j);
						pix_r = REF(p_r, i,j);

						sum += pix_l * pix_r;
						sum_l += pix_l * pix_l;
						sum_r += pix_r * pix_r;
					}
                }

				den = sqrt(sum_l * sum_r);

				if (den != 0)
					ncc = sum / den;
				else
					ncc = INFINITY;

                REF3(score, x,y,disp) = ncc;

				p_l++;
				p_r++;
			}	/* for x */
		}		/* for y */
	}			/* for disparity */
}

void 
stereo_matching_zncc(
    float *image_l, 
    float *image_r, 
    double *score, 
    int width, 
    int height, 
    int wx, 
    int wy, 
    int disparity_min, 
    int disparity_max)
{
	unsigned int    x;
	int             i, j, y, left, right, wx2, wy2;
	float  *p_l, *p_r;
	float          pix_l, pix_r;
    float          *mean_l, *mean_r,
	               *pmean_l, *pmean_r;

	double          ncc, den;
	double          sum, sum_l, sum_r;
	int             disp, right_lim, left_lim, ndisp;

	mean_l = (float *) CALLOC(width * height, sizeof(double));
	mean_r = (float *) CALLOC(width * height, sizeof(double));
	compute_means(image_l, image_r, wx, wy, width, height, mean_l, mean_r);

    // following line to all sim metrics PIC
    ndisp = disparity_max - disparity_min + 1;
    if (ndisp < 0)
        ndisp = -ndisp;
    ndisp++;

	wx2 = (wx - 1) / 2;
	wy2 = (wy - 1) / 2;
	left = wx2;
	right = width - wx2;

	for (disp = disparity_min; disp < disparity_max; disp++) {

		for (y = wy2; y < (height-wy2); y++) {
			if (disp < 0) {
				p_l = image_l + y * width + wx2;
				p_r = image_r + y * width - disp + wx2;
				pmean_l = mean_l + y * width + wx2;
				pmean_r = mean_r + y * width - disp + wx2;
			}
			else {
				p_l = image_l + y * width + disp + wx2;
				p_r = image_r + y * width + wx2;
				pmean_l = mean_l + y * width + disp + wx2;
				pmean_r = mean_r + y * width + wx2;
			}

			right_lim = (disp < 0) ? right + disp : right;
			left_lim = (disp < 0) ? left : left + disp;

			for (x = left_lim; x < right_lim; x++) {

				sum = 0;
				sum_l = 0;
				sum_r = 0;

                /* inner matching loop */
				for (i = -wx2; i <= wx2; i++) {      // x
					for (j = -wy2; j <= wy2; j++) {  // y
						pix_l = REF(p_l, i,j) - *pmean_l;
						pix_r = REF(p_r, i,j) - *pmean_r;

						sum += pix_l * pix_r;
						sum_l += pix_l * pix_l;
						sum_r += pix_r * pix_r;
					}
                }

				den = sqrt(sum_l * sum_r);

				if (den != 0)
					ncc = sum / den;
				else
					ncc = INFINITY;

                // following line to all sim metrics PIC
                REF3(score, x,y,disp-disparity_min) = ncc;

				p_l++;
				p_r++;
				pmean_l++;
				pmean_r++;
			}	/* for x */
		}		/* for y */
	}			/* for disparity */

	mxFree(mean_l);
	mxFree(mean_r);
}				/* match_ZNCC_l */

/***************************************************************************************
 compute_means
 ***************************************************************************************/
static void 
compute_means(
    float *image_l,
    float *image_r,
    int wx,
    int wy,
    int width,
    int height,
    float *mean_l,
    float *mean_r)
{
	int             wx2, wy2,
	                x, y, i, j;
	double         size_sq;

	size_sq = wx * wy;
	wx2 = (wx - 1) / 2;
	wy2 = (wy - 1) / 2;

	for (y = wy2; y < (height-wy2); y++)
		for (x = wx2; x < (width-wx2); x++) {
            float  sum_r = 0.0, sum_l = 0.0;

			for (i = -wx2; i <= wx2; i++)
				for (j = -wy2; j <= wy2; j++) {
					sum_l += REF(image_l, x+i, y+j);
					sum_r += REF(image_r, x+i, y+j);
				}

			REF(mean_r, x,y)  = sum_r / size_sq;
			REF(mean_l, x, y) = sum_l / size_sq;
		}
}

/*
 * disp = stereo(L, R, w, disprange, metric)
 *
 *  L and R are images (uint8 or double) of size NxM
 *  w is a 1-vector wx=wy=w or a 2-vector [wx wy] window size
 *  disprange is a 2-vector, D = abs(disprange(2)-disprange(1))
 *
 *  disp is a NxMxD matrix of int16
 */
void 
mexFunction(
		 int nlhs, mxArray *plhs[],
		 int nrhs, const mxArray *prhs[])
{
  unsigned int width_l, height_l, height_r, width_r, width, height;
  int wx, wy, i, j, dispmin, dispmax;
  float *leftI, *rightI, *leftI_ptr, *rightI_ptr; 
  double *p, *scoresD;
  mwSize    dims[3];
  double Z_NAN = mxGetNaN();
  double Z_INFINITY = mxGetInf();
#ifdef  TIMING
  struct timeval	t0, t1, t2, t3, t4;
#endif
  char  metric[8];


#ifdef  TIMING
  gettimeofday (&t0, NULL);
#endif

  /* Check for proper number of arguments */

  switch (nrhs) {
  case 4:
    strcpy(metric, "zncc");
    break;
  case 5:
    if (!mxIsChar(prhs[4]))
        mexErrMsgTxt("metric must be specified by a string");
    mxGetString(prhs[4], metric, 8);
    break;
  default:
    mexErrMsgTxt("expecting 4 or 5 arguments");
    return;
  }

  /* Check that input images are same size */
  height_l = mxGetM(prhs[0]);
  width_l = mxGetN(prhs[0]);
  height_r = mxGetM(prhs[1]);
  width_r = mxGetN(prhs[1]);

  if ((height_l != height_r) || (width_l != width_r))
    mexErrMsgTxt("Left and right images must be the same size");

  height = height_l;
  width = width_l;

  /* get window size */
  switch (mxGetNumberOfElements(prhs[2])) {
  case 1:
    wx = wy = mxGetScalar(prhs[2]);
    break;
  case 2: {
    double  *t = mxGetPr(prhs[2]);

    wx = t[0];
    wy = t[1];
    break;
  }
  default:
    mexErrMsgTxt("Window size must be 1- or 2-vector");
    return;
  }

  /* get disparity range */
  switch (mxGetNumberOfElements(prhs[3])) {
  case 1:
    dispmin = 0;
    dispmax = (int) mxGetScalar(prhs[3]);
    break;
  case 2: {
    double  *d = mxGetPr(prhs[3]);

	dispmin = (int)d[0];
	dispmax = (int)d[1];
    break;
  }
  default:
    mexErrMsgTxt("Disparities must be 1- or 2-vector");
    return;
  }

#ifdef  TIMING
  gettimeofday (&t1, NULL);
#endif

  /* allocate float images to hold copies of the images */
  leftI = (float*) mxCalloc (width * height, sizeof(float));
  rightI = (float*) mxCalloc (width * height, sizeof(float));

  leftI_ptr = leftI;
  rightI_ptr = rightI;

  if (mxGetClassID(prhs[0]) != mxGetClassID(prhs[1])) 
    mexErrMsgTxt("Images must be of the same class");

  switch (mxGetClassID(prhs[0])) {
  case mxDOUBLE_CLASS: {
      double   *l, *r;

      l = (double *) mxGetData(prhs[0]);
      r = (double *) mxGetData(prhs[1]);

      for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
          
          *(leftI_ptr++) = (float) REF_ML(l,j,i);
          *(rightI_ptr++) = (float) REF_ML(r,j,i);
        }
      }
    }
    break;

  case mxUINT8_CLASS: {
      PIX   *l, *r;

      l = (PIX *) mxGetData(prhs[0]);
      r = (PIX *) mxGetData(prhs[1]);

      for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
          
          *(leftI_ptr++) = (float) REF_ML(l,j,i);
          *(rightI_ptr++) = (float) REF_ML(r,j,i);
        }
      }
    }
    break;

  default:
    mexErrMsgTxt("Images must be double or uint8 class");
  }

  
#ifdef  TIMING
  gettimeofday (&t2, NULL);
#endif

  /* Create an NxMxD matrix for the return arguments */
  dims[0] = height_l;
  dims[1] = width_l;
  dims[2] = (int) (dispmax - dispmin);
  if (dims[2] < 0)
    dims[2] = -dims[2];
  dims[2] += 1;


  /* create 3D array to hold similarity scores */
  plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
  scoresD = (double *) mxGetData(plhs[0]);

  /* set all values to NaN now */
  for (i=0; i<dims[0]*dims[1]*dims[2]; i++)
    scoresD[i] = Z_NAN;

#ifdef  notdef
  /* set scores to NaN around the edges */
  wx2 = (wx - 1) / 2;
  wy2 = (wy - 1) / 2;
  for (y=0; y<wy2; y++)
      for (x=0; x<width; x++)
          for (d=0; d<dims[2]; d++)
              REF3(scoresD, x,y,d) = Z_NAN;
  for (y=(height-wy2); y<height; y++)
      for (x=0; x<width; x++)
          for (d=0; d<dims[2]; d++)
              REF3(scoresD, x,y,d) = Z_NAN;
  for (x=0; x<wx2; x++)
      for (y=0; y<height; y++)
          for (d=0; d<dims[2]; d++)
              REF3(scoresD, x,y,d) = Z_NAN;
  for (x=(width-wx2); x<width; x++)
      for (y=0; y<height; y++)
          for (d=0; d<dims[2]; d++)
              REF3(scoresD, x,y,d) = Z_NAN;
#endif
  //mexPrintf("Image size: %d x %d\n", height, width); 
  //mexPrintf("Window size: %d x %d, disparity: %d - %d\n", wx, wy, dispmin, dispmax);

#ifdef  TIMING
  gettimeofday (&t3, NULL);
#endif

  if (strcmp(metric, "zncc") == 0)
      stereo_matching_zncc(leftI, rightI, scoresD, width, height, wx, wy, dispmin, dispmax);
  else if (strcmp(metric, "ncc") == 0)
      stereo_matching_ncc(leftI, rightI, scoresD, width, height, wx, wy, dispmin, dispmax);
  else if (strcmp(metric, "ssd") == 0)
      stereo_matching_ssd(leftI, rightI, scoresD, width, height, wx, wy, dispmin, dispmax);
  else if (strcmp(metric, "sad") == 0)
      stereo_matching_sad(leftI, rightI, scoresD, width, height, wx, wy, dispmin, dispmax);
  else
    mexErrMsgTxt("Unknown metric");

#ifdef  TIMING
  gettimeofday (&t4, NULL);

  mexPrintf("arg checking     %.1fms\n", time_diff(&t1, &t0)*1000);
  mexPrintf("image conversion %.1fms\n", time_diff(&t2, &t1)*1000);
  mexPrintf("score array      %.1fms\n", time_diff(&t3, &t2)*1000);
  mexPrintf("stereo           %.1fms\n", time_diff(&t4, &t3)*1000);
#endif

  /* free the temporary float images */
  mxFree(leftI);
  mxFree(rightI);
}

#ifdef  TIMING
double
time_diff(struct timeval *tv1, struct timeval *tv2)
{
    double  dt;
    int     du;

    dt = tv1->tv_sec - tv2->tv_sec;
    du = tv1->tv_usec - tv2->tv_usec;

    if (du < 0) {
        dt--;
        du += 1000000;
    }

    dt += du / 1e6;

    return dt;
}
#endif
