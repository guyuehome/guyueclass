/* imatch.c
 *
 * Fast window matching
 *
 *	[XM,SCORE] = IMATCH(IM1, IM2, X, Y, W2, SEARCH)
 *
 * where	IM1 is the original image
 *		IM2 is the next image
 *		X,Y are the coordinate of the center of the region to track
 *		W2 is the half width of the match window
 *		SEARCH is the search bounds [xmin xmax ymin ymax] or if a scalar it is
 *				[-s s -s s]
 *
 *		XM is [DX, DY, CC] where X and Y are offsets relative to X, Y and CC is the match
 *				score
 *
 * Copyright (C) 1995-2009, by Peter I. Corke
 *
 * This file is part of The Machine Vision Toolbox for Matlab (MVTB).
 * 
 * MVTB is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * MVTB is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Leser General Public License
 * along with MVTB.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include "mex.h"
#include <math.h>

/* Input Arguments */

#define	IM1_IN		prhs[0]
#define	IM2_IN		prhs[1]
#define	X_IN		prhs[2]
#define	Y_IN		prhs[3]
#define	W2_IN		prhs[4]
#define	SEARCH_IN	prhs[5]

/* Output Arguments */
#define	XM_OUT		plhs[0]
#define	SCORE_OUT	plhs[1]


mxArray * imatch_double(double *xm, const mxArray *m_im1, const mxArray *m_im2, const mxArray *m_x, const mxArray *m_y, const mxArray *m_w2, const mxArray *m_search);
mxArray * imatch_uint8(double *xm, const mxArray *m_im1, const mxArray *m_im2, const mxArray *m_x, const mxArray *m_y, const mxArray *m_w2, const mxArray *m_search);

void
mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	mxArray	*r, *m_xm;

	/* Check for proper number of arguments */

	if (nrhs < 5)
		mexErrMsgTxt("IMATCH requires five input arguments.");

	m_xm = mxCreateDoubleMatrix(1, 3, mxREAL);

	/* Do the actual computations in a subroutine */

	if (mxGetClassID(IM1_IN) != mxGetClassID(IM2_IN))
		mexErrMsgTxt("IMATCH, images not of same class");

	switch (mxGetClassID(IM1_IN)) {
	case mxDOUBLE_CLASS:
		r = imatch_double(mxGetPr(m_xm), IM1_IN, IM2_IN, 
			X_IN, Y_IN, W2_IN, SEARCH_IN);
		break;
	case mxUINT8_CLASS:
		r = imatch_uint8(mxGetPr(m_xm), IM1_IN, IM2_IN, 
			X_IN, Y_IN, W2_IN, SEARCH_IN);
		break;
	default:
		mexErrMsgTxt("IMATCH, images of unsupported class");
	}
	if (r == NULL) {
		XM_OUT = mxCreateDoubleMatrix(0, 0, mxREAL);
		SCORE_OUT = mxCreateDoubleMatrix(0, 0, mxREAL);
		return;
	}

	if (nlhs > 0)
		XM_OUT = m_xm;
	if (nlhs == 2)
		SCORE_OUT = r;

	return;
}

#define	INDEX(p,x,y,h)	(p)[(y)+(x)*(h)]

/*	[XM,SCORE] = IMATCH(IM1, IM2, X, Y, W2, SEARCH)  */

mxArray *
imatch_double(double *xm,
	const mxArray *m_im1, const mxArray *m_im2,
	const mxArray *m_x, const mxArray *m_y, const mxArray *m_w2, const mxArray *m_search)
{
	mxArray	*m_score;
	const double	*im1, *im2;
	const double	*p, *p0, *p00, *q, *q0, *q00;
	double	*search;
	double	*score;
	double	pbar, qbar;
	double	sump, sump2, sumq, sumq2;
	double	c;
	double	sum, den;
	double	w2_sq;
	double	score_max;
	int	score_max_x, score_max_y;
	int	nrows_1, ncols_1, nrows_2, ncols_2;
	int	xmin, xmax, ymin, ymax;
	int	width, w2p1, x1, y1;
	int	dx, dy, x2, y2;
	int	height;
	int	w, h;

	/* unpack some of the parameters */
	search = mxGetPr(m_search);

	switch (mxGetN(m_search)) {
	case 1:
		xmin = -search[0];
		xmax = search[0];
		ymin = -search[0];
		ymax = search[0];
		break;
	case 4:
		xmin = search[0];
		xmax = search[1];
		ymin = search[2];
		ymax = search[3];
		break;
	default:
		mexErrMsgTxt("Search parameter must have 4 elements");
		break;
	}
	width = mxGetScalar(m_w2);
	w2p1 = 2*width + 1;
	w2_sq = w2p1 * w2p1;

	x1 = mxGetScalar(m_x) - 1;
	y1 = mxGetScalar(m_y) - 1;

	nrows_1 = mxGetM(m_im1);
	ncols_1 = mxGetN(m_im1);
	nrows_2 = mxGetM(m_im2);
	ncols_2 = mxGetN(m_im2);

	if ((nrows_1 != nrows_2) || (ncols_1 != ncols_2))
	    mexErrMsgTxt("Images dont conform");

	im1 = mxGetPr(m_im1);
	im2 = mxGetPr(m_im2);

	if ( 	((x1-width+xmin) < 0) ||
		((x1+width+xmax) >= ncols_1) ||
		((y1-width+ymin) < 0) ||
		((y1+width+ymax) >= nrows_1)
	)
	    return NULL;
		

	height = nrows_1;
	/*
	fprintf(stderr, "w=%d, (%d,%d) search %d %d %d %d, h=%d\n", w, x1, y1, xmin, xmax, ymin, ymax, height);
	*/

	m_score = mxCreateDoubleMatrix(ymax-ymin+1, xmax-xmin+1, mxREAL);
	score = mxGetPr(m_score);

	/*
	 * for all pixels in the window gather first and second
	 * moments.  From this compute mean and standard deviation
	 */
	sump = sump2 = 0.0;
	for (p00=p0=&INDEX(im1,(x1-width),(y1-width),height),w=w2p1; w-->0; p0+=height) {
		for (p=p0,h=w2p1; h-->0; ) {
			double	t = *p++;

			sump += t;
			sump2 += t*t;
		}
	}
	pbar = sump / w2_sq;

	score_max = -10.0;
	for (dx=xmin; dx<=xmax; dx++)
	    for (dy=ymin; dy<=ymax; dy++) {
		/*
		 * compute mean and standard deviation for this window
		 */
		x2 = x1 + dx;
		y2 = y1 + dy;

		sumq = sumq2 = 0.0;
		for (q00=q0=&INDEX(im2,(x2-width),(y2-width),height),w=w2p1; w-->0; q0+=height) {
			for (q=q0,h=w2p1; h-->0; ) {
				double	t = *q++;

				sumq += t;
				sumq2 += t*t;
			}
		}
		qbar = sumq / w2_sq;

		/*
		 * now compute the correlation (numerator) with mean offset
		 */
		sum = 0;
		for (p0=p00,q0=q00,w=w2p1; w-->0; p0+=height,q0+=height) {
			for (p=p0,q=q0,h=w2p1; h-->0; )
				sum += (*p++ - pbar) * (*q++ - qbar);
		}

		/*
		 * denominator is product of standard deviations
		 */
		den = (sump2 - ((double)sump*sump)/w2_sq) *
			(sumq2 - ((double)sumq*sumq)/w2_sq);

		if (den != 0)
			c = sum / sqrt(den);
		else
			c = -1.0;

	    	if (c > score_max) {
			score_max = c;
			score_max_x = dx;
			score_max_y = dy;
		}
		INDEX(score,dx-xmin,dy-ymin,(ymax-ymin+1)) = c;
	    }

	xm[0] = score_max_x;
	xm[1] = score_max_y;
	xm[2] = score_max;

	return m_score;
}

/*	[XM,SCORE] = IMATCH(IM1, IM2, X, Y, W2, SEARCH)  */

mxArray *
imatch_uint8(double *xm,
	const mxArray *m_im1, const mxArray *m_im2,
	const mxArray *m_x, const mxArray *m_y, const mxArray *m_w2, const mxArray *m_search)
{
	mxArray	*m_score;
	const unsigned char	*im1, *im2;
	const unsigned char	*p, *p0, *p00, *q, *q0, *q00;
	double	*search;
	double	*score;
	double	pbar, qbar;
	int	sump, sump2, sumq, sumq2;
	double	c;
	double	sum, den;
	double	w2_sq;
	double	score_max;
	int	score_max_x, score_max_y;
	int	nrows_1, ncols_1, nrows_2, ncols_2;
	int	xmin, xmax, ymin, ymax;
	int	width, w2p1, x1, y1;
	int	dx, dy, x2, y2;
	int	height;
	int	w, h;

	/* unpack some of the parameters */
	search = mxGetPr(m_search);

	switch (mxGetN(m_search)) {
	case 1:
		xmin = -search[0];
		xmax = search[0];
		ymin = -search[0];
		ymax = search[0];
		break;
	case 4:
		xmin = search[0];
		xmax = search[1];
		ymin = search[2];
		ymax = search[3];
		break;
	default:
		mexErrMsgTxt("Search parameter must have 4 elements");
		break;
	}
	width = mxGetScalar(m_w2);
	w2p1 = 2*width + 1;
	w2_sq = w2p1 * w2p1;

	x1 = mxGetScalar(m_x) - 1;
	y1 = mxGetScalar(m_y) - 1;

	nrows_1 = mxGetM(m_im1);
	ncols_1 = mxGetN(m_im1);
	nrows_2 = mxGetM(m_im2);
	ncols_2 = mxGetN(m_im2);

	if ((nrows_1 != nrows_2) || (ncols_1 != ncols_2))
	    mexErrMsgTxt("Images dont conform");

	im1 = (unsigned char *)mxGetData(m_im1);
	im2 = (unsigned char *)mxGetData(m_im2);

	if ( 	((x1-width+xmin) < 0) ||
		((x1+width+xmax) >= ncols_1) ||
		((y1-width+ymin) < 0) ||
		((y1+width+ymax) >= nrows_1)
	)
	    return NULL;
		

	height = nrows_1;
	/*
	fprintf(stderr, "w=%d, (%d,%d) search %d %d %d %d, h=%d\n", w, x1, y1, xmin, xmax, ymin, ymax, height);
	*/

	m_score = mxCreateDoubleMatrix(ymax-ymin+1, xmax-xmin+1, mxREAL);
	score = mxGetPr(m_score);

	/*
	 * for all pixels in the window gather first and second
	 * moments.  From this compute mean and standard deviation
	 */
	sump = sump2 = 0.0;
	for (p00=p0=&INDEX(im1,(x1-width),(y1-width),height),w=w2p1; w-->0; p0+=height) {
		for (p=p0,h=w2p1; h-->0; ) {
			double	t = *p++;

			sump += t;
			sump2 += t*t;
		}
	}
	pbar = (double)sump / w2_sq;

	score_max = -10.0;
	for (dx=xmin; dx<=xmax; dx++)
	    for (dy=ymin; dy<=ymax; dy++) {
		/*
		 * compute mean and standard deviation for this window
		 */
		x2 = x1 + dx;
		y2 = y1 + dy;

		sumq = sumq2 = 0.0;
		for (q00=q0=&INDEX(im2,(x2-width),(y2-width),height),w=w2p1; w-->0; q0+=height) {
			for (q=q0,h=w2p1; h-->0; ) {
				double	t = *q++;

				sumq += t;
				sumq2 += t*t;
			}
		}
		qbar = (double)sumq / w2_sq;

		/*
		 * now compute the correlation (numerator) with mean offset
		 */
		sum = 0;
		for (p0=p00,q0=q00,w=w2p1; w-->0; p0+=height,q0+=height) {
			for (p=p0,q=q0,h=w2p1; h-->0; )
				sum += (*p++ - pbar) * (*q++ - qbar);
		}

		/*
		 * denominator is product of standard deviations
		 */
		den = (sump2 - ((double)sump*sump)/w2_sq) *
			(sumq2 - ((double)sumq*sumq)/w2_sq);

		if (den != 0)
			c = sum / sqrt(den);
		else
			c = -1.0;

	    	if (c > score_max) {
			score_max = c;
			score_max_x = dx;
			score_max_y = dy;
		}
		INDEX(score,dx-xmin,dy-ymin,(ymax-ymin+1)) = c;
	    }

	xm[0] = score_max_x;
	xm[1] = score_max_y;
	xm[2] = score_max;

	return m_score;
}
