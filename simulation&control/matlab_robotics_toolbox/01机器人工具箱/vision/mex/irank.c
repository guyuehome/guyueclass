/*
 * irank.c
 * 
 * Fast rank filter for images.
 * 
 * IMM = IRANK(IM, ORDER, SE [,HISTOBINS] [, EDGE])
 *              0    1     2      3          4
 * 
 * where	SE is the structuring element ORDER is EDGE is 'border', 'none',
 * 'trim', 'wrap', 'zero'.
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
 * Uses code from the package VISTA Copyright 1993, 1994 University of 
 * British Columbia.
 */
#include <math.h>
#include <string.h>
#include "mex.h"
#include "edge.h"

#ifdef  __LCC__
#define  NAN    mxGetNaN()
#endif

#ifdef _W64
#include <float.h>
#define  isnan  _isnan
#define  NAN    mxGetNaN()
#endif 

/* Input Arguments */

#define	IM_IN		prhs[0]
#define	ORDER_IN	prhs[1]
#define	SE_IN		prhs[2]
#define	HISTBINS_IN	prhs[3]
#define	EDGE_IN		prhs[4]

#define	HISTBINS	256

/* Output Arguments */

#define	IMM_OUT	plhs[0]

enum op_type {
	OpMax,
	OpMin,
	OpDiff
}               oper;

#define	BUFLEN	100

mxArray        *irank(const mxArray * msrc, const mxArray * mmask, const mxArray *morder, int hbins);

void
mexFunction(int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[])
{
	char            s[BUFLEN];
	mxArray        *r;
	mxArray	*im;
    int     width, height;
	int		histbins = HISTBINS;

    pad_method = PadBorder;

/*
 * 
 * IMM = IRANK(IM, ORDER, SE [,HISTOBINS] [, EDGE])
 *              0    1     2      3          4
 */

	/* parse out the edge method */
	switch (nrhs) {
	case 5:
		if (!mxIsChar(EDGE_IN))
			mexErrMsgTxt("edge arg must be string");
		mxGetString(EDGE_IN, s, BUFLEN);
 		/* EDGE handling flags */
		if (strcmp(s, "replicate") == 0)
			pad_method = PadBorder;
		else if (strcmp(s, "none") == 0)
			pad_method = PadNone;
		else if (strcmp(s, "wrap") == 0)
			pad_method = PadWrap;
		else if (strcmp(s, "valid") == 0)
			pad_method = PadTrim;
        else
            mexErrMsgTxt("IRANK bad edge option");
		/* fall through */
	case 4:
		if (!mxIsNumeric(HISTBINS_IN))
			mexErrMsgTxt("histbins arg must be numeric");
		histbins = (int) * mxGetPr(HISTBINS_IN);
		/* fall through */
    case 3:
        break;
    default:
		mexErrMsgTxt("IRANK requires three input arguments.");
	}

	if ((!mxIsNumeric(ORDER_IN)) || (!mxIsNumeric(SE_IN)))
			mexErrMsgTxt("first 3 args must be numeric");
		
    if (mxIsComplex(IM_IN))
		mexErrMsgTxt("IRANK requires a real matrix.");

    if (mxGetNumberOfDimensions(IM_IN) > 2)
        mexErrMsgTxt("Only greylevel images allowed.");

    height = mxGetM(IM_IN);
    width = mxGetN(IM_IN);

    switch (mxGetClassID(IM_IN)) {
        case mxLOGICAL_CLASS: {
            double *p;
            mxLogical *q = (mxLogical *)mxGetPr(IM_IN);
            int     i;

            im = mxCreateDoubleMatrix(height, width, mxREAL);
            p = mxGetPr(im);

            for (i=0; i<width*height; i++)
                    *p++ = *q++;
            break;
        }
        case mxUINT8_CLASS: {
            double *p;
            unsigned char  *q = (unsigned char *)mxGetPr(IM_IN);
            int     i;

            im = mxCreateDoubleMatrix(height, width, mxREAL);
            p = mxGetPr(im);

            for (i=0; i<width*height; i++)
                    *p++ = *q++;
            break;
        }
        case mxUINT16_CLASS: {
            double *p;
            unsigned short  *q = (unsigned short *)mxGetPr(IM_IN);
            int     i;

            im = mxCreateDoubleMatrix(height, width, mxREAL);
            p = mxGetPr(im);
            
            for (i=0; i<width*height; i++)
                    *p++ = *q++;
            break;
        }
        case mxDOUBLE_CLASS: {
            im = (mxArray *)IM_IN;
            break;
        }
        default:
            mexErrMsgTxt("Only logical, uint8, uint16 or double images allowed");
    }

	/* Do the actual computations in a subroutine */
	r = irank(im, SE_IN, ORDER_IN, histbins);
	if (nlhs == 1)
		plhs[0] = r;

    /* free tempory storage if we created it */
    if (im != IM_IN)
        mxDestroyArray(im);
    
	return;
}


#define	SPixel(r, c)	src[r+c*src_nrows]
#define	DPixel(r, c)	dest[r+c*dest_nrows]
#define	MPixel(r, c)	mask[r+c*mask_nrows]

mxArray        *
irank(const mxArray * msrc, const mxArray * mmask, const mxArray * morder, int histbins)
{
	int             dest_nrows, dest_ncols, mask_nrows, mask_ncols;
	int             src_nrows, src_ncols;
	mxArray        *mdest;
	double         *src, *dest, *mask, min, max, offset, scale;
	int             row_offset, col_offset;
	int             src_row, src_col, dest_row,
	                mask_row, mask_col, dest_col, i, j, k, iorder;
	unsigned int	*hist, *ph, cumsum, count, nbinned;

	src_nrows = mxGetM(msrc);
	src_ncols = mxGetN(msrc);
	mask_nrows = mxGetM(mmask);
	mask_ncols = mxGetN(mmask);

	/* process input variables */
	src = mxGetPr(msrc);
	mask = mxGetPr(mmask);
	iorder = (int) mxGetScalar(morder);

    if (mxGetNumberOfElements(mmask) == 1) {
        // mask is a scalar, make it an NxN matrix of ones
        int h = mxGetScalar(mmask);
        int i;
        int w;

        w = 2*h+1;      // half-width was passed in
        mask = mxCalloc(w*w, sizeof(double));
        for (i=0; i<(w*w); i++)
            mask[i] = 1;
        mask_nrows = mask_ncols = w;
    }

	/*
	 * find span of pixel values in the image
	 */
	max = -DBL_MAX;
	min = DBL_MAX;
	for (src_col = 0; src_col < src_ncols; src_col++)
		for (src_row = 0; src_row < src_nrows; src_row++) {
			double          p = SPixel(src_row, src_col);

			if (p > max)
				max = p;
			if (p < min)
				min = p;
		}
	printf("image pixel values: %f to %f\n", min, max);
	offset = min;
	if ( (max - min) == 0)
			mexErrMsgTxt("Image has no variance");
	scale = (histbins-1) / (max - min);
    printf(" %d bins, scale=%f, off=%f\n", histbins, scale, offset);

	/*
	 * determine number of non-zero mask elements
	 */
	count = 0;
	for (mask_col = 0; mask_col < mask_ncols; mask_col++)
		for (mask_row = 0; mask_row < mask_nrows; mask_row++) {
			double          p = MPixel(mask_row, mask_col);

			if (p > 0)
				count++;
		}
	printf("%d non-zero mask elements\n", count);
	if ( (iorder < 1) || (iorder > count) )
			mexErrMsgTxt("Order must be between 1 and number of elements in mask");
	iorder = count - iorder;

	/*
	 * Determine what dimensions the destination image should have: o if
	 * the pad method is Trim, the destination image will have smaller
	 * dimensions than the source image (by an amount corresponding to the
	 * mask size); otherwise it will have the same dimensions.
	 */
	dest_nrows = src_nrows;
	dest_ncols = src_ncols;
	if (pad_method == PadTrim) {
		dest_nrows -= (mask_nrows - 1);
		dest_ncols -= (mask_ncols - 1);
		if (dest_nrows <= 0 || dest_ncols <= 0)
			mexErrMsgTxt("Image is smaller than mask");
	}

	mdest = mxCreateDoubleMatrix(dest_nrows, dest_ncols, mxREAL);
	dest = mxGetPr(mdest);

	if ((hist = mxCalloc(histbins, sizeof(unsigned int))) == NULL)
		mexErrMsgTxt("irank: calloc() failed");

	/*
	 * Determine the mapping from destination coordinates + mask
	 * coordinates to source coordinates:
	 */
	if (pad_method == PadTrim)
		row_offset = col_offset = 0;
	else {
		row_offset = -(mask_nrows / 2);
		col_offset = -(mask_ncols / 2);
	}

    // index over the input image
	for (dest_row = 0; dest_row < dest_nrows; dest_row++)
		for (dest_col = 0; dest_col < dest_ncols; dest_col++) {
			/* zero the histogram */
			for (i=0, ph=hist; i<histbins; i++)
				*ph++ = 0;
            nbinned = 0;

            // index over the mask matrix
			for (j = 0; j < mask_nrows; j++) {
				src_row = dest_row + j + row_offset;
				ClampIndex(src_row, src_nrows, END);
				for (k = 0; k < mask_ncols; k++) {
					double	p;
					src_col = dest_col + k + col_offset;
					ClampIndex(src_col, src_ncols, END);
					if (MPixel(j, k) > 0) {
						int	t;

                        // get the pixel
						p = SPixel(src_row, src_col);
                        if (!isnan(p)) {
                            // update the histogram of values
                            t = (int) ((p-offset)*scale);
                            hist[t]++;
                            nbinned++;
                        }
					}
				}
			}
	END:		;
            // compute cumulative sum of histogram, break when the
            // sum exceeds the specified rank
            if (nbinned > iorder) {
                for (ph=hist, cumsum=0; cumsum<iorder; ph++)
                    cumsum += *ph;
                /* convert bin index to value */
                DPixel(dest_row, dest_col) = (double)(ph - hist) / scale + offset;
            } else {
                DPixel(dest_row, dest_col) = NAN;
            }
		}

    if (mxGetNumberOfElements(mmask) == 1) {
        // mask is a scalar, free the allocated storage
        mxFree(mask);
    }

	return mdest;
}
