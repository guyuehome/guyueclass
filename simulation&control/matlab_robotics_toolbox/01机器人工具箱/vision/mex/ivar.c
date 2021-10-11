/* ivar.c
 *
 * Fast region variance/kurtosis/skewness for images
 *
 *	IMM = IVAR(IM, SE, OP [, EDGE])
 *
 * where	SE is the structuring element
 *		OP is 'var', 'kurt', 'skew'
 *		EDGE is 'border', 'none', 'trim', 'wrap', 'zero'.
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

/* Input Arguments */

#define	IM_IN		prhs[0]
#define	SE_IN		prhs[1]
#define	OP_IN		prhs[2]
#define	EDGE_IN		prhs[3]

/* Output Arguments */

#define	IMM_OUT	plhs[0]


enum op_type {
	OpVar,
	OpSkew,
	OpKurt
} oper;

#define	BUFLEN	100

mxArray *ivar(const mxArray *msrc, const mxArray *mmask);

void
mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	char	s[BUFLEN];
	mxArray	*r;
	mxArray	*im;
    int width, height;

    pad_method = PadBorder;

	/* Check for proper number of arguments */


	/* parse out the edge method */
	switch (nrhs) {
	case 4:
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
            mexErrMsgTxt("IVAR bad edge option");
		/* fall through */
	case 3:
		if (!mxIsChar(OP_IN))
			mexErrMsgTxt("op arg must be string");
		mxGetString(OP_IN, s, BUFLEN);
		/* OP is 'variance', 'skewness', 'kurtosis' */
		if (strncmp(s, "var", 3) == 0)
			oper = OpVar;
		else if (strncmp(s, "ske", 3) == 0)
			oper = OpSkew;
		else if (strncmp(s, "kur", 3) == 0)
			oper = OpKurt;
        break;
    default:
		mexErrMsgTxt("IVAR requires at least three input arguments.");
	}
			
    if (mxIsComplex(IM_IN))
		mexErrMsgTxt("IVAR requires a real matrix.");

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

	r = ivar(im, SE_IN);
	if (nlhs == 1)
		plhs[0] = r;

    /* free tempory storage if we created it */
    if (im != IM_IN)
        mxDestroyArray(im);
    
	return;
}


/*
 *  Morph
 *
 *  This macro performs the actual varological operaton, iterating over pixels
 *  of the destination and mask images.
 */


#define	SPixel(r, c)	src[r+c*src_nrows]
#define	DPixel(r, c)	dest[r+c*dest_nrows]
#define	MPixel(r, c)	mask[r+c*mask_nrows]

mxArray *
ivar(const mxArray *msrc, const mxArray *mmask)
{
    int dest_nrows, dest_ncols, mask_nrows,mask_ncols;
    int	src_nrows, src_ncols, N;
    mxArray	*mdest;
    double	*src, *dest, *mask;
    int band_offset, row_offset, col_offset;
    int src_band, src_row, src_col, dest_band, dest_row, dest_col, i, j, k;

    src_nrows = mxGetM(msrc);
    src_ncols = mxGetN(msrc);
    mask_nrows = mxGetM(mmask);
    mask_ncols = mxGetN(mmask);

    /* Determine what dimensions the destination image should have:
	o if the pad method is Trim, the destination image will have smaller
	  dimensions than the source image (by an amount corresponding to the
	  mask size); otherwise it will have the same dimensions. */
    dest_nrows = src_nrows;
    dest_ncols = src_ncols;
    if (pad_method == PadTrim) {
	dest_nrows -= (mask_nrows - 1);
	dest_ncols -= (mask_ncols - 1);
	if (dest_nrows <= 0 || dest_ncols <= 0)
	    mexErrMsgTxt("Image is smaller than mask");
    }


    /* Locate the destination. Since the operation cannot be carried out in
       place, we create a temporary work image to serve as the destination if
       dest == src or dest == mask: */
    mdest = mxCreateDoubleMatrix(dest_nrows, dest_ncols, mxREAL);

    src = mxGetPr(msrc);
    dest = mxGetPr(mdest);
    mask = mxGetPr(mmask);

    /* Determine the mapping from destination coordinates + mask coordinates to
       source coordinates: */
    if (pad_method == PadTrim)
	row_offset = col_offset = 0;
    else {
	row_offset = - (mask_nrows / 2);
	col_offset = - (mask_ncols / 2);
    }

    /*
     * count the number of set pixels
     */
    N = 0;
    for (j = 0; j < mask_nrows; j++)
	for (k = 0; k < mask_ncols; k++)
	    if (MPixel(j,k) > 0)
		N++;
	//printf("%d pixels in mask\n", N);

    /* Perform the convolution over all destination rows, columns: */
    switch (oper) {
    case OpVar:
	{
	    double	s, s2, s3, s4, sq;
	    double	p, a;
		for (dest_row = 0; dest_row < dest_nrows; dest_row++)
		    for (dest_col = 0; dest_col < dest_ncols; dest_col++) {
			s = s2 = 0.0;
			    for (j = 0; j < mask_nrows; j++) {
				src_row = dest_row + j + row_offset;
				ClampIndex (src_row, src_nrows, L1);
				for (k = 0; k < mask_ncols; k++) {
				    src_col = dest_col + k + col_offset;
				    ClampIndex (src_col, src_ncols, L1);
				    if (MPixel(j,k) > 0) {
					p = SPixel( src_row, src_col);
					sq=p*p; s+=p; s2+=sq;
				    }
				}
			    }
	  L1:	DPixel(dest_row, dest_col) = (N*s2 - s*s) / (double)(N*(N-1));
		    }
	}
	break;
    case OpSkew:
	{
	    double	s, s2, s3, s4, sq;
	    double	p, a;
		for (dest_row = 0; dest_row < dest_nrows; dest_row++)
		    for (dest_col = 0; dest_col < dest_ncols; dest_col++) {
			s = s2 = s3 = 0.0;
			    for (j = 0; j < mask_nrows; j++) {
				src_row = dest_row + j + row_offset;
				ClampIndex (src_row, src_nrows, L2);
				for (k = 0; k < mask_ncols; k++) {
				    src_col = dest_col + k + col_offset;
				    ClampIndex (src_col, src_ncols, L2);
				    if (MPixel(j,k) > 0) {
					p = SPixel( src_row, src_col);
					sq=p*p; s+=p; s2+=sq; s3+=sq*p;
				    }
				}
			    }
	  L2:	DPixel(dest_row, dest_col) =  (N*N*s3 - 3*N*s2*s + 3*s*s*s - s*s*s )/(N*N*(N-1));
		    }
	}
	break;
    case OpKurt:
	{
	    double	s, s2, s3, s4, sq;
	    double	p, a;
		for (dest_row = 0; dest_row < dest_nrows; dest_row++)
		    for (dest_col = 0; dest_col < dest_ncols; dest_col++) {
			s = s2 = s3 = s4 = 0.0;
			    for (j = 0; j < mask_nrows; j++) {
				src_row = dest_row + j + row_offset;
				ClampIndex (src_row, src_nrows, L3);
				for (k = 0; k < mask_ncols; k++) {
				    src_col = dest_col + k + col_offset;
				    ClampIndex (src_col, src_ncols, L3);
				    if (MPixel(j,k) > 0) {
					p = SPixel( src_row, src_col);
					sq=p*p; s+=p; s2+=sq; s3+=sq*p; s4 += sq*sq;
				    }
				}
			    }
	  L3:	DPixel(dest_row, dest_col) =  (N*N*N*s4 - 4*N*N*s3*s + 6*N*s2*s*s - 3*s*s*s*s)/ (N*N*N*(N-1));
		    }
	}
	break;
	}

    return mdest;
}
