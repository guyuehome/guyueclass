/* imorph.c
 *
 * Fast morphological transform for images.
 *
 *  IMM = IMORPH(IM, SE, OP [, EDGE])
 *
 * where    SE is the structuring element
 *      OP is 'min', 'max', 'diff', 'plusmin'
 *      EDGE is 'border', 'none', 'trim', 'wrap', 'zero'.
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

#define IM_IN       prhs[0]
#define SE_IN       prhs[1]
#define OP_IN       prhs[2]
#define EDGE_IN     prhs[3]

/* Output Arguments */

#define IMM_OUT plhs[0]

enum op_type {
    OpMax,
    OpMin,
    OpDiff,
    OpPlusMin,
} oper;

#define BUFLEN  100

mxArray *imorph(const mxArray *msrc, const mxArray *mmask);

void
mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char    s[BUFLEN];
    mxArray *r;
    mxArray *im;
    int     width, height;

    pad_method = PadBorder;

    /* Check for proper number of arguments */

    /* parse out the edge method */
    switch (nrhs) {
    case 4:
        if (!mxIsChar(prhs[3]))
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
            mexErrMsgTxt("IMORPH bad edge option");
        /* fall through */
    case 3:
        if (!mxIsChar(OP_IN))
            mexErrMsgTxt("op arg must be string");
        mxGetString(OP_IN, s, BUFLEN);
        /* OP is 'min', 'max', 'diff' */
        if (strcmp(s, "min") == 0)
            oper = OpMin;
        else if (strcmp(s, "max") == 0)
            oper = OpMax;
        else if (strcmp(s, "diff") == 0)
            oper = OpDiff;
        else if (strcmp(s, "plusmin") == 0)
            oper = OpPlusMin;
        break;
    default:
        mexErrMsgTxt("IMORPH requires three input arguments.");
    }
            
    if (mxIsComplex(IM_IN))
        mexErrMsgTxt("IMORPH requires a real matrix.");

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
        case mxSINGLE_CLASS: {
            double *p;
            float  *q = (float *)mxGetPr(IM_IN);
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

    r = imorph(im, SE_IN);
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
 *  This macro performs the actual morphological operaton, iterating over pixels
 *  of the destination and mask images.
 */

#define Morph(op, label)            \
{                                       \
    double  min, max;                               \
    double  p, a;               \
    for (dest_row = 0; dest_row < dest_nrows; dest_row++)     \
        for (dest_col = 0; dest_col < dest_ncols; dest_col++) { \
            max = -mxGetInf();                              \
            min = mxGetInf();                               \
            for (j = 0; j < mask_nrows; j++) {          \
                src_row = dest_row + j + row_offset;        \
                ClampIndex (src_row, src_nrows, label);     \
                for (k = 0; k < mask_ncols; k++) {      \
                    src_col = dest_col + k + col_offset;    \
                    ClampIndex (src_col, src_ncols, label); \
                    if (MPixel(j,k) > 0) {                 \
                        p = SPixel( src_row, src_col);      \
                        if (p > max)                        \
                            max = p;            \
                        if (p < min)                        \
                            min = p;            \
                    }                                       \
                }                       \
            }                           \
  label:    DPixel(dest_row, dest_col) = op;                    \
        }                               \
}

#define MorphPlusMin(label)         \
{                                       \
    double  min;                                \
    double  p, a;               \
    for (dest_row = 0; dest_row < dest_nrows; dest_row++)     \
        for (dest_col = 0; dest_col < dest_ncols; dest_col++) { \
        if ( mxIsNaN(SPixel( dest_row, dest_col)) ) {     \
            DPixel(dest_row, dest_col) = mxGetNaN();         \
            continue;     \
        }                 \
        min = mxGetInf();                               \
            for (j = 0; j < mask_nrows; j++) {          \
            src_row = dest_row + j + row_offset;        \
            ClampIndex (src_row, src_nrows, label);     \
            for (k = 0; k < mask_ncols; k++) {      \
                src_col = dest_col + k + col_offset;    \
                ClampIndex (src_col, src_ncols, label); \
                p = SPixel( src_row, src_col) + MPixel(j,k); \
                if (p < min)                        \
                    min = p;            \
            }                       \
            }                           \
  label:    DPixel(dest_row, dest_col) = min;                    \
        }                               \
}

#define SPixel(r, c)    src[r+c*src_nrows]
#define DPixel(r, c)    dest[r+c*dest_nrows]
#define MPixel(r, c)    mask[r+c*mask_nrows]

mxArray *
imorph(const mxArray *msrc, const mxArray *mmask)
{
    int dest_nrows, dest_ncols, mask_nrows,mask_ncols;
    int src_nrows, src_ncols;
    mxArray *mdest;
    double  *src, *dest, *mask;
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

    /* Perform the convolution over all destination rows, columns: */
    switch (oper) {
    case OpMin:
        Morph(min, L1); break;
    case OpMax:
        Morph(max, L2); break;
    case OpDiff:
        Morph(max-min, L3); break;
    case OpPlusMin:
        MorphPlusMin(L4); break;
    }

    return mdest;
}
