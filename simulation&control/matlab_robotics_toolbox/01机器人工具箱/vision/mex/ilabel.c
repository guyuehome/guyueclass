/*LABEL ilabel.c
 *
 * Fast image labelling funciton.
 *
 *  L = ILABEL(IM [,OPT])
 *  [L,LMAX] = ILABEL(IM)
 *  [L,LMAX,PARENTS] = ILABEL(IM)
 *  [L,LMAX,PARENTS,COLOR] = ILABEL(IM)
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

 /*
  * TODO:
  * eliminate lmap2
  * eliminate double copying of parent, color data
  * edge is not propagated properly
  */
#include "mex.h"
#include <math.h>

#ifdef __LCC__
#ifdef  WIN32
typedef unsigned int uint32_t;
#else
#error "LCC under WIN64 not supported"
#endif
#else
#include    <stdint.h>
#endif

/*
#define DEBUG   1
*/

/*
 * TODO:
 * relabeling is complex, dont understand it
 * make the threshold thing work
 * enclosures seem to happen too often
 */

#define THRESH  0

/* Input Arguments */

#define IM_IN       prhs[0]
#define CONNECT_IN  prhs[1]
#define MINSIZE_IN  prhs[2]

/* Output Arguments */

#define IM_OUT  plhs[0]
#define MAX_OUT plhs[1]
#define PARENT_OUT  plhs[2]
#define COLOR_OUT   plhs[3]
#define EDGE_OUT    plhs[4]

#define MAXLABEL    10000
#define UNKNOWN     0

#define TERMINAL    0x8000000

typedef unsigned int LABEL;
typedef int  PIXEL;

/*
#define PIX(v,r,c)      v[(r)*height+(c)]
#define PIX_N(v,r,c)    v[(r-1)*height+(c)]
#define PIX_NW(v,r,c)   v[(r-1)*height+(c-1)]
#define PIX_W(v,r,c)    v[(r)*height+(c-1)]
*/
#define PIX(v,r,c)      v[(r)+(c)*height]
#define PIX_N(v,r,c)    PIX(v,r-1,c)
#define PIX_NW(v,r,c)   PIX(v,r-1,c-1)
#define PIX_W(v,r,c)    PIX(v,r,c-1)

LABEL lresolve(LABEL label);

static LABEL    *lmap;

#define NEWLABEL    (++newlabel)

/**
 * Image labeling algorithm.
 *
 *  @param im  pointer to input image of type PIXEL
 *  @param width  width input image in pixels
 *  @param height  height input image in pixels
 *  @param connectivty  number of connectivity directions (4 or 8)
 *  @param minsize  smallest allowable blob size, they get subsumed into the parent blob
 *  @param limage pointer to label image, of same size as \p im (output)
 *  @param parent_out pointer to pointer for return of an array of region parents.  Can be NULL. (output)
 *  @param color_out pointer to pointer for return of an array of region color. Can be NULL. (output)
 *  @param edge_out pointer to pointer for return of an array of region edge points.  Can be NULL. (output)
 *  @return the number of regions found
 */
static int 
ilabel(PIXEL *im, int width, int height, int connectivity, int minsize,
    LABEL *limage,
    LABEL **parent_out, PIXEL **color_out, unsigned int **edge_out)
{
    int     *blobsize, row, col, i, j, nlabels;
    int newlabel;
    LABEL   *lmap2;
    LABEL   prevlab, curlab;
    LABEL   *parents;
    PIXEL   curpix, prevpix;
    PIXEL   *color;
    unsigned int    *edge;
    int     maxlabel = MAXLABEL;

    /* allocate label map and initialize to zero */
    lmap = (LABEL *)mxCalloc(maxlabel, sizeof(LABEL));
    lmap2 = (LABEL *)mxCalloc(maxlabel, sizeof(LABEL));

    if (parent_out)
        parents = (LABEL *)mxCalloc(maxlabel, sizeof(LABEL));
    if (color_out)
        color = (PIXEL *)mxCalloc(maxlabel, sizeof(PIXEL));
    if (edge_out)
        edge = (unsigned int *)mxCalloc(maxlabel, sizeof(int));

    /* region size */
    blobsize = (int *)mxCalloc(maxlabel, sizeof(int));

    /*
     * Blob labels are ints >= 1
     * newlabel holds the most recently assigned label value.
     * labels are unique and never recycled.
     *
     * When 2 blobs merge: A = A + B an entry is placed in the label map
     *     lmap[B] = A indicating that all pixels that were B are now A.
     * If lmap[X] = 0 then X is unmerged, X is X
     * It is possible that a later merge: C = C + A
     *     lmap[A] = C indicating that all As are now Cs
     * A pixel that was once B is mapped to A, then to C, but lmap[C] is 0 so
     * the indirection stops there.  B --> A --> C
     *
     * Resolving the indirection is the job of lresolve()
     *      lresolve(A) --> C
     *      lresolve(B) --> C
     *      lresolve(C) --> C
     */

    for (row=0; row<height; row++) {
        for (col=0; col<width; col++) {
            curpix = PIX(im,row,col);
#ifdef  DEBUG
            printf("%2d ", curpix);
#endif
        }
#ifdef  DEBUG
        printf("\n");
#endif
    }

    /*
     * first pass labelling loop.  Only does 4-way connectivity
     */
    newlabel = 0;
    for (row=0; row<height; row++) {
        prevlab = UNKNOWN;
        for (col=0; col<width; col++) {
            curpix = PIX(im,row,col);
            curlab = UNKNOWN;       // start with no known label
            if (col > 0) {
                prevpix = PIX(im,row,col-1);
                /* if no change in pixel value then inherit label from left */
                if (curpix == prevpix)
                    curlab = prevlab;
            }

            
#ifdef  DEBUG
                printf("(%d,%d) cp=%d, pp=%d, cl=%d, pl=%d\n", row, col, curpix, prevpix,
                    curlab, prevlab);
#endif
            /*
             * check whether a label merge should happen, adjacent
             * pixels with the same value but different labels
             * means that one should change.
             *
             * merge can only happen on second row onwards 
             */
            if (row > 0) {
                if (    (PIX(im,row-1,col) == curpix) &&
                    (lresolve(PIX(limage,row-1,col)) != curlab)
                ) {
                    /* we have a label assignment from N */
                    int newlabel;
                    
                    newlabel = lresolve(PIX(limage,row-1,col));
                    /*
                    newlabel = PIX(lim,row-1,col);
                    */

#ifdef  DEBUG
                    printf("mergeN(%d,%d): %d becomes %d: curpix=%d, prevpix=%d\n",
                        row, col, curlab, newlabel, curpix, PIX(im,row-1,col));
#endif
                    // newlabel dominates
                    if (curlab != UNKNOWN) {
                        lmap[curlab] = newlabel;
                        blobsize[newlabel] += blobsize[curlab];
                        //if (edge[curlab] < edge[newlabel])
                            //edge[newlabel] = edge[curlab];
                        if (parents[curlab] == 0)
                            parents[newlabel] = 0;
                    }
                    curlab = newlabel;

                } else if (
                    connectivity == 8 &&
                    (col > 0) &&
                    (PIX(im,row-1,col-1) == curpix) &&
                    (lresolve(PIX(limage,row-1,col-1)) != curlab)
                ) {
                    //  TODO: factorize these two merge cases

                    /* we have a merge to NW */
                    int newlabel;
                    
                    newlabel = lresolve(PIX(limage,row-1,col-1));
                    /*
                    newlabel = PIX(lim,row-1,col);
                    */

#ifdef  DEBUG
                    printf("mergeNW(%d,%d): %d becomes %d\n",
                        row, col, curlab, newlabel);
#endif
                    if (curlab != UNKNOWN)
                        lmap[curlab] = newlabel;
                    if (parents[curlab] == 0)
                        parents[newlabel] = 0;
                    //if (edge[curlab] < edge[newlabel])
                        //edge[newlabel] = edge[curlab];
                    blobsize[newlabel] += blobsize[curlab];
                    curlab = newlabel;

                } else if (
                    connectivity == 8 &&
                    (col < (width-1)) &&
                    (PIX(im,row-1,col+1) == curpix) &&
                    (lresolve(PIX(limage,row-1,col+1)) != curlab)
                ) {
                    /* we have a merge to NE */
                    int newlabel;
                    
                    newlabel = lresolve(PIX(limage,row-1,col+1));
                    /*
                    newlabel = PIX(lim,row-1,col);
                    */

#ifdef  DEBUG
                    printf("mergeNE(%d,%d): %d becomes %d\n",
                        row, col, curlab, newlabel);
#endif
                    if (curlab != UNKNOWN)
                        lmap[curlab] = newlabel;
                    blobsize[newlabel] += blobsize[curlab];
                    //if (edge[curlab] < edge[newlabel])
                        //edge[newlabel] = edge[curlab];
                    curlab = newlabel;

                }

            }

            if ((row > 0) && (col > 0)) {
                /*
                 * check for enclosure
                 */
                int left, above, northwest;

                left = prevlab;
                above = lresolve( PIX(limage,row-1,col) );
                northwest = lresolve( PIX(limage,row-1,col-1) );
                if (    (left == curlab) &&
                    (above == curlab) &&
                    (northwest != curlab)
                ) {
#ifdef  DEBUG
                    printf("(%d,%d): label %d encloses %d\n",
                        row, col,
						curlab, northwest);
#endif
					/* we have an enclosure */
                    if (blobsize[curlab] > THRESH) {
                        // mark the parent of this blob
                        parents[northwest] = curlab;
                        //edge[northwest] = (row-1) + height*(col-1) + 1;
                        //printf("edge is %d\n", edge[northwest]);
                    } else {
                        // it's a runt, merge it with its parent
                        lmap[curlab] = northwest;
                    }
                }
            }

            /* if label still not known, assign new */
            if (curlab == UNKNOWN) {
                curlab = NEWLABEL;
                if (newlabel >= maxlabel) {
                    /*
                     * too many labels, increase the size of all the working arrays 
                     */
                    maxlabel *= 2;
                    printf("realloc array, maxlabel now %d\n", maxlabel);
                    lmap = (LABEL *)mxRealloc(lmap, maxlabel*sizeof(LABEL));
                    lmap2 = (LABEL *)mxRealloc(lmap2, maxlabel*sizeof(LABEL));

                    if (parent_out)
                        parents = (LABEL *)mxRealloc(parents, maxlabel*sizeof(LABEL));
                    if (color_out)
                        color = (PIXEL *)mxRealloc(color, maxlabel*sizeof(PIXEL));
                    if (edge_out)
                        edge = (unsigned int *)mxRealloc(edge, maxlabel*sizeof(int));

                    /* region size */
                    blobsize = (int *)mxRealloc(blobsize, maxlabel*sizeof(int));
                }
                color[curlab] = curpix;
                edge[curlab] = row + height*col + 1;
                //printf("color %d %f\n", curlab, curpix);
#ifdef  DEBUG
                printf("new label(%d,%d): %d\n", 
                    row, col, curlab);
#endif
            }
            blobsize[curlab] += 1;

            PIX(limage,row,col) = curlab;
            prevlab = curlab;
            prevpix = curpix;
        }
    }

#ifdef  DEBUG
    printf("max lim is %d\n", newlabel);
#endif

    /*
     * The label indirection map can have have quite long chains of indirection
     * as the result of region merges.
     *
     * We pass over the map, and set each entry to its final value, or 0 meaning
     * that it is not indirected.
     *

     */

     /*
      * create a new label map that maps all old labels to new consecutive labels
      */
#ifdef  DEBUG
    printf("----------------------\nlmap:\n");
#endif
    for (i=1,nlabels=0; i<=newlabel; i++) {
#ifdef  DEBUG
        printf("(%d) = %d\n", i, lmap[i]);
#endif
        if (lmap[i] == 0)
            lmap2[i] = ++nlabels;   /* assign new sequential label */
    }
    /*
     * now adjust the label map so that consecutive labels appear in the
     * labelled image, ie. no missing labels.
     */
    for (i=0; i<=newlabel; i++)
        if (lmap[i] != 0) {
            j = lresolve(i);
            lmap2[i] = lmap2[j];
        }
#ifdef  DEBUG
    printf("----------------------\nlmap2:\n");
    for (i=1; i<=newlabel; i++)
        printf("old(%d) -> %d\n", i, lmap2[i]);
#endif

#ifdef  DEBUG
    printf("----------------------\nparents:\n");
    for (i=1; i<=newlabel; i++)
        printf("parent[%d] = %d\n", i, parents[i]);
#endif

    /*
     * resolve the labels in the parent array and assign to double proc
     * output array
     */
    if (parent_out) {
        LABEL   *lab;

        lab = mxCalloc(nlabels, sizeof(LABEL));
        for (i=1; i<=newlabel; i++) {
            LABEL   par = parents[i], child;

            if (par) {
                child = lmap2[i];
                par = lmap2[par];
                lab[child-1] = par;
                //printf("parent [%d] = %d\n", child, par);
            }
        }
        mxFree(parents);
        *parent_out = lab;
    }

    if (color_out || edge_out) {
        PIXEL   *c;
        unsigned int    *e;

        c = (PIXEL *)mxCalloc(nlabels, sizeof(PIXEL));
        e = (unsigned int *)mxCalloc(nlabels, sizeof(unsigned int));
        for (i=1; i<=newlabel; i++) {
            LABEL     l;

            l = lmap2[i];
            c[l-1] = color[i];
            if (e[l-1] == 0)
                e[l-1] = edge[i];
            //printf("edge %d %d %d\n", i, l, edge[i]);
            //printf("color [%d] = %f\n", l, color[i]);
        }
        mxFree(color);
        mxFree(edge);
        *color_out = c;
        *edge_out = e;
   }

    /*
     * resolve the labels in the integer labelled image and assign
     * to the double prec. output image
     */
    for (row=0; row<height; row++)
        for (col=0; col<width; col++)
            PIX(limage,row,col) = lmap2[ PIX(limage,row,col) ];

    mxFree(lmap);
    mxFree(lmap2);
    mxFree(blobsize);

    return(nlabels);
}

/*
 * resolve a label to it's true value via the label map
 */
LABEL
lresolve(LABEL l)
{
    LABEL   i;

    for (i=l; lmap[i] > 0; )
        i = lmap[i];
#ifdef  DEBUG
    if (l != i)
        printf("resolved %d to %d\n", l, i);
#endif
    return i;
}


/*
 * MATLAB interface function, check arguments, then call ilabel() above
 */
void
mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    PIXEL   *im, *color;
    unsigned int     *edge;
    LABEL   maxlabel, *label, *parents;
    int height, width;


    /* handle input arguments */

    int connectivityWays = 4;
    int minSize = 0;

    switch (nrhs) {
    case 3:
        minSize = mxGetScalar(MINSIZE_IN);

    case 2: {
        double  opt = mxGetScalar(CONNECT_IN);

        if (opt == 8)
            connectivityWays = 8;
        else if (opt != 4)
            mexErrMsgTxt("ILABEL connectivity must be 4 or 8.");
        }
    case 1: {
        
        if (mxIsComplex(IM_IN))
            mexErrMsgTxt("ILABEL requires a real matrix.");

        if (mxGetNumberOfDimensions(prhs[0]) > 2)
            mexErrMsgTxt("Only greylevel images allowed.");

        height = mxGetM(prhs[0]);
        width = mxGetN(prhs[0]);

        /*
        printf("size %d x %d\n", height, width);
        */

        // allocate storage for the image of type PIXEL
        im = mxCalloc(width*height, sizeof(PIXEL));

        // convert input images to PIXEL type
        switch (mxGetClassID(prhs[0])) {
            case mxLOGICAL_CLASS: {
                PIXEL *p = im;
                mxLogical *q = (mxLogical *)mxGetPr(prhs[0]);
                int     i;
                
                for (i=0; i<width*height; i++)
                        *p++ = *q++;
                break;
            }
            case mxUINT8_CLASS: {
                PIXEL *p = im;
                unsigned char  *q = (unsigned char *)mxGetPr(prhs[0]);
                int     i;
                
                for (i=0; i<width*height; i++)
                        *p++ = *q++;
                break;
            }
            case mxUINT16_CLASS: {
                PIXEL *p = im;
                unsigned short  *q = (unsigned short *)mxGetPr(prhs[0]);
                int     i;
                
                for (i=0; i<width*height; i++)
                        *p++ = *q++;
                break;
            }
            case mxDOUBLE_CLASS: {
                PIXEL *p = im;
                double  *q = mxGetPr(prhs[0]);
                int     i;
                
                for (i=0; i<width*height; i++)
                        *p++ = *q++;
                break;
            }
            default:
                mexErrMsgTxt("Only logical, uint8, uint16 or double images allowed");
        }
        break;
    }
    default:
        mexErrMsgTxt("ILABEL requires one or more input arguments");
        break;
    }

    // allocate a LABEL image the same size as input image
    label = (LABEL *)mxCalloc(width*height, sizeof(LABEL));

    /****************************************************************************************
     * Do the actual computations in a subroutine 
     ****************************************************************************************/
    maxlabel = ilabel(im, width, height, connectivityWays, minSize,
        label, &parents, &color, &edge);

    /* return the required output arguments */
    switch (nlhs) {
    case 5: {
        double  *p;
        int i;

        EDGE_OUT = mxCreateNumericMatrix(maxlabel, 1, mxDOUBLE_CLASS, mxREAL);
        p = (double *)mxGetData(EDGE_OUT);
        for (i=0; i<maxlabel; i++)
            p[i] = edge[i];
        }
        /* fall through */
    case 4: {
        double   *p;
        int i;

        COLOR_OUT = mxCreateNumericMatrix(maxlabel, 1, mxDOUBLE_CLASS, mxREAL);
        p = (double *)mxGetData(COLOR_OUT);
        for (i=0; i<maxlabel; i++)
            p[i] = color[i];
        }
        /* fall through */
    case 3: {
        LABEL   *p;
        int i;

        PARENT_OUT = mxCreateNumericMatrix(maxlabel, 1, mxUINT32_CLASS, mxREAL);
#ifdef  __LCC__
        p = (unsigned int *)mxGetData(PARENT_OUT);
#else
        p = (uint32_t *)mxGetData(PARENT_OUT);
#endif
        for (i=0; i<maxlabel; i++)
            p[i] = parents[i];
        }
        /* fall through */
    case 2: {
        MAX_OUT = mxCreateDoubleScalar( (double)maxlabel);
        }
        /* fall through */
    case 1: {
        double   *p;
        LABEL   *l;
        int i;

        /* Create a matrix for the return argument */
        IM_OUT = mxCreateNumericMatrix(height, width, mxDOUBLE_CLASS, mxREAL);
        p = (double *)mxGetData(IM_OUT);
        l = label;
        for (i=0; i<width*height; i++)
            *p++ = *l++;
        break;
      }
    }

    mxFree(label);
    mxFree(edge);
    mxFree(color);
    mxFree(parents);
    mxFree(im);

    return;
}
