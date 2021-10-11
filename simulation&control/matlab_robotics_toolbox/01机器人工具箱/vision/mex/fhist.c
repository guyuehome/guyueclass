/* ihist.c
 *
 * Fast histogram function for images.
 *
 *	N = IHIST(IM)
 *	[N,X] = IHIST(IM)
 *
 *	$Header: /home/autom/pic/cvsroot/image-toolbox/fhist.c,v 1.1 2002/08/28 04:53:05 pic Exp $
 *
 *	$Log: fhist.c,v $
 *	Revision 1.1  2002/08/28 04:53:05  pic
 *	Initial CVS version.
 *	
 *	Revision 1.1  2000/03/10 07:04:11  pic
 *	Initial revision
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
 */
#include "mex.h"
#include <math.h>

/* Input Arguments */

#define	IM_IN		prhs[0]

/* Output Arguments */

#define	N_OUT	plhs[0]
#define	X_OUT	plhs[1]

void
mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	mxArray	*Mbins, *Mbinnum;
	double	*bins, *binnum;
	const int	nbins = 256;    // default number of bins
    int m, n, i;
	unsigned int	*ibin;
    unsigned char   *im;

	/* Check for proper number of arguments */

	switch (nrhs) {
	case 1:
		break;
	default:
		mexErrMsgTxt("IHIST requires one input arguments.");
		break;
	}


	if (!mxIsUint8(IM_IN)) {
		mexErrMsgTxt("fhist: requires a uint8 image.");
	}

    // get number of pixels in the image
	m = mxGetM(IM_IN);
	n = mxGetN(IM_IN);

	im = (unsigned char *)mxGetPr(IM_IN);	/* get pointer to image */
    
    // for speed we use an integer array to hold counts
	ibin = (unsigned int *)mxCalloc(nbins, sizeof(unsigned int));

    for (i=0; i<(m*n); i++) {
        ibin[*im++] ++;
    }

    // convert integer histogram to double
	Mbins = mxCreateDoubleMatrix(nbins, 1, mxREAL);
	bins = mxGetPr(Mbins);		/* get pointer to bins */
	for (i=0; i<nbins; i++)
		*bins++ = ibin[i];
	mxFree(ibin);

	switch (nlhs) {
	case 2:
		Mbinnum = mxCreateDoubleMatrix(nbins, 1, mxREAL);
		binnum = mxGetPr(Mbinnum);
		for (i=0; i<nbins; i++)
			binnum[i] = i;
		X_OUT = Mbinnum;
		/* fall through */
	case 1:
		N_OUT = Mbins;
		break;
	}

	return;
}
