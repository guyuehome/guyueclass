/**
 * Common definitions for edge handling in window functions
 %
 * Uses code from the package VISTA Copyright 1993, 1994 University of 
 * British Columbia.
 */

enum pad {
	PadBorder,  // replicate border pixels
	PadNone,    // dont use pixels off the edge
	PadWrap,    // consider the image wraps L-R, T-B
	PadTrim     // only return pixels where window is entirely within image
} pad_method;

/*
 *  ClampIndex
 *
 *  This macro implements behavior near the borders of the source image.
 *  Index is the band, row or column of the pixel being convolved.
 *  Limit is the number of bands, rows or columns in the source image.
 *  Label is a label to jump to to break off computation of the current
 *  destination pixel.
 */

            //case PadNone:       goto label;    \

#define ClampIndex(index, limit, label)    \
{                      \
    if (index < 0)          \
        switch (pad_method) {\
            case PadBorder: index = 0; break;\
            case PadNone:       continue;    \
            case PadWrap:       index += limit; break;    \
            default:            continue;    \
        }           \
    else if (index >= limit)        \
        switch (pad_method) {       \
            case PadBorder: index = limit - 1; break; \
            case PadNone:       continue;    \
            case PadWrap:       index -= limit; break;      \
            default:            continue;       \
        }       \
}
