/* firewire.c
 *
 * Firewire camera interface for Linux
 *
 *	handle = firewire(port, [res], color, rate)
 *	im = firewire(handle)
 *
 *	$Header: /home/autom/pic/cvsroot/image-toolbox/firewire.c,v 1.2 2005/10/23 11:10:56 pic Exp $
 *
 *	$Log: firewire.c,v $
 *	Revision 1.2  2005/10/23 11:10:56  pic
 *	Bug with number of planes in mono mode.
 *
 *	Revision 1.1  2005/10/21 06:07:06  pic
 *	Mex-file wrapper for reading from firewire interface using dc1394 libs
 *	under Linux.
 *
 *
 * Copyright (c) Peter Corke, 2005  Machine Vision Toolbox for Matlab
 *		pic 6/2005
 *
 */
#include "mex.h"
#include <math.h>

#include </usr/include/libraw1394/raw1394.h>
#include </usr/include/libdc1394/dc1394_control.h>


typedef struct
{
	// Firewire interface 

	raw1394handle_t handle;	// handle to raw firewire
	int             verbose;
} FWdev;

typedef struct
{
	// Firewire camera

	raw1394handle_t handle;	// handle to raw firewire
	dc1394_cameracapture cam;	// handle to camera for dc control
	int             Bus;	// bus number camera is on                
	int             isoChannel;	// ISO channel selection
	int             cameraNode;	// Camera node on the bus (chosen by index)
	int             frameRate;	// Camera frame rate
	int             colorFormat;	// Camera mode

	// Useful debugging/operation info
	dc1394_camerainfo camInfo;
	dc1394_miscinfo camMiscInfo;
	dc1394_feature_set camFeatureSet;
	int             verbose;
	int             status;
	int             errCount;
	int             frameCount;
	int             width;
	int             height;
	int             depth;

} FWcamera;


// Input Arguments 

#define	PORT_IN		prhs[0]
#define	HANDLE_IN	prhs[0]
#define	COLOR_IN	prhs[1]
#define	RATE_IN		prhs[2]

// Output Arguments 

#define	IM_OUT		plhs[0]
#define	HANDLE_OUT	plhs[0]

// forward defines
int             FW_close(FWdev * fw);

FWcamera       *FW_camera_init(FWdev * fw, int guid, int format,
			       int framerate);
int             FW_camera_close(FWcamera * c);
int             FW_init_capture(FWcamera * c);
unsigned char  *FW_grab(FWcamera * c);

void            FW_info_print(FWcamera * c);
static void           * FW_error_null(char *s);
static int FW_error(char *s);
static int fw_grab(double *dimage, int slot, int width, int height, int nplanes);
static int fw_grab2(double *Y, double *U, double *V, int slot, int height, int width);
static FWdev          * FW_open(int verbose);
static int fw_open(int port, int mode, int rate);

#define	NCAM	8

static FWcamera *camHandle[NCAM];

#define	BUFLEN	4096

void
mexFunction(int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[])
{
	mxArray        *r;
	char            color[BUFLEN];
	double          rate;
	int             port;
	int             cmode;
	int             rmode;
	int             ret;

	/*
	 * Check for proper number of arguments 
	 */

	switch (nrhs) {
	case 3: {
		double         *p;

		// h = firewire(port, color, rate);
		//

		port = round(mxGetScalar(PORT_IN));

		if (mxGetString(COLOR_IN, color, BUFLEN) != 0)
			mexErrMsgTxt("bad string for colormode");
		if (strcmp(color, "mono") == 0)
			cmode = MODE_640x480_MONO;
		else if (strcmp(color, "rgb") == 0)
			cmode = MODE_640x480_RGB;
		else if (strcmp(color, "yuv") == 0)
			cmode = MODE_640x480_YUV422;
		else
			mexErrMsgTxt("bad color spec string");

		// set rate to camera rate less than or equal 
		// to request
		rate = mxGetScalar(RATE_IN);
		if (rate >= 60)
			rmode = FRAMERATE_60;
		else if (rate >= 30)
			rmode = FRAMERATE_30;
		else if (rate >= 15)
			rmode = FRAMERATE_15;
		else if (rate >= 7.5)
			rmode = FRAMERATE_7_5;
		else if (rate >= 3.75)
			rmode = FRAMERATE_3_75;
		else
			rmode = FRAMERATE_1_875;

		ret = fw_open(port, cmode, rmode);
		if (ret < 0)
			mexErrMsgTxt("error opening firewire camera");

		//fprintf(stderr, "rmode=%d, cmode=%d, slot=%d\n", rmode, cmode, ret);
		// save the handle and return it
		HANDLE_OUT = mxCreateDoubleMatrix(1, 1, mxREAL);
		p = mxGetPr(HANDLE_OUT);
		*p = ret;
		break;
	}
	case 1: {
		//  im = firewire(h);
		//
		//  grab a frame from the firewire device and return
		//  it to matlab workspace
		//
		int             slot = round(mxGetScalar(HANDLE_IN));
		double         *p;
		FWcamera       *cam = camHandle[slot];

		if (nlhs == 0)
			return;

		// allocate space according to color format
		switch (cam->colorFormat) {
		case MODE_640x480_MONO:
			IM_OUT = mxCreateDoubleMatrix(cam->height,
						      cam->width,
						      mxREAL);
			if (IM_OUT == NULL) {
				mexErrMsgTxt("couldnt allocate matrix");
				return;
			}
			p = mxGetPr(IM_OUT);
			fw_grab(p, slot, cam->height, cam->width, 1);
			break;

		case MODE_640x480_YUV422: {
			mxArray	*Y, *U, *V;
			const char	*names[] = {"y", "u", "v"};
			int	dims[] = {1, 1};


			Y = mxCreateDoubleMatrix(cam->height, cam->width, mxREAL);

			U = mxCreateDoubleMatrix(cam->height, cam->width/2, mxREAL);
			V = mxCreateDoubleMatrix(cam->height, cam->width/2, mxREAL);

			if ((Y == NULL) || (U == NULL) || (V == NULL)) 
				mexErrMsgTxt("couldnt allocate plane array");
			// create a structure
			IM_OUT = mxCreateStructArray(2, dims, 3, names);
			if (IM_OUT == NULL)
				mexErrMsgTxt("couldnt allocate cell array");
			mxSetField(IM_OUT, 0, "y", Y);
			mxSetField(IM_OUT, 0, "u", U);
			mxSetField(IM_OUT, 0, "v", V);

			fw_grab2(mxGetPr(Y), mxGetPr(U), mxGetPr(V), slot, cam->height, cam->width);
			break;
		    }
		case MODE_640x480_RGB: {
			int             dims[3];

			dims[0] = cam->height;
			dims[1] = cam->width;
			dims[2] = 3;
			IM_OUT = mxCreateNumericArray(3, dims,
						      mxDOUBLE_CLASS,
						      mxREAL);
			if (IM_OUT == NULL) {
				mexErrMsgTxt("couldnt allocate matrix");
				return;
			}
			p = mxGetPr(IM_OUT);
			fw_grab(p, slot, cam->height, cam->width, 3);
			break;
		    }
		}

		return;
	}
	default:
		mexErrMsgTxt("Incorrect number of arguments");
	}
}

static int
fw_open(int port, int mode, int rate)
{
	FWcamera       *cam;
	FWdev          *fw;
	int             slot;

	// initialize a connection to the camera and stash the
	// handle in the slot table
	fw = FW_open(port);
	if (fw == NULL)
		return -1;
	//printf("cmode = %d, rate = %d\n", mode, rate);
	cam = FW_camera_init(fw, 0, mode, rate);
	if (cam == NULL)
		return -1;

	if (FW_init_capture(cam) < 0)
		return FW_error("camera init capture failed");

	//fprintf(stderr, "image is %d x %d\n", cam->width, cam->height);

	// put the camera handle into a vacant slot
	for (slot = 0; slot < NCAM; slot++)
		if (camHandle[slot] == NULL) {
			camHandle[slot] = cam;
			return slot;
		}
	return -1;
}

static int
fw_grab(double *dimage, int slot, int height, int width, int nplanes)
{
	FWcamera       *cam = camHandle[slot];
	unsigned char  *image;
	int             r, c, p;

	image = (unsigned char *) FW_grab(cam);

	for (p = 0; p < nplanes; p++)
		for (c = 0; c < width; c++)
			for (r = 0; r < height; r++)
				*dimage++ = (double) image[c*nplanes+r*width*nplanes+p];
}

static int
fw_grab2(double *Y, double *U, double *V, int slot, int height, int width)
{
	FWcamera       *cam = camHandle[slot];
	unsigned char  *image;
	int             r, c, p;

	image = (unsigned char *) FW_grab(cam);

	for (r = 0; r < height; r++)
		for (c = 0; c < width; c+=2) {
			U[(c>>1)*height+r] = (double) (*image++);
			Y[c*height+r] = (double) (*image++);
			V[(c>>1)*height+r] = (double) (*image++);
			Y[(c+1)*height+r] = (double) (*image++);
		}
}

/*******************************************************************
  below be dragons...
  ******************************************************************/

#include <assert.h>
#include <unistd.h>
#include <memory.h>
#include <time.h>


// Support for the new version of libdc1394
#define DC1394_0_9_0

static const int DMA_BUFFERS = 40;
static const char *DMA_DEVICE_NAME = "/dev/video1394";

static int
FW_error(char *s)
{
	fprintf(stderr, s);
	return -1;
}

static void           *
FW_error_null(char *s)
{
	fprintf(stderr, s);
	return NULL;
}

static FWdev          *
FW_open(int verbose)
{

  /*-----------------------------------------------------------------------
   * Get the raw firewire handle on the desired firewire bus (i/o card)
   *-----------------------------------------------------------------------*/
	FWdev          *fw;

	fw = (FWdev *) calloc(1, sizeof(FWdev));
	if (fw == NULL)
		FW_error_null("FW_i1394_open: no mem");
	fw->handle = dc1394_create_handle(0);	// bus number
	if (fw->handle == NULL)
		FW_error_null("FW_i1394_open: create_handle failed");

	fw->verbose = verbose;
	if (fw->verbose)
		fprintf(stderr, "ifImageServerFW: acquired handle\n");

	return fw;
}

// Initialize the Firewire camera
FWcamera       *
FW_camera_init(FWdev * fw, int guid, int format, int framerate)
{
	nodeid_t       *cameras;
	int             nodeCount, cameraCount;
	dc1394bool_t    isCamera;
	FWcamera       *c;
	static char    *func = "FW_camera_init";

	c = (FWcamera *) calloc(1, sizeof(FWcamera));
	if (c == NULL)
		return FW_error_null("FW_init_camera: no mem");

	c->verbose = fw->verbose;
	c->handle = fw->handle;
	c->colorFormat = format;
	c->frameRate = framerate;
	c->verbose = 0;

  /*-----------------------------------------------------------------------
   * Find the specified camera, or just the first camera on the bus
   * if no camera ID is specified
   *-----------------------------------------------------------------------*/
	nodeCount = raw1394_get_nodecount(fw->handle);
	if (c->verbose)
		fprintf(stderr, "Nodes on 1394 interface: %d\n", nodeCount);

	if (guid != 0) {
		/*
		 * If the ID is specified, try to get that camera first... 
		 */
		if (c->verbose)
			fprintf(stderr,
				"Looking for camera ID: %x\n",
				guid);
		cameras =
			dc1394_get_sorted_camera_nodes(fw->handle, 1, &guid,
						       &cameraCount, 1);
		if (!cameras || cameraCount < 1)
			return FW_error_null
				("FW_init_camera: camera  not found on bus");

	}
	else {
		/*
		 * If no camera ID is specified, just grab the first camera 
		 */
		if (c->verbose)
			fprintf(stderr,
				"FW_init_camera: Looking for first camera on bus...\n");
		cameras = dc1394_get_camera_nodes(fw->handle, &cameraCount, 1);	// 1 = print
		if (!cameras || cameraCount < 1)
			return FW_error_null
				("FW_init_camera: no cameras on bus");
	}

	/*
	 * Get the node number of the desired camera 
	 */
	c->cameraNode = cameras[0];
	if (c->verbose)
		fprintf(stderr,
			"%s: found %d cameras.  Requested camera is node %d\n",
			func, cameraCount, c->cameraNode);

  /*-----------------------------------------------------------------------
   * Make sure the camera is valid
   *-----------------------------------------------------------------------*/

	/*
	 * Make sure the node is a camera 
	 */
	if (dc1394_is_camera(fw->handle, c->cameraNode, &isCamera) !=
	    DC1394_SUCCESS)
		return FW_error_null("FW_init_camera: camera check failed");
	if (!isCamera)
		return FW_error_null("FW_init_camera: node is not a camera");
	/*
	 * Make sure the camera is not the root node! 
	 */
	if (cameras[c->cameraNode] == nodeCount - 1) {
		fprintf(stderr,
			"\n*********************************************************\n"
			"Sorry, your camera is the highest numbered node\n"
			"of the bus, and has therefore become the root node.\n"
			"The root node is responsible for maintaining \n"
			"the timing of isochronous transactions on the IEEE \n"
			"1394 bus.  However, if the root node is not cycle master \n"
			"capable (it doesn't have to be), then isochronous \n"
			"transactions will not work.  The host controller card is \n"
			"cycle master capable, however, most cameras are not.\n\n"
			"The quick solution is to add the parameter \n"
			"attempt_root=1 when loading the OHCI driver as a \n"
			"module.  So please do (as root):\n\n"
			"   rmmod ohci1394\n"
			"   insmod ohci1394 attempt_root=1\n\n"
			"for more information see the FAQ at \n"
			"http://linux1394.sourceforge.net/faq.html#DCbusmgmt\n\n"
			"\n*********************************************************\n");
		return NULL;
	}

  /*-----------------------------------------------------------------------
   * Setup the camera to grab images
   *-----------------------------------------------------------------------*/

	/*
	 * Turn the camera on 
	 */
	if (dc1394_camera_on(fw->handle, c->cameraNode) != DC1394_SUCCESS)
		return FW_error_null
			("FW_init_camera: could not power up camera");
	if (fw->verbose)
		fprintf(stderr, "Turned camera power on\n");

	if (dc1394_stop_iso_transmission(fw->handle, c->cam.node) !=
	    DC1394_SUCCESS) {
		fprintf(stderr, "WARN: could not stop ISO transmission\n");
	}
	/*
	 * Setup the capture format 
	 */
	if (FW_init_capture(c) < 0)
		return FW_error_null
			("FW_init_camera: camera failed to initalize");

  /*-----------------------------------------------------------------------
   *  have the camera start sending us data
   *-----------------------------------------------------------------------*/
	if (dc1394_start_iso_transmission(fw->handle, c->cam.node) !=
	    DC1394_SUCCESS)
		return FW_error_null
			("FW_init_camera: unable to start iso transmission\n");

  /*-----------------------------------------------------------------------
   * Get feature set, etc. from this camera
   *-----------------------------------------------------------------------*/
	if ((dc1394_get_camera_feature_set(fw->handle,
					   c->cam.node,
					   &c->camFeatureSet) !=
	     DC1394_SUCCESS)
	    || (dc1394_get_camera_info(fw->handle,
				       c->cam.node,
				       &c->camInfo) != DC1394_SUCCESS)
	    || (dc1394_get_camera_misc_info(fw->handle,
					    c->cam.node,
					    &c->camMiscInfo) !=
		DC1394_SUCCESS)) {
		return FW_error_null
			("FW_init_camera: could not get info from camera");
	}

  /*-----------------------------------------------------------------------
   * Print camera info and feature set 
   *-----------------------------------------------------------------------*/
	if (c->verbose) {
		dc1394_print_camera_info(&c->camInfo);
		dc1394_print_feature_set(&c->camFeatureSet);
		FW_info_print(c);
	}


	/*
	 * Reset frame and error counts 
	 */
	c->frameCount = c->errCount = 0;

	/*
	 * Initialized successfully! 
	 */
	c->status = 0;

	return c;
}

// Initialize the capturing
int
FW_init_capture(FWcamera * c)
{
	/*
	 * NOTE: We use camera node for iso channel currently 
	 */
	/*
	 * if (dc1394_setup_capture( all args the same, but remove DMA buffer len 
	 */
	if (c->verbose)
		fprintf(stderr, "Setting up DMA capture...\n");
	if (c->verbose)
		fprintf(stderr, "Mode, Frame rate, speed: %d %d %d\n",
			c->colorFormat, c->frameRate, SPEED_400);

	/*
	 * Set the default device filename (if only one card and not using devfs 
	 c->cam.dma_device_file = DMA_DEVICE_NAME;
	 */

	/*
	 * Call the setup routine 
	 */
#ifdef	notdef
	if (dc1394_dma_setup_capture(c->handle, c->cameraNode,	/* Node number */
				     0,	/* isoc channel */
				     FORMAT_VGA_NONCOMPRESSED, c->colorFormat, SPEED_400, c->frameRate, DMA_BUFFERS,	/* DMA BUFFER LENGTH */
				     1,	/* Drop frames ? */
				     "/dev/video1394/0",	/* Device file name */
				     &c->cam) != DC1394_SUCCESS) {
		// check permission on firewire device and parameters are
		// achievable by the camera
		return FW_error("FW_init_capture: Could not setup DMA buffer");
	}
#else
	if (dc1394_setup_capture(c->handle, c->cameraNode,	/* Node number */
				 0,	/* isoc channel */
				 FORMAT_VGA_NONCOMPRESSED,
				 c->colorFormat,
				 SPEED_400,
				 c->frameRate, &c->cam) != DC1394_SUCCESS) {
		// check permission on firewire device and parameters are
		// achievable by the camera
		return FW_error("FW_init_capture: Could not setup DMA buffer");
	}
#endif

#ifdef	notdef
	{
		quadlet_t       val;
		dc1394_query_supported_modes(c->handle, c->cameraNode,
					     FORMAT_VGA_NONCOMPRESSED, &val);
		printf("mode query %d\n", val);
	}
#endif
	/*
	 * Set internal width and height values 
	 */
	c->width = c->cam.frame_width;
	c->height = c->cam.frame_height;
	if (c->colorFormat == MODE_640x480_MONO)
		c->depth = 1;
	else if (MODE_640x480_RGB == c->colorFormat ||
		 MODE_640x480_YUV422 == c->colorFormat)
		c->depth = 3;
	else
		return FW_error
			("FW_init_capture: invalid color format (not mono or rgb)");

	if (c->verbose) {
		fprintf(stderr, "Image parameters (w,h,d): (%d,%d,%d)\n",
			c->width, c->height, c->depth);
		fprintf(stderr, "Image server buffer ptr: %p\n",
			(void *) c->cam.capture_buffer);
	}

	/*
	 * Allocate buffer 
	 */
	//c->buffer = new unsigned char[c->width * c->height * c->depth];

	return 0;
}

int
FW_camera_close(FWcamera * c)
{
	/*
	 * Release the camera 
	 */
	/*
	 * Release memory and handle 
	 dc1394_dma_release_camera(c->handle, &c->cam);
	 */

	/*
	 * Tell video1394 to stop iso reception 
	 dc1394_dma_unlisten(c->handle, &c->cam);
	 */

	/*
	 * Stop ISO transmission 
	 */
	dc1394_stop_iso_transmission(c->handle, c->cam.node);

	/*
	 * dc1394_release_camera(w->handle, &c->cam); For non-dma version 
	 */
	return 0;
}

int
FW_close(FWdev * fw)
{
	/*
	 * Release the camera and handle 
	 */
	if (fw->handle) {
		raw1394_destroy_handle(fw->handle);
	}
	return 0;
}

//////////////////////////////////////////////////////////
// Get the next image and return it
//////////////////////////////////////////////////////////
unsigned char  *
FW_grab(FWcamera * c)
{
	dc1394bool_t    isoOn = DC1394_FALSE;

	/*
	 * Make sure we are transmitting 
	 */
	if (dc1394_get_iso_status(c->handle, c->cam.node, &isoOn) !=
	    DC1394_SUCCESS)
		return FW_error_null("FW_grab: error checking iso status");
	if (DC1394_TRUE != isoOn)
		return FW_error_null("FW_grab: not iso transmitting");

	/*
	 * Release the previous DMA buffer 
	 if (dc1394_dma_done_with_buffer(&c->cam) != DC1394_SUCCESS)
	 return FW_error_null("FW_grab: Could not release DMA buffer");
	 */

	/*
	 * Get an image 
	 if (dc1394_dma_single_capture(&c->cam) != DC1394_SUCCESS)
	 return FW_error_null("FW_grab: getImage capture failed");
	 */
	if (dc1394_single_capture(c->handle, &c->cam) != DC1394_SUCCESS)
		return FW_error_null("FW_grab: getImage capture failed");

	/*
	 * Increment the frame count 
	 */
	c->frameCount++;

	/*
	 * Return the buffer 
	 */
	return (unsigned char *) c->cam.capture_buffer;
}

//////////////////////////////////////////////////////////
// Print miscellaneous information
//////////////////////////////////////////////////////////
void
FW_info_print(FWcamera * c)
{

	return;
	fprintf(stderr, "\tformat: %i \tmode: %i \tframerate: %i\n",
		c->camMiscInfo.format,
		c->camMiscInfo.mode, c->camMiscInfo.framerate);
	fprintf(stderr,
		"\tis_iso_on: %i \tiso_channel: %i \tiso_speed: %i\n",
		c->camMiscInfo.is_iso_on, c->camMiscInfo.iso_channel,
		c->camMiscInfo.iso_speed);
}
