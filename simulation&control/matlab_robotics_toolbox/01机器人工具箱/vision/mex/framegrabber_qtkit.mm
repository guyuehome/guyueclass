/*
 * framegrabber_qtkit.mm
 *
 * A MEX wrapper for QTKit camera interface.
 *
 * Peter Corke 26/1/2012
 * 
 * Based on CvCapture.mm distributed as part of OpenCV (modules/highgui/src)
 *
 *  Created by Nicholas Butko on 11/3/09.
 *  Copyright 2009. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution. 
 * 3. The name of the author may not be used to endorse or promote products 
 *    derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO 
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * TODO
 *   STOP/START methods are pretty fragile and crash MATLAB
 */
#include <iostream>
#import <QTKit/QTKit.h>

using namespace std; 

/********************** Declaration of class headers ************************/

/*****************************************************************************
 *
 * CaptureDelegate Declaration. 
 *
 * CaptureDelegate is notified on a separate thread by the OS whenever there
 *   is a new frame. When "updateImage" is called from the main thread, it
 *   copies this new frame into an IplImage, but only if this frame has not
 *   been copied before. When "getOutput" is called from the main thread, 
 *   it gives the last copied IplImage. 
 *
 *****************************************************************************/

#ifndef QTKIT_VERSION_7_6_3
#define QTKIT_VERSION_7_6_3         70603
#define QTKIT_VERSION_7_0           70000
#endif

#ifndef QTKIT_VERSION_MAX_ALLOWED
#define QTKIT_VERSION_MAX_ALLOWED QTKIT_VERSION_7_0
#endif

#define DISABLE_AUTO_RESTART 999

@interface CaptureDelegate : NSObject
{
	int newFrame; 
    CVImageBufferRef  mCurrentImageBuffer;
	uint8_t* imagedata; 
	size_t currSize; 
    size_t rowBytes;
    int    nframes;

  @package;
    size_t width;
    size_t height;
}

- (void)captureOutput:(QTCaptureOutput *)captureOutput 
  didOutputVideoFrame:(CVImageBufferRef)videoFrame 
	 withSampleBuffer:(QTSampleBuffer *)sampleBuffer 
	   fromConnection:(QTCaptureConnection *)connection;

- (void)captureOutput:(QTCaptureOutput *)captureOutput 
didDropVideoFrameWithSampleBuffer:(QTSampleBuffer *)sampleBuffer 
	   fromConnection:(QTCaptureConnection *)connection;

- (int)updateImage; 
- (uint8_t*)getOutput; 

@end

/*****************************************************************************
 *
 * CvCaptureCAM Declaration. 
 *
 * CvCaptureCAM is the instantiation of a capture source for cameras.
 *
 *****************************************************************************/

class CvCaptureCAM {
public:
	CvCaptureCAM(int cameraNum = -1) ;
	~CvCaptureCAM(); 
	virtual bool grabFrame(); 
	virtual uint8_t* retrieveFrame(int);
    virtual void getProperty(int &width, int &height);
    virtual void setProperty(int &width, int &height);
	virtual int didStart(); 
	
	
	QTCaptureSession            *mCaptureSession;
	QTCaptureDeviceInput        *mCaptureDeviceInput;
	QTCaptureDecompressedVideoOutput    *mCaptureDecompressedVideoOutput;
	CaptureDelegate* capture; 
	char        *mCaptureDeviceName;
	
	int startCaptureDevice(int cameraNum); 
	void stopCaptureDevice(); 
    int isRunning();
	
	bool grabFrame(double timeOut); 
	
	int camNum; 
	int width; 
	int height; 
	int settingWidth;
	int settingHeight;  
	int started; 
	int disableAutoRestart; 
	
}; 

#include    "camera_ops.h"
#include    "mex.h"

/*
#include    <Foundation/NSObject.h>
#include    <Foundation/NSString.h>
#include    <Foundation/NSAutoreleasePool.h>
*/

/* forward defines */
static int getInt(const mxArray *p);
void camera_list();

/* local storage */
static CvCaptureCAM * grabber;

void mexFunction(
         int          nlhs,
         mxArray      *plhs[],
         int          nrhs,
         const mxArray *prhs[]
         )
{
    int opcode;

    if (nrhs == 0)
        mexErrMsgTxt("no arguments provided\n");

    opcode = getInt(prhs[0]);

    /* check for valid grabber pointer */
    switch (opcode) {
        case CAMERA_OP_LIST:
        case CAMERA_OP_OPEN:
		break;
        default:
	    if (grabber == NULL)
		    mexErrMsgTxt("null pointer, device closed?");
    }

    switch (opcode) {
        case CAMERA_OP_LIST:
            camera_list();
            break;

        case CAMERA_OP_OPEN: {
            int camera = 0;

            if (nrhs > 1)
                camera = getInt(prhs[1]);

            grabber = new CvCaptureCAM(camera);
            break;
        }

        case CAMERA_OP_CLOSE:
            delete grabber;
            grabber = NULL;
            break;

        case CAMERA_OP_START:
            grabber->startCaptureDevice(grabber->camNum);
            break;

        case CAMERA_OP_STOP:
            grabber->stopCaptureDevice();
            break;

        case CAMERA_OP_ISRUNNING:
            if (nlhs > 0) {
                double r = grabber->isRunning();
                plhs[0] = mxCreateDoubleScalar(r);
            }
            break;

        case CAMERA_OP_GET_PARAMS:
            if (nlhs > 0) {
                int width, height;
                double *d;

                grabber->getProperty(width, height);
                plhs[0] = mxCreateDoubleMatrix(
                    (mwSize) 1, (mwSize) 2, mxREAL);
                d = mxGetPr(plhs[0]);
                d[0] = width;
                d[1] = height;
            }
            break;

        case CAMERA_OP_SET_WIDTH_HEIGHT:
            break;

        case CAMERA_OP_GRAB: {
            if (nlhs > 0) {
                uint8_t *ret_image, *frame;
                mwSize  dims[3];

                // grab the frame from QTKit using the Objective C class
                grabber->grabFrame();
                frame = grabber->retrieveFrame(0);
                if (frame == NULL) {
                    mexWarnMsgTxt("failed to grab a frame");
                    plhs[0] = mxCreateDoubleMatrix(
                        (mwSize) 0, (mwSize) 0, mxREAL);
                    return;
                }

                // assign an output matrix
                //   ret_image points to that matrix
                int width = grabber->capture->width,  height = grabber->capture->height;
                dims[0] = height; dims[1] = width; dims[2] = 3;
                plhs[0] = mxCreateNumericArray(
                    (mwSize) 3, dims, mxUINT8_CLASS, mxREAL);
                ret_image = (uint8_t *) mxGetData(plhs[0]);

                // copy the data from B G R A B G R A row format to Matlab order
                for (int plane=0; plane<3; plane++) {
                    uint8_t *src, *dst, *dst0;

                    src = &frame[2-plane];      // first pixel of src for this color plane
                    dst0 = &ret_image[plane*width*height];  // first pixel of dst for this color plane

                    for (int row=0; row<height; row++) {
                        dst = dst0++;           // first pixel for this row
                        for (int col=0; col<width; col++) {
                            *dst = *src;
                            dst += height;      // next column
                            src += 4;           // next pixel of the same color
                        }
                    }
                }
            }
            break;
        }

        case CAMERA_OP_GET_NAME:
            if (nlhs > 0) {
                plhs[0] = mxCreateString( grabber->mCaptureDeviceName );
            }
            break;

        default: {
            char err[1024];

            sprintf(err, "unknown operation code %d\n", opcode);
            mexErrMsgTxt(err);
          }
        }
    return;
}


void
camera_list()
{
	NSAutoreleasePool* localpool = [[NSAutoreleasePool alloc] init];

	NSArray* devices = [QTCaptureDevice inputDevicesWithMediaType:QTMediaTypeVideo];

    for (int i=0; i<[devices count]; i++)
        printf("dev[%d] %s\n", i,
            [[[devices objectAtIndex:i] localizedDisplayName] UTF8String]);

	[localpool drain]; 
}

/********************** Implementation of Classes ****************************/ 

/*****************************************************************************
 *
 * CvCaptureCAM Implementation. 
 *
 * CvCaptureCAM is the instantiation of a capture source for cameras.
 *
 *****************************************************************************/

CvCaptureCAM::CvCaptureCAM(int cameraNum) {
	mCaptureSession = nil;
	mCaptureDeviceInput = nil;
	mCaptureDecompressedVideoOutput = nil;
	capture = nil; 
	
	width = 0; 
	height = 0; 
	settingWidth = 0; 
	settingHeight = 0; 
	disableAutoRestart = 0; 
	
	camNum = cameraNum; 
	
	if (!startCaptureDevice(camNum)) {
		cout << "Warning, camera failed to properly initialize!" << endl; 
		started = 0; 
	} else {
		started = 1; 
	}
	
}

CvCaptureCAM::~CvCaptureCAM() {
	stopCaptureDevice(); 
	
	cout << "Cleaned up camera." << endl; 
}

int CvCaptureCAM::didStart() {
	return started; 
}

int CvCaptureCAM::isRunning() {
	return capture != nil;
}


bool CvCaptureCAM::grabFrame() {
	return grabFrame(5); 
}

bool CvCaptureCAM::grabFrame(double timeOut) {
    // poll the capture delegate object (in a thread kind way) until an
    // image is available
	
	NSAutoreleasePool* localpool = [[NSAutoreleasePool alloc] init];

	double sleepTime = 0.005;
	double total = 0;
    int retval = 0;
    
	
    NSDate * startLoop = [NSDate date];  // time now
    
	while ([startLoop timeIntervalSinceNow ] > -timeOut) {  // run till now+timeOut
        // try to grab a frame
        if (retval=[capture updateImage])
            break;
        
        // run the event loop once but block no more than sleepTime
        NSDate *runUntil = [NSDate dateWithTimeIntervalSinceNow:sleepTime];
        [[NSRunLoop currentRunLoop] runMode: NSDefaultRunLoopMode
                                         beforeDate:runUntil];
    }

	[localpool drain];

	return retval;
}

uint8_t* CvCaptureCAM::retrieveFrame(int) {
    fprintf(stderr, "in grabFrame\n");
	return [capture getOutput]; 
}

void CvCaptureCAM::stopCaptureDevice() {
	NSAutoreleasePool* localpool = [[NSAutoreleasePool alloc] init];
	
	[mCaptureSession stopRunning];
	
	QTCaptureDevice *device = [mCaptureDeviceInput device];
    if ([device isOpen])  [device close];
	
	[mCaptureSession release];
    [mCaptureDeviceInput release];
	
	[mCaptureDecompressedVideoOutput setDelegate:mCaptureDecompressedVideoOutput]; 
	[mCaptureDecompressedVideoOutput release]; 
	[capture release];
    capture = nil;
	[localpool drain]; 
	
}


int CvCaptureCAM::startCaptureDevice(int cameraNum) {
	NSAutoreleasePool* localpool = [[NSAutoreleasePool alloc] init];
	
	capture = [[CaptureDelegate alloc] init]; 
	
	QTCaptureDevice *device; 
	NSArray* devices = [QTCaptureDevice inputDevicesWithMediaType:QTMediaTypeVideo];
	if ([devices count] == 0) {
		cout << "QTKit didn't find any attached Video Input Devices!" << endl; 
		[localpool drain]; 
		return 0; 
	}
	
	if (cameraNum >= 0) {
		int nCameras = [devices count];
        if( cameraNum < 0 || cameraNum >= nCameras )
            return 0;
		device = [devices objectAtIndex:cameraNum] ;
	} else {
		device = [QTCaptureDevice defaultInputDeviceWithMediaType:QTMediaTypeVideo]  ;
	}
	int success; 
	NSError* error; 
	
    if (device) {
		
		success = [device open:&error];
        if (!success) {
			cout << "QTKit failed to open a Video Capture Device" << endl; 			
			[localpool drain]; 
			return 0; 
        }
		
        mCaptureDeviceName = strdup( [[device localizedDisplayName] UTF8String] );
		mCaptureDeviceInput = [[QTCaptureDeviceInput alloc] initWithDevice:device] ;
		mCaptureSession = [[QTCaptureSession alloc] init] ;
		
        success = [mCaptureSession addInput:mCaptureDeviceInput error:&error];		
		
		if (!success) {
			cout << "QTKit failed to start capture session with opened Capture Device" << endl;
			[localpool drain]; 
			return 0; 
        }
		
		
		mCaptureDecompressedVideoOutput = [[QTCaptureDecompressedVideoOutput alloc] init];
		[mCaptureDecompressedVideoOutput setDelegate:capture];
		NSDictionary *pixelBufferOptions ;
		if (width > 0 && height > 0) {
			pixelBufferOptions = [NSDictionary dictionaryWithObjectsAndKeys:
								  [NSNumber numberWithDouble:1.0*width], (id)kCVPixelBufferWidthKey,
								  [NSNumber numberWithDouble:1.0*height], (id)kCVPixelBufferHeightKey,
								  //[NSNumber numberWithUnsignedInt:k32BGRAPixelFormat], (id)kCVPixelBufferPixelFormatTypeKey,
								  [NSNumber numberWithUnsignedInt:kCVPixelFormatType_32BGRA],
								  (id)kCVPixelBufferPixelFormatTypeKey,
								  nil]; 
		} else {
			pixelBufferOptions = [NSDictionary dictionaryWithObjectsAndKeys:
								  [NSNumber numberWithUnsignedInt:kCVPixelFormatType_32BGRA], 
								  (id)kCVPixelBufferPixelFormatTypeKey,
								  nil]; 
		}
		[mCaptureDecompressedVideoOutput setPixelBufferAttributes:pixelBufferOptions]; 
		
#if QTKIT_VERSION_MAX_ALLOWED >= QTKIT_VERSION_7_6_3
		[mCaptureDecompressedVideoOutput setAutomaticallyDropsLateVideoFrames:YES]; 
#endif
		
		
        success = [mCaptureSession addOutput:mCaptureDecompressedVideoOutput error:&error];
        if (!success) {
            cout << "QTKit failed to add Output to Capture Session" << endl; 
			[localpool drain]; 
            return 0;
        }
		
		[mCaptureSession startRunning];
		
		grabFrame(10);
		
		return 1; 
	}
	
	[localpool drain]; 
	return 0; 
}


void CvCaptureCAM::getProperty(int &width, int &height){
	NSAutoreleasePool* localpool = [[NSAutoreleasePool alloc] init]; 
	
	NSArray* connections = [mCaptureDeviceInput	connections]; 
	QTFormatDescription* format = [[connections objectAtIndex:0] formatDescription]; 
	NSSize s1 = [[format attributeForKey:QTFormatDescriptionVideoCleanApertureDisplaySizeAttribute] sizeValue]; 
	
	width=s1.width;
    height=s1.height; 
	
	[localpool drain]; 
	
}

void  CvCaptureCAM::setProperty(int &width, int &height) {
	NSAutoreleasePool* localpool = [[NSAutoreleasePool alloc] init]; 
	NSDictionary* pixelBufferOptions = [NSDictionary dictionaryWithObjectsAndKeys:
						  [NSNumber numberWithDouble:1.0*width], (id)kCVPixelBufferWidthKey,
						  [NSNumber numberWithDouble:1.0*height], (id)kCVPixelBufferHeightKey,
						  [NSNumber numberWithUnsignedInt:kCVPixelFormatType_32BGRA],
						  (id)kCVPixelBufferPixelFormatTypeKey,
						  nil]; 
	
	[mCaptureDecompressedVideoOutput setPixelBufferAttributes:pixelBufferOptions];
	grabFrame(10);
	[localpool drain]; 
}


/*****************************************************************************
 *
 * CaptureDelegate Implementation. 
 *
 * CaptureDelegate is notified on a separate thread by the OS whenever there
 *   is a new frame. When "updateImage" is called from the main thread, it
 *   copies this new frame into an IplImage, but only if this frame has not
 *   been copied before. When "getOutput" is called from the main thread, 
 *   it gives the last copied IplImage. 
 *
 *****************************************************************************/

@implementation CaptureDelegate 

- (id)init {
	[super init]; 
	newFrame = 0; 
	imagedata = NULL; 
	currSize = 0;
    nframes = 0;
	return self; 
}


-(void)dealloc {
	if (imagedata != NULL) free(imagedata); 
	[super dealloc]; 
}

// delegate method for QTCaptureDecompressedVideoOutput
//   invoked when a frame is available
//    sets the newFrame property
//    mCurrentImageBuffer points to the latest image
- (void)captureOutput:(QTCaptureOutput *)captureOutput 
  didOutputVideoFrame:(CVImageBufferRef)videoFrame 
	 withSampleBuffer:(QTSampleBuffer *)sampleBuffer 
	   fromConnection:(QTCaptureConnection *)connection {
	
    CVBufferRetain(videoFrame);  // mark new buffer to be retained
    // make a reference to current buffer
	CVImageBufferRef imageBufferToRelease  = mCurrentImageBuffer;
    
    @synchronized (self) {
		
        // update current buffer to the new frame
        mCurrentImageBuffer = videoFrame;
		newFrame = 1; 
    }
	
    // release the previous buffer
	CVBufferRelease(imageBufferToRelease);
    nframes++;
    
}

// delegate method for QTCaptureDecompressedVideoOutput
//  invoked when a frame is dropped
- (void)captureOutput:(QTCaptureOutput *)captureOutput 
didDropVideoFrameWithSampleBuffer:(QTSampleBuffer *)sampleBuffer 
	   fromConnection:(QTCaptureConnection *)connection {
	cout << "Camera dropped frame!" << endl; 
}

-(uint8_t*) getOutput {
	return imagedata; 
}

-(int) updateImage {
    // attempt to copy the image placed in shared memory by the captureOutput method
    //  return 0 if no image actually available
	if (newFrame==0)
        return 0;
	CVPixelBufferRef pixels; // a CoreVideo pixel buffer
	
	@synchronized (self){
		pixels = CVBufferRetain(mCurrentImageBuffer);
		newFrame = 0; 
	}
	
    // lock it down and get memory pointer and dimensions
	CVPixelBufferLockBaseAddress(pixels, 0);		
	uint32_t* baseaddress = (uint32_t*)CVPixelBufferGetBaseAddress(pixels);
	
	width = CVPixelBufferGetWidth(pixels);
	height = CVPixelBufferGetHeight(pixels);
	rowBytes = CVPixelBufferGetBytesPerRow(pixels);
	
    int retval = 0;
	if (rowBytes != 0) { // if it has finite size
        
		// if image size has changed, realloc buffer
		if (currSize != rowBytes*height*sizeof(uint8_t)) {
			currSize = rowBytes*height*sizeof(uint8_t); 
			if (imagedata != NULL) free(imagedata); 
            imagedata = (uint8_t *) malloc(currSize);
		}
		
        // copy pixels to imagedata
		memcpy(imagedata, baseaddress, currSize);
		retval = 1;
	}
	
	CVPixelBufferUnlockBaseAddress(pixels, 0);
	CVBufferRelease(pixels); 
	
	return retval;
}

@end

static int
getInt(const mxArray *p)
{
    return (int) mxGetScalar(p);
}
