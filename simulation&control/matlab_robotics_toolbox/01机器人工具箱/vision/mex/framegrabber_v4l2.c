/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 *
 * Handy references at http://v4l2spec.bytesex.org/
 *
 * Peter Corke May 2012.
 */

#include    "mex.h"

#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <string.h>

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

// define valid camera operations
#include    "camera_ops.h"

static void open_device(void);
static void init_device(void);
static void mainloop(void);
static void uninit_device(void);
static void close_device(void);
static void start_capturing(void);
static void stop_capturing(void);
static int  read_frame(void);
static void wait_frame(void);
static int  xioctl(int fd, int request, void *arg);
static void error_handler1(const char *s);
static void error_handler2(const char *s);
static int  getInt(const mxArray *p);
void        camera_list();

// data structures
typedef enum {
        IO_METHOD_READ,
        IO_METHOD_MMAP,
} io_method;

struct buffer {
        void *                  start;
        size_t                  length;
};

// useful macros
#define CLEAR(x) memset (&(x), 0, sizeof (x))

// forward defines

// globals
static char             dev_name[1024];
static int              fd              = -1;
struct buffer           *buffers        = NULL;
static unsigned int     n_buffers       = 0;
static io_method        io;
uint8_t                 *grabbed_frame;

void mexFunction(
         int          nlhs,
         mxArray      *plhs[],
         int          nrhs,
         const mxArray *prhs[]
         )
{
    int opcode;

    if (nrhs == 0) {
        error_handler1("no arguments provided");
        return;
    }

    opcode = getInt(prhs[0]);
    fprintf(stderr, "op=%d\n", opcode);

    if (fd < 0 && !(opcode == CAMERA_OP_OPEN || opcode == CAMERA_OP_LIST)) {
        error_handler1("framegrabber_v4l2: video device not open");
        return;
    }

    switch (opcode) {
        case CAMERA_OP_LIST:
            camera_list();
            break;

        case CAMERA_OP_OPEN: {
            int camera = 0;
            buffers         = NULL;
            n_buffers       = 0;

            if (nrhs > 1)
                camera = getInt(prhs[1]);

            // build device name
            sprintf(dev_name, "/dev/video%d", camera);

            open_device();
            init_device();
            break;
        }

        case CAMERA_OP_CLOSE:
			stop_capturing ();
		    uninit_device ();
		    close_device ();
            break;

        case CAMERA_OP_START:
            start_capturing ();
            break;

        case CAMERA_OP_STOP:
			stop_capturing ();
            break;

        case CAMERA_OP_ISRUNNING:
            if (nlhs > 0) {
                double r;// = grabber->started;
                plhs[0] = mxCreateDoubleScalar(r);
            }
            break;

        case CAMERA_OP_GET_PARAMS:
            if (nlhs > 0) {
                int width, height;
                double *d;
                struct v4l2_format fmt;
                char    format[5];

                CLEAR (fmt);
                fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl (fd, VIDIOC_G_FMT, &fmt))
                        error_handler2 ("VIDIOC_G_FMT");
                plhs[0] = mxCreateDoubleMatrix(
                    (mwSize) 1, (mwSize) 3, mxREAL);
                d = mxGetPr(plhs[0]);
                d[0] = fmt.fmt.pix.width;
                d[1] = fmt.fmt.pix.height;
                switch (fmt.fmt.pix.pixelformat) {
                case V4L2_PIX_FMT_RGB24:
                    d[2] = 0;
                    break;
                case V4L2_PIX_FMT_YUYV:
                    d[2] = 1;
                    break;
                }

                fprintf(stderr, "bpl = %d\n", fmt.fmt.pix.bytesperline);
                for (int i=0; i<4; i++)
                    format[i] = (fmt.fmt.pix.pixelformat >> (i*8)) & 0xff;
                format[4] = '\0';
                fprintf(stderr, "format = %4s, colorspace = %d\n",
                    format, fmt.fmt.pix.colorspace);
            }
            break;

        case CAMERA_OP_SET_WIDTH_HEIGHT:
            break;

        case CAMERA_OP_GRAB: {
            if (nlhs > 0) {
                uint8_t *ret_image, *frame;
                mwSize  dims[3];
                int width, height, span;
                struct v4l2_format fmt;

                // by default return an empty array
                plhs[0] = mxCreateDoubleMatrix((mwSize) 0, (mwSize) 0, mxREAL);

                // get the image dimensions
                CLEAR (fmt);
                fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl (fd, VIDIOC_G_FMT, &fmt)) {
                        error_handler2("VIDIOC_G_FMT");
                        return;
                }

                width = fmt.fmt.pix.width;
                height = fmt.fmt.pix.height;
                span = fmt.fmt.pix.bytesperline;

                // get the frame from hardware
                grabbed_frame = NULL;
                wait_frame();
                fprintf(stderr, "unpacking\n");
                if (grabbed_frame == NULL) {
                    // if no frame returned then
                    // return []
                    return;
                }

                // assign an output matrix: uint8: height x width x 3
                mxDestroyArray(plhs[0]);
                dims[0] = height; dims[1] = width; dims[2] = 3;
                plhs[0] = mxCreateNumericArray(
                    (mwSize) 3, dims, mxUINT8_CLASS, mxREAL);

                fprintf(stderr, "%d %d %d\n", width, height, span);
                fprintf(stderr, "p = 0x%x\n", grabbed_frame);

                // get a pointer to the return matrix
                ret_image = (uint8_t *) mxGetData(plhs[0]);

                // copy the data from Y U Y V row format to Matlab order

                // do the Y plane first
                {
                    uint8_t *src, *dst, *dst0;
                    src = &grabbed_frame[0];  // first pixel of src is Y 
                    dst0 = &ret_image[0];     // first pixel of dst for Y
                    for (int row=0; row<height; row++) {
                        dst = dst0++;           // first pixel for this row
                        for (int col=0; col<width; col++) {
                            *dst = *src;
                            dst += height;      // next column
                            src += 2;           // next Y pixel
                        }
                    }
                }

                // now do the U and V planes.  We need to replicate the pixel
                // values since U, V are undersampled
                {
                    uint8_t *src, *dstU, *dstV, *dstU0, *dstV0;
                    dstU0 = &ret_image[width*height];  // first pixel of dst for U
                    dstV0 = &ret_image[2*width*height];  // first pixel of dst for V
                    for (int row=0; row<height; row++) {
                        dstU = dstU0++;           // first U pixel for this row
                        dstV = dstV0++;           // first V pixel for this row

                        src = &grabbed_frame[row*span+1];  // first U pixel of src

                        for (int col=0; col<width/2; col++) {
                            // write two copies of U to dst
                            *dstU = *src;
                            dstU += height;      // next column
                            *dstU = *src;
                            dstU += height;      // next column

                            src += 2;           // next V pixel

                            // write two copies of V to dst
                            *dstV = *src;
                            dstV += height;      // next column
                            *dstV = *src;
                            dstV += height;      // next column
                            src += 2;           // next U pixel
                        }
                    }
                }
            }
            fprintf(stderr, "done unpacking\n");
            break;
        }

        case CAMERA_OP_GET_NAME:
            if (nlhs > 0) {
                //plhs[0] = mxCreateString( grabber->mCaptureDeviceName );
            }
            break;

        default: {
            char err[1024];

            sprintf(err, "unknown operation code %d\n", opcode);
            error_handler1(err);
        }
    }
    return;
}

void
camera_list()
{
    int fd;
    struct v4l2_capability cap;
    struct v4l2_input input;

    for (int dev=0; dev<256; dev++) {
        char    dev_name[1024];

        sprintf(dev_name, "/dev/video%d", dev);
        if((fd = open(dev_name, O_RDONLY)) == -1) {
            return;
        }

        // if doesnt support QUERY just move on
        if (-1 == xioctl (fd, VIDIOC_QUERYCAP, &cap))
                continue;

        // if doesnt support video just move on
        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
            continue;

        fprintf(stderr, "%d: %s (%s, bus %s)\n", 
            dev, cap.card, cap.driver, cap.bus_info);

        close(fd);
    }
    return;
}

static void
process_image                   (const void *           p)
{
    grabbed_frame = (uint8_t *)p;
    fprintf(stderr, "got a frame\n");
}

static int
read_frame(void)
{
        struct v4l2_buffer buf;
        unsigned int i;

        grabbed_frame = (uint8_t *)NULL;
        fprintf(stderr, "in read_frame %d\n", io);
        switch (io) {
        case IO_METHOD_READ:
                if (-1 == read (fd, buffers[0].start, buffers[0].length)) {
                        switch (errno) {
                        case EAGAIN:
                                return 0;

                        case EIO:
                                /* Could ignore EIO, see spec. */

                                /* fall through */

                        default:
                                error_handler2("read");
                        }
                }
                process_image (buffers[0].start);
                break;

        case IO_METHOD_MMAP:
                CLEAR (buf);

                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;

                if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
                        switch (errno) {
                        case EAGAIN:
                                fprintf(stderr, "in read_frame EAGAIN\n");
                                return 0;
                        case EIO:
                                /* Could ignore EIO, see spec. */

                                /* fall through */
                        default:
                                error_handler2 ("VIDIOC_DQBUF");
                        }
                }

                assert (buf.index < n_buffers);

                process_image (buffers[buf.index].start);

                if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                        error_handler2 ("VIDIOC_QBUF");
                break;
        }
        return 1;
}

static void
wait_frame(void)
{
        unsigned int count;

        count = 1;

        while (count-- > 0) {
                for (;;) {
                        fd_set fds;
                        struct timeval tv;
                        int r;

                        FD_ZERO (&fds);
                        FD_SET (fd, &fds);

                        /* Timeout. */
                        tv.tv_sec = 2;
                        tv.tv_usec = 0;

                        r = select (fd + 1, &fds, NULL, NULL, &tv);

                        if (-1 == r) {
                                if (EINTR == errno)
                                        continue;
                                error_handler2 ("select");
                        }
                        if (0 == r) {
                                error_handler2 ("select timeout\n");
                        }
                        if (read_frame ())
                                break;
        
                        /* EAGAIN - continue select loop. */
                }
        }
}

static void
stop_capturing(void)
{
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

                if (-1 == xioctl (fd, VIDIOC_STREAMOFF, &type))
                        error_handler2 ("VIDIOC_STREAMOFF");

                break;
        }
}

static void
start_capturing(void)
{
        unsigned int i;
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR (buf);

                        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory      = V4L2_MEMORY_MMAP;
                        buf.index       = i;

                        if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                                error_handler2 ("VIDIOC_QBUF");
                }
                
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

                if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
                        error_handler2 ("VIDIOC_STREAMON");

                break;

        }
}

static void
uninit_device(void)
{
        unsigned int i;

        switch (io) {
        case IO_METHOD_READ:
                free (buffers[0].start);
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i)
                        if (-1 == munmap (buffers[i].start, buffers[i].length))
                                error_handler2 ("munmap");
                break;
        }

        free (buffers);
}

static void
init_read                       (unsigned int           buffer_size)
{
        buffers = calloc (1, sizeof (*buffers));

        if (!buffers) {
                error_handler2 ("Out of memory\n");
        }

        buffers[0].length = buffer_size;
        buffers[0].start = malloc (buffer_size);

        if (!buffers[0].start) {
                error_handler2 ("Out of memory\n");
        }
}

static void
init_mmap(void)
{
        struct v4l2_requestbuffers req;

        CLEAR (req);

        req.count               = 4;
        req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory              = V4L2_MEMORY_MMAP;

        if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        error_handler2 ("does not support memory mapping");
                } else {
                        error_handler2 ("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) {
                error_handler2 ("Insufficient buffer memory");
        }

        buffers = calloc (req.count, sizeof (*buffers));

        if (!buffers) {
                error_handler2 ( "Out of memory");
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR (buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl (fd, VIDIOC_QUERYBUF, &buf))
                        error_handler2 ("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap (NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        error_handler2 ("mmap");
        }
}

static void
init_device(void)
{
        struct v4l2_capability cap;
        struct v4l2_cropcap cropcap;
        struct v4l2_crop crop;
        struct v4l2_format fmt;
        unsigned int min;

        if (-1 == xioctl (fd, VIDIOC_QUERYCAP, &cap)) {
                if (EINVAL == errno) {
                        error_handler2 ("not a V4L2 device");
                } else {
                        error_handler2 ("VIDIOC_QUERYCAP");
                }
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
                error_handler2 ("not a video capture device");
        }
	printf("driver %s, card %s, bus %s\n", cap.driver, cap.card, cap.bus_info);


	if (cap.capabilities & V4L2_CAP_READWRITE) {
		fprintf (stderr, "%s supports read i/o\n", dev_name);

		io = IO_METHOD_READ;
	} else if (cap.capabilities & V4L2_CAP_STREAMING) {
		fprintf (stderr, "%s supports streaming i/o\n", dev_name);
		io = IO_METHOD_MMAP;
	}

	if (io == 0) {
		error_handler2 ("does not support read i/o or streaming");
	}


        /* Select video input, video standard and tune here. */


        CLEAR (cropcap);

        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (0 == xioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
                crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                crop.c = cropcap.defrect; /* reset to default */

                if (-1 == xioctl (fd, VIDIOC_S_CROP, &crop)) {
                        switch (errno) {
                        case EINVAL:
                                /* Cropping not supported. */
                                break;
                        default:
                                /* Errors ignored. */
                                break;
                        }
                }
        } else {        
                /* Errors ignored. */
        }


        CLEAR (fmt);

        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = 640; 
        fmt.fmt.pix.height      = 480;
        // driver doesn't support RGB24
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
        fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

        // try with RGB packed format first
        if (-1 == xioctl (fd, VIDIOC_S_FMT, &fmt)) {
            // try YUYV format instead
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
            if (-1 == xioctl (fd, VIDIOC_S_FMT, &fmt)) {
                error_handler2 ("VIDIOC_S_FMT");
            }
        }

        switch (fmt.fmt.pix.pixelformat) {
        case V4L2_PIX_FMT_RGB24:
            fprintf(stderr, "packed RGB format\n");
            break;
        case V4L2_PIX_FMT_YUYV:
            fprintf(stderr, "packed YUV422 format\n");
            break;
        }

        /* Note VIDIOC_S_FMT may change width and height. */

        /* Buggy driver paranoia. */
        min = fmt.fmt.pix.width * 2;
        if (fmt.fmt.pix.bytesperline < min)
                fmt.fmt.pix.bytesperline = min;
        min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
        if (fmt.fmt.pix.sizeimage < min)
                fmt.fmt.pix.sizeimage = min;

        switch (io) {
        case IO_METHOD_READ:
                init_read (fmt.fmt.pix.sizeimage);
                break;

        case IO_METHOD_MMAP:
                init_mmap ();
                break;

        }
}

static void
close_device(void)
{
        if (fd > 0)
            if (-1 == close (fd))
                    error_handler2 ("close");

        fd = -1;
}

static void
open_device(void)
{
        struct stat st; 

        if (-1 == stat (dev_name, &st)) {
                error_handler1 ("Cannot stat device");
        }

        if (!S_ISCHR (st.st_mode)) {
                error_handler1 ("No such device");
        }

        fd = open (dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
                error_handler2 ("Cannot open device");
        }
}

int handler2_active = 0;

static void
error_handler1(const char *s)
{
    fprintf(stderr, "** error1: %s\n", s);
}

// come here on system call error
//  there's some chatter that says mexErrMsgTxt is faulty under Linux..
static void
error_handler2(const char *s)
{
        char    errmsg[1024];

    fprintf(stderr, "** error2: %s\n", s);
    return;
        // save details of the error
        //sprintf(errmsg, "%s (errno=%d)", strerror(errno), errno);

        if (handler2_active == 0) {
            // handling an error, try and wrap up
            handler2_active = 1;    // avoid recursion
            stop_capturing();
            uninit_device();
        }
        handler2_active = 0;

        close(fd);
        fd = -1;
        /*
        mexErrMsgIdAndTxt("MVTB:framegrabber:V4L2",
            "framegrabber_v4l2: %s %s", s, errmsg);
            */
}

static int
xioctl                          (int                    fd,
                                 int                    request,
                                 void *                 arg)
{
        int r;

        do r = ioctl (fd, request, arg);
        while (-1 == r && EINTR == errno);

        return r;
}

static int
getInt(const mxArray *p)
{
    return (int) mxGetScalar(p);
}
