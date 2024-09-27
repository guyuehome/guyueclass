# -*- coding: utf-8 -*-
"""Code for visualizing data in sim via python.

Author: Andrew Hundt <ATHundt@gmail.com>

License: Apache v2 https://www.apache.org/licenses/LICENSE-2.0
"""
import os
import errno
import traceback

import numpy as np
import six  # compatibility between python 2 + 3 = six
import matplotlib.pyplot as plt

try:
    import sim as sim
except Exception as e:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in PYTHONPATH folder relative to this file,')
    print ('or appropriately adjust the file "sim.py. Also follow the"')
    print ('ReadMe.txt in the sim remote API folder')
    print ('--------------------------------------------------------------')
    print ('')
    raise e

import tensorflow as tf

from tensorflow.python.platform import flags
from tensorflow.python.platform import gfile
from tensorflow.python.ops import data_flow_ops
from ply import write_xyz_rgb_as_ply
from PIL import Image

# progress bars https://github.com/tqdm/tqdm
# import tqdm without enforcing it as a dependency
try:
    from tqdm import tqdm
except ImportError:

    def tqdm(*args, **kwargs):
        if args:
            return args[0]
        return kwargs.get('iterable', None)

from depth_image_encoding import ClipFloatValues
from depth_image_encoding import FloatArrayToRgbImage
from depth_image_encoding import FloatArrayToRawRGB
from skimage.transform import resize
from skimage import img_as_ubyte
from skimage import img_as_uint
from skimage.color import grey2rgb

try:
    import eigen  # https://github.com/jrl-umi3218/Eigen3ToPython
    import sva  # https://github.com/jrl-umi3218/SpaceVecAlg
except ImportError:
    print('eigen and sva python modules are not available. To install run the script at:'
          'https://github.com/ahundt/robotics_setup/blob/master/robotics_tasks.sh'
          'or follow the instructions at https://github.com/jrl-umi3218/Eigen3ToPython'
          'and https://github.com/jrl-umi3218/SpaceVecAlg. '
          'When you build the modules make sure python bindings are enabled.')

tf.flags.DEFINE_string('csimVisualizeDepthFormat', 'csim_depth_encoded_rgb',
                       """ Controls how Depth images are displayed. Options are:
                           None: Do not modify the data and display it as-is for rgb input data (not working properly for float depth).
                           'depth_rgb': convert a floating point depth image to a straight 0-255 encoding of depths less than 3m
                           'depth_encoded_rgb': convert a floating point depth image to the rgb encoding used by
                               the google brain robot data grasp dataset's raw png depth image encoding,
                               see https://sites.google.com/site/brainrobotdata/home/depth-image-encoding.
                           'sim': add a sim prefix to any of the above commands to
                               rotate image by 180 degrees, flip left over right, then invert the color channels
                               after the initial conversion step.
                               This is due to a problem where CoppeliaSim seems to display images differently.
                               Examples include 'csim_depth_rgb' and 'csim_depth_encoded_rgb',
                               see http://forum.coppeliarobotics.com/viewtopic.php?f=9&t=737&p=27805#p27805.
                       """)
tf.flags.DEFINE_string('csimVisualizeRGBFormat', 'csim_rgb',
                       """  Controls how images are displayed. Options are:
                        None: Do not modify the data and display it as-is for rgb input data (not working properly for float depth).
                        'depth_rgb': convert a floating point depth image to a straight 0-255 encoding of depths less than 3m
                        'depth_encoded_rgb': convert a floating point depth image to the rgb encoding used by
                            the google brain robot data grasp dataset's raw png depth image encoding,
                            see https://sites.google.com/site/brainrobotdata/home/depth-image-encoding.
                        'sim': add a sim prefix to any of the above commands to
                            rotate image by 180 degrees, flip left over right, then invert the color channels
                            after the initial conversion step.
                            This is due to a problem where CoppeliaSim seems to display images differently.
                            Examples include 'csim_depth_rgb' and 'csim_depth_encoded_rgb',
                            see http://forum.coppeliarobotics.com/viewtopic.php?f=9&t=737&p=27805#p27805.
                       """)

# the following line is needed for tf versions before 1.5
# flags.FLAGS._parse_flags()
FLAGS = flags.FLAGS


def depth_image_to_point_cloud(depth, intrinsics_matrix, dtype=np.float32, verbose=0):
    """Depth images become an XYZ point cloud in the camera frame with shape (depth.shape[0], depth.shape[1], 3).

    Transform a depth image into a point cloud in the camera frame with one point for each
    pixel in the image, using the camera transform for a camera
    centred at cx, cy with field of view fx, fy.

    Based on:
    https://github.com/tensorflow/models/blob/master/research/cognitive_mapping_and_planning/src/depth_utils.py
    https://codereview.stackexchange.com/a/84990/10101

    also see grasp_geometry_tf.depth_image_to_point_cloud().

    # Arguments

      depth: is a 2-D ndarray with shape (rows, cols) containing
          32bit floating point depths in meters. The result is a 3-D array with
          shape (rows, cols, 3). Pixels with invalid depth in the input have
          NaN or 0 for the z-coordinate in the result.

      intrinsics_matrix: 3x3 matrix for projecting depth values to z values
      in the point cloud frame. http://ksimek.github.io/2013/08/13/intrinsic/
      In this case x0, y0 are at index [2, 0] and [2, 1], respectively.

      transform: 4x4 Rt matrix for rotating and translating the point cloud
    """
    fy = intrinsics_matrix[1, 1]
    fx = intrinsics_matrix[0, 0]
    # center of image y coordinate
    center_y = intrinsics_matrix[2, 1]
    # center of image x coordinate
    center_x = intrinsics_matrix[2, 0]
    depth = np.squeeze(depth)
    y_range, x_range = depth.shape

    y, x = np.meshgrid(np.arange(y_range),
                       np.arange(x_range),
                       indexing='ij')
    assert y.size == x.size and y.size == depth.size
    x = x.flatten()
    y = y.flatten()
    depth = depth.flatten()

    X = (x - center_x) * depth / fx
    Y = (y - center_y) * depth / fy

    assert X.size == Y.size and X.size == depth.size
    assert X.shape == Y.shape and X.shape == depth.shape

    if verbose > 0:
        print('X np: ', X.shape)
        print('Y np: ', Y.shape)
        print('depth np: ', depth.shape)
    XYZ = np.column_stack([X, Y, depth])
    assert XYZ.shape == (y_range * x_range, 3)
    if verbose > 0:
        print('XYZ pre reshape np: ', XYZ.shape)
    XYZ = XYZ.reshape((y_range, x_range, 3))

    return XYZ.astype(dtype)


def csimPrint(client_id, message):
    """Print a message in both the python command line and on the CoppeliaSim Statusbar.

    The Statusbar is the white command line output on the bottom of the CoppeliaSim GUI window.
    """
    sim.simxAddStatusbarMessage(client_id, message, sim.simx_opmode_oneshot)
    print(message)


def create_dummy(client_id, display_name, transform=None, parent_handle=-1, debug=FLAGS.csimDebugMode, operation_mode=sim.simx_opmode_blocking):
    """Create a dummy object in the simulation

    # Arguments

        transform_display_name: name string to use for the object in the sim scene
        transform: 3 cartesian (x, y, z) and 4 quaternion (x, y, z, w) elements, same as sim
        parent_handle: -1 is the world frame, any other int should be a sim object handle
    """
    if transform is None:
        transform = np.array([0., 0., 0., 0., 0., 0., 1.])
    # 2. Now create a dummy object at coordinate 0.1,0.2,0.3 with name 'MyDummyName':
    empty_buffer = bytearray()
    res, ret_ints, ret_floats, ret_strings, ret_buffer = sim.simxCallScriptFunction(
        client_id,
        'remoteApiCommandServer',
        sim.sim_scripttype_childscript,
        'createDummy_function',
        [parent_handle],
        transform,
        [display_name],
        empty_buffer,
        operation_mode)
    if res == sim.simx_return_ok:
        # display the reply from CoppeliaSim (in this case, the handle of the created dummy)
        if debug is not None and 'print_transform' in debug:
            print ('Dummy name:', display_name, ' handle: ', ret_ints[0], ' transform: ', transform)
    else:
        print('create_dummy remote function call failed.')
        print(''.join(traceback.format_stack()))
        return -1
    return ret_ints[0]


def setPose(client_id, display_name, transform=None, parent_handle=-1):
    """Set the pose of an object in the simulation

    # Arguments

        transform_display_name: name string to use for the object in the sim scene
        transform: 3 cartesian (x, y, z) and 4 quaternion (x, y, z, w) elements, same as sim
        parent_handle: -1 is the world frame, any other int should be a sim object handle
    """
    if transform is None:
        transform = np.array([0., 0., 0., 0., 0., 0., 1.])
    # 2. Now create a dummy object at coordinate 0.1,0.2,0.3 with name 'MyDummyName':
    empty_buffer = bytearray()
    res, ret_ints, ret_floats, ret_strings, ret_buffer = sim.simxCallScriptFunction(
        client_id,
        'remoteApiCommandServer',
        sim.sim_scripttype_childscript,
        'createDummy_function',
        [parent_handle],
        transform,
        [display_name],
        empty_buffer,
        sim.simx_opmode_blocking)
    if res == sim.simx_return_ok:
        # display the reply from CoppeliaSim (in this case, the handle of the created dummy)
        print ('SetPose object name:', display_name, ' handle: ', ret_ints[0], ' transform: ', transform)
    else:
        print('setPose remote function call failed.')
        print(''.join(traceback.format_stack()))
        return -1
    return ret_ints[0]


def set_vision_sensor_image(client_id, display_name, image, convert=None, scale_factor=256000.0, operation_mode=sim.simx_opmode_oneshot_wait):
    """Display vision sensor image data in a CoppeliaSim simulation.

    [CoppeliaSim Vision Sensors](http://www.coppeliarobotics.com/helpFiles/en/visionSensors.htm)
    [simSetVisionSensorImage](http://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm#simSetVisionSensorImage)

    # Arguments

    display_name: the string display name of the sensor object in the CoppeliaSim scene
    image: an rgb char array containing an image
    convert: Controls how images are displayed. Options are:
            None: Do not modify the data and display it as-is for rgb input data (not working properly for float depth).
            'depth_rgb': convert a floating point depth image to a straight 0-255 encoding of depths less than 3m
            'depth_encoded_rgb': convert a floating point depth image to the rgb encoding used by
                the google brain robot data grasp dataset's raw png depth image encoding,
                see https://sites.google.com/site/brainrobotdata/home/depth-image-encoding.
            'sim': add a sim prefix to any of the above commands to
                rotate image by 180 degrees, flip left over right, then invert the color channels
                after the initial conversion step.
                This is due to a problem where CoppeliaSim seems to display images differently.
                Examples include 'csim_depth_rgb' and 'csim_depth_encoded_rgb',
                see http://www.forum.coppeliarobotics.com/viewtopic.php?f=9&t=737&p=27805#p27805.
    """
    strings = [display_name]
    parent_handle = -1

    # TODO(ahundt) support is_greyscale True again
    is_greyscale = 0
    csim_conversion = False
    if convert is not None:
        csim_conversion = 'sim' in convert

        if 'depth_encoded_rgb' in convert:
            image = np.array(FloatArrayToRgbImage(image, scale_factor=scale_factor, drop_blue=False), dtype=np.uint8)
        elif 'depth_rgb' in convert:

            image = img_as_uint(image)

        elif not csim_conversion:
            raise ValueError('set_vision_sensor_image() convert parameter must be one of `depth_encoded_rgb`, `depth_rgb`, or None'
                             'with the optional addition of the word `sim` to rotate 180, flip left right, then invert colors.')

    if csim_conversion:
        # rotate 180 degrees, flip left over right, then invert the colors
        image = np.array(256 - np.fliplr(np.rot90(image, 2)), dtype=np.uint8)

    if np.issubdtype(image.dtype, np.integer):
        is_float = 0
        floats = []
        color_buffer = bytearray(image.flatten().tobytes())
        color_size = image.size
        num_floats = 0
    else:
        is_float = 1
        floats = [image]
        color_buffer = bytearray()
        num_floats = image.size
        color_size = 0

    cloud_handle = -1
    res, ret_ints, ret_floats, ret_strings, ret_buffer = sim.simxCallScriptFunction(
        client_id,
        'remoteApiCommandServer',
        sim.sim_scripttype_childscript,
        'setVisionSensorImage_function',
        [parent_handle, num_floats, is_greyscale, color_size],  # int params
        np.append(floats, []),  # float params
        strings,  # string params
        # byte buffer params
        color_buffer,
        operation_mode)
    if res == sim.simx_return_ok:
        print ('point cloud handle: ', ret_ints[0])  # display the reply from CoppeliaSim (in this case, the handle of the created dummy)
        # set the transform for the point cloud
        return ret_ints[0]
    else:
        print('insertPointCloud_function remote function call failed.')
        print(''.join(traceback.format_stack()))
        return res


def create_point_cloud(client_id, display_name, transform=None, point_cloud=None, depth_image=None, color_image=None,
                       camera_intrinsics_matrix=None, parent_handle=-1, clear=True,
                       max_voxel_size=0.01, max_point_count_per_voxel=10, point_size=10, options=8,
                       rgb_sensor_display_name=None, depth_sensor_display_name=None, convert_depth=FLAGS.csimVisualizeDepthFormat,
                       convert_rgb=FLAGS.csimVisualizeRGBFormat, save_ply_path=None, rgb_display_mode='vision_sensor'):
    """Create a point cloud object in the simulation, plus optionally render the depth and rgb images.

    # Arguments

        display_name: name string to use for the object in the sim scene
        depth_image: A depth image of size [width, height, 3]
        transform: [x, y, z, qw, qx, qy, qz] with 3 cartesian (x, y, z) and 4 quaternion (qx, qy, qz, qw) elements, same as sim
            This transform is from the parent handle to the point cloud base
        parent_handle: -1 is the world frame, any other int should be a sim object handle
        clear: clear the point cloud if it already exists with the provided display name
        maxVoxelSize: the maximum size of the octree voxels containing points
        maxPtCntPerVoxel: the maximum number of points allowed in a same octree voxel
        options: bit-coded:
        bit0 set (1): points have random colors
        bit1 set (2): show octree structure
        bit2 set (4): reserved. keep unset
        bit3 set (8): do not use an octree structure. When enabled, point cloud operations are limited, and point clouds will not be collidable, measurable or detectable anymore, but adding points will be much faster
        bit4 set (16): color is emissive
        pointSize: the size of the points, in pixels
        reserved: reserved for future extensions. Set to NULL
        save_ply_path: save out a ply file with the point cloud data
        point_cloud: optional XYZ point cloud of size [width, height, 3], will be generated if not provided.
        convert_rgb: Controls how images are displayed. Options are:
            None: Do not modify the data and display it as-is for rgb input data (not working properly for float depth).
            'depth_rgb': convert a floating point depth image to a straight 0-255 encoding of depths less than 3m
            'depth_encoded_rgb': convert a floating point depth image to the rgb encoding used by
                the google brain robot data grasp dataset's raw png depth image encoding,
                see https://sites.google.com/site/brainrobotdata/home/depth-image-encoding.
            'sim': add a sim prefix to any of the above commands to
                rotate image by 180 degrees, flip left over right, then invert the color channels
                after the initial conversion step.
                This is due to a problem where CoppeliaSim seems to display images differently.
                Examples include 'csim_depth_rgb' and 'csim_depth_encoded_rgb',
                see http://www.forum.coppeliarobotics.com/viewtopic.php?f=9&t=737&p=27805#p27805.
        rgb_display_mode: Options help with working around quirks in input image data's layout.
            'point_cloud' to display the image when the point cloud is being generated.
            'vision_sensor' to make a separate call go the vison sensor display function.
    """
    if transform is None:
        transform = np.array([0., 0., 0., 0., 0., 0., 1.])

    if point_cloud is None:
        point_cloud = depth_image_to_point_cloud(depth_image, camera_intrinsics_matrix)
        point_cloud = point_cloud.reshape([point_cloud.size/3, 3])

    # show the depth sensor image
    if depth_sensor_display_name is not None and depth_image is not None:
        # matplotlib.image.imsave(display_name + depth_sensor_display_name + '_norotfliplr.png', depth_image)
        # rotate 180, flip left over right then invert the image colors for display in CoppeliaSim
        # depth_image = np.fliplr(np.rot90(depth_image, 2))
        # matplotlib.image.imsave(display_name + depth_sensor_display_name + '_rot90fliplr.png', depth_image)
        set_vision_sensor_image(client_id, depth_sensor_display_name, depth_image, convert=convert_depth)

    # show the rgb sensor image, this overwrites the rgb display
    # done in insertPointCloud_function, which is buggy
    if rgb_sensor_display_name is not None and color_image is not None and rgb_display_mode == 'vision_sensor':
        # matplotlib.image.imsave(display_name + rgb_sensor_display_name + '_norotfliplr.png', color_image)
        # rotate 180, flip left over right then invert the image colors for display in CoppeliaSim
        # matplotlib.image.imsave(display_name + rgb_sensor_display_name + '_rot90fliplr.png', color_image)
        set_vision_sensor_image(client_id, rgb_sensor_display_name, color_image, convert=convert_rgb)

    # Save out Point cloud
    if save_ply_path is not None:
        write_xyz_rgb_as_ply(point_cloud, color_image, save_ply_path)

    # color_buffer is initially empty
    color_buffer = bytearray()
    strings = [display_name]
    if rgb_sensor_display_name is not None and rgb_display_mode == 'point_cloud':
        strings = [display_name, rgb_sensor_display_name]

    transform_entries = 7
    if clear:
        clear = 1
    else:
        clear = 0

    cloud_handle = -1
    # Create the point cloud if it does not exist, or retrieve the handle if it does
    res, ret_ints, ret_floats, ret_strings, ret_buffer = sim.simxCallScriptFunction(
        client_id,
        'remoteApiCommandServer',
        sim.sim_scripttype_childscript,
        'createPointCloud_function',
        # int params
        [parent_handle, transform_entries, point_cloud.size, cloud_handle, clear, max_point_count_per_voxel, options, point_size],
        # float params
        [max_voxel_size],
        # string params
        strings,
        # byte buffer params
        color_buffer,
        sim.simx_opmode_blocking)

    setPose(client_id, display_name, transform, parent_handle)

    if res == sim.simx_return_ok:
        cloud_handle = ret_ints[0]

        # convert the rgb values to a string
        color_size = 0
        if color_image is not None:
            # see simInsertPointsIntoPointCloud() in sim documentation
            # 3 indicates the cloud should be in the parent frame, and color is enabled
            # bit 2 is 1 so each point is colored
            simInsertPointsIntoPointCloudOptions = 3
            # color_buffer = bytearray(np.fliplr(np.rot90(color_image, 3)).flatten().tobytes())
            color_buffer = bytearray(color_image.flatten().tobytes())
            color_size = color_image.size
        else:
            simInsertPointsIntoPointCloudOptions = 1

        # Actually transfer the point cloud
        res, ret_ints, ret_floats, ret_strings, ret_buffer = sim.simxCallScriptFunction(
            client_id,
            'remoteApiCommandServer',
            sim.sim_scripttype_childscript,
            'insertPointCloud_function',
            [parent_handle, transform_entries, point_cloud.size, cloud_handle, color_size, simInsertPointsIntoPointCloudOptions],
            np.append(point_cloud, []),
            strings,
            color_buffer,
            sim.simx_opmode_blocking)

        if res == sim.simx_return_ok:
            print ('point cloud handle: ', ret_ints[0])  # display the reply from CoppeliaSim (in this case, the handle of the created dummy)
            # set the transform for the point cloud
            return ret_ints[0]
        else:
            print('insertPointCloud_function remote function call failed.')
            print(''.join(traceback.format_stack()))
            return res

    else:
        print('createPointCloud_function remote function call failed')
        print(''.join(traceback.format_stack()))
        return res


def drawLines(client_id, display_name, lines, parent_handle=-1, transform=None, debug=FLAGS.csimDebugMode, operation_mode=sim.simx_opmode_blocking):
    """Create a line in the simulation.

    Note that there are currently some quirks with this function. Only one line is accepted,
    and sometimes CoppeliaSim fails to delete the object correctly and lines will fail to draw.
    In that case you need to close and restart CoppeliaSim.

    # Arguments

        transform_display_name: name string to use for the object in the sim scene
        transform: 3 cartesian (x, y, z) and 4 quaternion (x, y, z, w) elements, same as sim
        parent_handle: -1 is the world frame, any other int should be a sim object handle
        lines: array of line definitions using two endpoints (x0, y0, z0, x1, y1, z1).
            Multiple lines can be defined but there should be 6 entries (two points) per line.
    """
    # 2. Now create a dummy object at coordinate 0.1,0.2,0.3 with name 'MyDummyName':
    empty_buffer = bytearray()
    res, ret_ints, ret_floats, ret_strings, ret_buffer = sim.simxCallScriptFunction(
        client_id,
        'remoteApiCommandServer',
        sim.sim_scripttype_childscript,
        'addDrawingObject_function',
        [parent_handle, int(lines.size/6)],
        # np.append(transform, lines),
        lines,
        [display_name],
        empty_buffer,
        operation_mode)
    if res == sim.simx_return_ok:
        # display the reply from CoppeliaSim (in this case, the handle of the created dummy)
        if debug is not None and 'print_drawLines' in debug:
            print ('drawLines name:', display_name, ' handle: ', ret_ints[0], ' transform: ', transform)

        if transform is not None:
            # set the transform for the point cloud
            setPose(client_id, display_name, transform, parent_handle)
    else:
        print('drawLines remote function call failed.')
        print(''.join(traceback.format_stack()))
        return -1
    return ret_ints[0]


def restore_cropped(cropped_image, crop_size, crop_offset, full_size):
    """ Restore cropped_image to full size image with zero padding
        First scale image back to crop_size, then padding
    """

    cropped_image = np.squeeze(cropped_image)
    restored = np.zeros((full_size[0], full_size[1]), dtype=cropped_image.dtype)
    scaled_crop = resize(cropped_image, (crop_size[0], crop_size[1]))
    restored[crop_offset[0]:crop_offset[0]+crop_size[0],
             crop_offset[1]:crop_offset[1]+crop_size[1]] = scaled_crop

    return restored
