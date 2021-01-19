# Defines a 3D classification result.
#
# This result does not contain any position information. It is designed for
#   classifiers, which simply provide probabilities given a source image.

Header header

# Class probabilities
ObjectHypothesis[] results

# The 3D data that generated these results (i.e. region proposal cropped out of
#   the image). Not required for all detectors, so it may be empty.
sensor_msgs/PointCloud2 source_cloud