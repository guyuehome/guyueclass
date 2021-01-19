# Industrial Core

## ROS Distro Support

|         | Indigo | Jade | Kinetic |
|:-------:|:------:|:----:|:-------:|
| Branch  | [`indigo-devel`](https://github.com/ros-industrial/industrial_core/tree/indigo-devel) | [`jade-devel`](https://github.com/ros-industrial/industrial_core/tree/jade-devel) | [`kinetic-devel`](https://github.com/ros-industrial/industrial_core/tree/kinetic-devel) |
| Status  |  supported | supported |  supported |
| Version | [version](http://repositories.ros.org/status_page/ros_indigo_default.html?q=industrial_core) | [version](http://repositories.ros.org/status_page/ros_jade_default.html?q=industrial_core) | [version](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=industrial_core) |

## Travis - Continuous Integration

Status: [![Build Status](https://travis-ci.org/ros-industrial/industrial_core.svg?branch=kinetic-devel)](https://travis-ci.org/ros-industrial/industrial_core)

## ROS Buildfarm

|         | Indigo Source | Indigo Debian | Jade Source | Jade Debian |  Kinetic Source  |  Kinetic Debian |
|:-------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|
| industrial_core | [![not released](http://build.ros.org/buildStatus/icon?job=Isrc_uT__industrial_core__ubuntu_trusty__source)](http://build.ros.org/view/Isrc_uT/job/Isrc_uT__industrial_core__ubuntu_trusty__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__industrial_core__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Ibin_uT64/job/Ibin_uT64__industrial_core__ubuntu_trusty_amd64__binary/) | [![not released](http://build.ros.org/buildStatus/icon?job=Jsrc_uT__industrial_core__ubuntu_trusty__source)](http://build.ros.org/view/Jsrc_uT/job/Jsrc_uT__industrial_core__ubuntu_trusty__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__industrial_core__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Jbin_uT64/job/Jbin_uT64__industrial_core__ubuntu_trusty_amd64__binary/) | [![not released](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__industrial_core__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__industrial_core__ubuntu_xenial__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__industrial_core__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__industrial_core__ubuntu_xenial_amd64__binary/) |


[ROS-Industrial][] core meta-package. See the [ROS wiki][] page for more
information.

## Contents

Branch naming follows the ROS distribution they are compatible with. `-devel`
branches may be unstable. Releases are made from the distribution branches
(`hydro`, `indigo`, `jade`).

Older releases may be found in the old ROS-Industrial [subversion repository][].


[ROS-Industrial]: http://wiki.ros.org/Industrial
[ROS wiki]: http://wiki.ros.org/industrial_core
[subversion repository]: https://github.com/ros-industrial/swri-ros-pkg

## Docker 

Industrial Core is also available as a Docker image from the [ROS-Industrial Docker Hub](https://hub.docker.com/u/rosindustrial).

Example usage:
```
docker run -it --rm rosindustrial/core:kinetic rosmsg show industrial_msgs/RobotStatus
```
