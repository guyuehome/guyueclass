#!/bin/bash
#
# Copyright 2014-2021 The MathWorks, Inc.

ARCHIVE="$1"
CATKIN_WS="$2"

catkinWorkspaceHelp() {
   echo ""
   echo "You can create a Catkin workspace as follows:"
   echo "  mkdir -p ~/catkin_ws/src"
   echo "  cd ~/catkin_ws/src"
   echo "  catkin_init_workspace"
}


commandUsage() {
   echo "Usage: $(basename $0) ARCHIVE_NAME... CATKIN_WS..."
   echo "Extract and build a C++ ROS node generated from a Simulink model."
   echo "ARCHIVE_NAME is the name of the TGZ file generated from the Simulink model."
   echo "CATKIN_WS is the full path to your ROS Catkin workspace."
   echo ""
   echo "Example:"
   echo "  ./$(basename $0) simulinkmodel.tgz ~/catkin_ws"   
}


fullUsage() {
   commandUsage
   catkinWorkspaceHelp
   exit
}


toLowerCase() {
   echo $1 | tr '[A-Z]' '[a-z]'
}

trim() {
    local var="$*"
    # remove leading whitespace characters
    var="${var#"${var%%[![:space:]]*}"}"
    # remove trailing whitespace characters
    var="${var%"${var##*[![:space:]]}"}"
    echo -n "$var"
}

if [ -z "$1" ] || ([ ! -z "$1" ] && [ "$1" = "-h" ] || [ "$1" = "--help" ]) ; then
   fullUsage
   exit 0
fi

if [ ! $# -eq 2 ] ; then
   echo "Expected two input arguments. Got $#."
   fullUsage
   exit 1
fi

# Check Catkin workspace
if [ ! -d "$CATKIN_WS" ] ; then
   echo "The catkin workspace directory, "$CATKIN_WS", does not exist."
   echo "Enter a valid catkin workspace directory."
   catkinWorkspaceHelp
   exit 1
fi

# Sanity check for CATKIN workspace
if [ ! -f "$CATKIN_WS"/src/CMakeLists.txt ] || [ ! -f "$CATKIN_WS"/devel/setup.bash ] ; then
   echo "The Catkin workspace directory, "$CATKIN_WS", is not a valid Catkin workspace."
   echo "Enter a valid Catkin workspace directory."
   catkinWorkspaceHelp
   exit 1
fi

# Check Simulink archive
if [ ! -f "$ARCHIVE" ] ; then
   echo "The archive, "$ARCHIVE", does not exist."
   echo "Enter a valid Simulink model archive (.tgz file)."
   echo ""
   commandUsage
   exit 1
fi

# Enforce that $ARCHIVE ends with .tgz, since the model 
# name is derived by stripping off the .tgz extension
if [ ${ARCHIVE: -4} != ".tgz" ] ; then
   echo "The archive, "$ARCHIVE", does not have a .tgz extension."
   echo "Enter a valid Simulink model archive (.tgz file)."
   echo ""   
   commandUsage
   exit 1
fi

# Check if $ARCHIVE is a valid zip file
gzip -t "$ARCHIVE" 2> /dev/null
VALID_ZIP=$?
if [ $VALID_ZIP -ne 0 ] ; then
   echo "The archive, "$ARCHIVE", is not a valid .tgz (tar zip) file."
   echo ""
   commandUsage
   exit 1   
fi

# Check for one of the standard files generated from Simulink or MATLAB
# (rosnodeinterface.cpp or main.cpp or <modelname>_ctrlr_host.cpp)
tar ztf "$ARCHIVE" | grep -q -E -- 'rosnodeinterface|main|*_ctrlr_host'.cpp 2> /dev/null
VALID_ARCHIVE=$?

if [ $VALID_ARCHIVE -ne 0 ] ; then
   echo "The archive, "$ARCHIVE", is not a valid Simulink model archive (.tgz file)."
   echo ""
   commandUsage
   exit 1
fi

# $ARCHIVE appears to be valid.
# Extract and build it

ARCHIVE_DIR=$(dirname "$ARCHIVE")
ARCHIVE_BASE=$(basename "$ARCHIVE" .tgz)

PKGNAME=$(toLowerCase $ARCHIVE_BASE)
PROJECT_DIR="$CATKIN_WS/src"

echo "Catkin project directory: $PROJECT_DIR"

# Extract files of the main archive (top-level model) to catkin project directory
mkdir -p "$PROJECT_DIR"
rm -fr "$PROJECT_DIR/$PKGNAME"
tar -C "$PROJECT_DIR" -xf "$ARCHIVE"

# Extract model reference archives (if needed)
MODEL_REF_LIST="$ARCHIVE_DIR/$ARCHIVE_BASE"ModelRefs.txt
if [ -f "$MODEL_REF_LIST" ] ; then
    while IFS= read -r mdlRefArchive
    do
        # Trim whitespaces and newlines to account for OS-specific text
        mdlRefArchive=$(trim "$ARCHIVE_DIR/$mdlRefArchive")

        echo "Extracting model reference archive $mdlRefArchive"        

        # Extract archive if it exists
        if [ -f "$mdlRefArchive" ] ; then
            # Create folder
            MDLREF_DIR="$PROJECT_DIR/$(toLowerCase $(basename "$mdlRefArchive" .tgz))"
            rm -fr "$MDLREF_DIR"

            # Extract archive into created folder
            tar -C "$PROJECT_DIR" -xf "$mdlRefArchive"
        fi
    done < "$MODEL_REF_LIST"
fi

# Ensure that catkin_make will rebuild the executable
touch  "$PROJECT_DIR/$PKGNAME"/src/*.cpp

# Build the Simulink model as a catkin project
CURR_DIR=`pwd`
cd "$CATKIN_WS"
catkin_make --pkg "$PKGNAME"
cd "$CURR_DIR"

exit 0
