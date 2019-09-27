#!/bin/bash

# Full photo->3d point cloud->mesh workflow script
#
# Farran Rebbeck
# 2015-12-15
#

# Assumes both OpenMVG and OpenMVS build instructions are followed
# https://raw.githubusercontent.com/openMVG/openMVG/master/BUILD
# https://github.com/cdcseacave/openMVS/wiki/Building

# --- Variables and Directories set up here ---

# MAIN VARIABLES - DEPENDANT ON SYSTEM
OPENMVG_BUILD_DIR=/home/alessandro/dev/openMVG_Build
OPENMVG_INSTALL_DIR=/home/alessandro/openMVG_Build/Linux-x86_64-RELEASE
OPENMVS_BUILD_DIR=/home/alessandro/openMVS_build
PYTHON_LOC=`which python`
INPUT_DIR=$1
DTSTAMP=`date +%y.%m.%d-%H.%M.%S`
#DEFAULT_OUTPUT_DIR=out-left-$CFEATURE_TYPE-$DTSTAMP
#MATCHES_DIR=$DEFAULT_OUTPUT_DIR/matches
#RECONSTRUCT_DIR=$DEFAULT_OUTPUT_DIR/recseq
CAMERA_SENSOR_DEFS=/home/alessandro/openMVG/src/openMVG/exif/sensor_width_database/sensor_width_camera_database.txt

# --- PROCESSING OPTIONS ---
# OVERRIDE INTRINSICS (For images that wont process properly)
FOCAL='24'
#INTRINSICS='"f;0;ppx;0;f;ppy;0;0;1"'

# THREADS
THREADS=4

if [ "$INPUT_DIR" == "right" ]; then
  INTRINSICS="868;0;325;0;868;250;0;0;1"
elif [ "$INPUT_DIR" == "left" ]; then
  INTRINSICS="857;0;313;0;857;241;0;0;1"
elif [ "$INPUT_DIR" == "test4" ]; then
  INTRINSICS="838;0;352;0;911;302;0;0;1"
fi
# phonatom k matrix [838.349405, 0.0, 352.957933, 0.0, 911.904209, 302.195598, 0.0, 0.0, 1.0
# gigital 3279.710472019044, 0.0, 585.1559724440893, 0.0, 3307.399632573944, 704.1754224285519, 0.0, 0.0, 1.0
# FEATURES
# Feature selection, can be SIFT/SIFT_ANATOMY/AKAZE_FLOAT/AKAZE_MLDB
echo -e "Feature selection:\n 1)SIFT \n 2)SIFT_ANATOMY \n 3)AKAZE_FLOAT \n 4)AKAZE_MLDB"
read -p "> " CFEATURE_TYPE
case $CFEATURE_TYPE in
1)
CFEATURE_TYPE=SIFT
;;
2)
CFEATURE_TYPE=SIFT_ANATOMY
;;
3)
CFEATURE_TYPE=AKAZE_FLOAT
;;
4)
CFEATURE_TYPE=AKAZE_MLDB
;;
*)
CFEATURE_TYPE=AKAZE_FLOAT
;;
esac


# Image Describer setup, can be NORMAL/HIGH/ULTRA
echo -e "Which describer standard to use:\n 1)NORMAL \n 2)HIGH \n 3)ULTRA (Takes a long time)"
read -p "> " CDESC_TYPE
case $CDESC_TYPE in
1)
CDESC_TYPE=NORMAL
;;
2)
CDESC_TYPE=HIGH
;;
3)
CDESC_TYPE=ULTRA
;;
*)
CDESC_TYPE=NORMAL
;;
esac


# MATCHING
# Nearest matching method, can be AUTO/BRUTEFORCEL2/ANNL2/CASCADEHASHINGL2/FASTCASCADEHASHINGL2/bin BRUTEFORCEHAMMING
echo -e "Matching method to use:\n 1)AUTO \n 2)BRUTEFORCEL2 \n 3)ANNL2 \n 4)CASCADEHASHINGL2 \n 5)FASTCASCADEHASHINGL2 \n 6)BRUTEFORCEHAMMING"
read -p "> " MMETHOD
case $MMETHOD in
1)
MMETHOD=AUTO
;;
2)
MMETHOD=BRUTEFORCEL2
;;
3)
MMETHOD=ANNL2
;;
4)
MMETHOD=CASCADEHASHINGL2
;;
5)
MMETHOD=FASTCASCADEHASHINGL2
;;
6)
MMETHOD=BRUTEFORCEHAMMING
;;
*)
MMETHOD=AUTO
;;
esac

# Ratio
echo -e "Enter a matching ratio, default 0.6, 0.8 recommended:"
read -p "> " MRATIO
if [ -z $MRATIO ]
then
MRATIO=0.6
fi


# Robust filtering (Wont affect MVS)
echo -e "Enter a filtering setting for robust filtering, default 4, for big messy sets, use <1:"
read -p "> " ROBUSTFILTER
if [ -z $ROBUSTFILTER ]
then
ROBUSTFILTER=1 #UPDATE DA 0.5
fi

DEFAULT_OUTPUT_DIR=out-$INPUT_DIR-$CFEATURE_TYPE-$DTSTAMP
MATCHES_DIR=$DEFAULT_OUTPUT_DIR/matches
RECONSTRUCT_DIR=$DEFAULT_OUTPUT_DIR/recseq

# Create working directories
if [ ! -d $DEFAULT_OUTPUT_DIR ]
then
    mkdir $DEFAULT_OUTPUT_DIR
fi

if [ ! -d $MATCHES_DIR ]
then
    mkdir $MATCHES_DIR
fi

if [ ! -d $RECONSTRUCT_DIR ]
then
    mkdir $RECONSTRUCT_DIR
fi


INTRINSICS_R="868;0;325;0;868;250;0;0;1" #RIGHT
INTRINSICS_L="857;0;313;0;857;241;0;0;1" #LEFT

# Intrinsics Analysis
echo "Begin intrinsics analysis - image listing..."
echo "RUN: $OPENMVG_INSTALL_DIR/bin/openMVG_main_SfMInit_ImageListing -i ./ -o $MATCHES_DIR -d $CAMERA_SENSOR_DEFS"
$OPENMVG_INSTALL_DIR/openMVG_main_SfMInit_ImageListing -i ./$INPUT_DIR -o $MATCHES_DIR -d $CAMERA_SENSOR_DEFS -f 3290

# Compute Features
echo "Computing features within images..."
echo "RUN: $OPENMVG_INSTALL_DIR/bin/openMVG_main_ComputeFeatures -i $MATCHES_DIR/sfm_data.json -o $MATCHES_DIR -m $CFEATURE_TYPE -p $CDESC_TYPE -n $THREADS"
$OPENMVG_INSTALL_DIR/openMVG_main_ComputeFeatures -i $MATCHES_DIR/sfm_data.json -o $MATCHES_DIR -m $CFEATURE_TYPE -p $CDESC_TYPE -n $THREADS

# Compute Matches
echo "Computing matches between features in images..."
echo "RUN: $OPENMVG_INSTALL_DIR/bin/openMVG_main_ComputeMatches -i $MATCHES_DIR/sfm_data.json -o $MATCHES_DIR -n $MMETHOD -r $MRATIO"
$OPENMVG_INSTALL_DIR/openMVG_main_ComputeMatches -i $MATCHES_DIR/sfm_data.json -o $MATCHES_DIR -n $MMETHOD -r $MRATIO

# Reconstruction - seq
echo "Performing Sequential reconstruction..."
echo "RUN: $OPENMVG_INSTALL_DIR/bin/openMVG_main_IncrementalSfM -i $MATCHES_DIR/sfm_data.json -m $MATCHES_DIR -o $RECONSTRUCT_DIR -c 1"
$OPENMVG_INSTALL_DIR/openMVG_main_IncrementalSfM -i $MATCHES_DIR/sfm_data.json -m $MATCHES_DIR -o $RECONSTRUCT_DIR -c 1

if [ $? -gt 0 ]
then
  echo "Failed to start reconstruction, likely camera data doesn't exist."
  exit 1
fi

# Colorize structure
echo "Colorizing..."
echo "RUN: $OPENMVG_INSTALL_DIR/bin/openMVG_main_ComputeSfM_DataColor -i $RECONSTRUCT_DIR/sfm_data.bin -o $RECONSTRUCT_DIR/colorized.ply"
$OPENMVG_INSTALL_DIR/openMVG_main_ComputeSfM_DataColor -i $RECONSTRUCT_DIR/sfm_data.bin -o $RECONSTRUCT_DIR/colorized.ply

# Structure from known poses
echo "Structure from known poses..."
echo "RUN: $OPENMVG_INSTALL_DIR/bin/openMVG_main_ComputeStructureFromKnownPoses -i $RECONSTRUCT_DIR/sfm_data.bin -m $MATCHES_DIR -f $MATCHES_DIR/matches.f.bin -o $RECONSTRUCT_DIR/robust.json -r $ROBUSTFILTER"
$OPENMVG_INSTALL_DIR/openMVG_main_ComputeStructureFromKnownPoses -i $RECONSTRUCT_DIR/sfm_data.bin -m $MATCHES_DIR -f $MATCHES_DIR/matches.f.bin -o $RECONSTRUCT_DIR/robust.json -r $ROBUSTFILTER

echo "Colorizing..."
echo "RUN: $OPENMVG_INSTALL_DIR/bin/openMVG_main_ComputeSfM_DataColor -i $RECONSTRUCT_DIR/robust.json -o $RECONSTRUCT_DIR/robust_colorized.ply"
$OPENMVG_INSTALL_DIR/openMVG_main_ComputeSfM_DataColor -i $RECONSTRUCT_DIR/robust.json -o $RECONSTRUCT_DIR/robust_colorized.ply

# Export from OpenMVG to OpenMVS
echo "Convert to OPENMVS for handover..."
echo "RUN: $OPENMVG_INSTALL_DIR/bin/openMVG_main_openMVG2openMVS -i ./$RECONSTRUCT_DIR/sfm_data.bin -o ./$DEFAULT_OUTPUT_DIR/scene.mvs"
$OPENMVG_INSTALL_DIR/openMVG_main_openMVG2openMVS -i ./$RECONSTRUCT_DIR/sfm_data.bin -o ./$DEFAULT_OUTPUT_DIR/scene.mvs

# Create Dense Point Cloud
echo "OPENMVS Create dense point cloud..."
echo "RUN: $OPENMVS_BUILD_DIR/bin/DensifyPointCloud ./$DEFAULT_OUTPUT_DIR/scene.mvs"
$OPENMVS_BUILD_DIR/bin/DensifyPointCloud ./$DEFAULT_OUTPUT_DIR/scene.mvs

# Create Mesh from Point Cloud
echo "OPENMVS Create mesh..."
echo "RUN: $OPENMVS_BUILD_DIR/bin/ReconstructMesh ./$DEFAULT_OUTPUT_DIR/scene_dense.mvs"
$OPENMVS_BUILD_DIR/bin/ReconstructMesh ./$DEFAULT_OUTPUT_DIR/scene_dense.mvs

# Create textures for PointCloudMesh
echo "OPENMVS Create textures for mesh..."
echo "RUN: $OPENMVS_BUILD_DIR/bin/TextureMesh ./$DEFAULT_OUTPUT_DIR/scene_dense_mesh.mvs"
$OPENMVS_BUILD_DIR/bin/TextureMesh ./$DEFAULT_OUTPUT_DIR/scene_dense_mesh.mvs

# Cleanup of logs and stuff
echo "Cleanup..."
if [ ! -d logs ]
then
  mkdir logs
fi

mv *log ./logs/
rm *dmap

echo "Done!..."
