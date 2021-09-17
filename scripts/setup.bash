#!/bin/bash

# Declare global variables
CORES=$(grep -c ^processor /proc/cpuinfo)
ECLIPSE=4.20.0
DB_TYPE=Debug
RL_TYPE=Release

# Declare directory variables
PROJECT_DIR=$(pwd)/..
SRC_DIR=$PROJECT_DIR/src
BUILD_DIR=$PROJECT_DIR/build
INSTALL_DIR=$PROJECT_DIR/install
TEMP_DIR=$PROJECT_DIR/temp

# Declare Boost variables
BOOST_DIR_BUILD=$BUILD_DIR/boost
BOOST_DIR_INSTALL=$INSTALL_DIR/boost
BOOST_VER_MAJ=1
BOOST_VER_MIN=64
BOOST_VER_PAT=0
BOOST_URL="https://sourceforge.net/projects/boost/files/boost/$BOOST_VER_MAJ.$BOOST_VER_MIN.$BOOST_VER_PAT/boost_${BOOST_VER_MAJ}_${BOOST_VER_MIN}_${BOOST_VER_PAT}.tar.gz/download"
BOOST_CFLAGS="-Wno-maybe-uninitialized -Wno-unused-function -Wno-unused-variable -Wno-aligned-new -Wno-parentheses -Wno-deprecated-declarations -fPIC"
BOOST_BOOTSTRAP_FLAGS="--with-libraries=system,filesystem,regex,program_options"

# Declare Eigen variables
EIGEN_DIR_SRC=$SRC_DIR/eigen
EIGEN_DIR_BUILD=$BUILD_DIR/eigen
EIGEN_DIR_INSTALL=$INSTALL_DIR/eigen
EIGEN_VER_MAJ=3
EIGEN_VER_MIN=3
EIGEN_VER_PAT=4
EIGEN_URL="https://gitlab.com/libeigen/eigen/-/archive/$EIGEN_VER_MAJ.$EIGEN_VER_MIN.$EIGEN_VER_PAT/eigen-${EIGEN_VER_MAJ}.${EIGEN_VER_MIN}.${EIGEN_VER_PAT}.tar.gz"

# Declare OpenCV variables
OPENCV_DIR_SRC=$SRC_DIR/opencv
OPENCV_DIR_BUILD=$BUILD_DIR/opencv
OPENCV_DIR_INSTALL=$INSTALL_DIR/opencv
OPENCV_VER_MAJ=3
OPENCV_VER_MIN=2
OPENCV_VER_PAT=0
OPENCV_URL="https://github.com/opencv/opencv/archive/${OPENCV_VER_MAJ}.${OPENCV_VER_MIN}.${OPENCV_VER_PAT}.tar.gz"

# Declare Depth Clustering variables
DC_DIR_BUILD_DB=$BUILD_DIR/depth_clustering/$(echo ${DB_TYPE,,})
DC_DIR_BUILD_RL=$BUILD_DIR/depth_clustering/$(echo ${RL_TYPE,,})

# Create temporary folder
if [ -d "$TEMP_DIR" ]; then
	echo "[INFO]: Removing existing temporary folders..."
	rm -rf "$TEMP_DIR"
fi
echo "[INFO]: Creating temporary folder..."
mkdir -p "$TEMP_DIR"
RETURN=$?
if [ $RETURN -ne 0 ]; then
	echo "[ERROR]: Setup failed. Quit."
	exit $RETURN
fi

# Install Boost
echo "[INFO]: Installing Boost $BOOST_VER_MAJ.$BOOST_VER_MIN.$BOOST_VER_PAT..."
if [ ! -d "$BOOST_DIR_BUILD" ]; then
	echo "[INFO]: Downloading Boost $BOOST_VER_MAJ.$BOOST_VER_MIN.$BOOST_VER_PAT..."
	cd "$TEMP_DIR"
	wget -q --show-progress "$BOOST_URL" -O "boost_${BOOST_VER_MAJ}_${BOOST_VER_MIN}_${BOOST_VER_PAT}.tar.gz"

	echo "[INFO]: Extracting Boost $BOOST_VER_MAJ.$BOOST_VER_MIN.$BOOST_VER_PAT..."
	tar -xzf "boost_${BOOST_VER_MAJ}_${BOOST_VER_MIN}_${BOOST_VER_PAT}.tar.gz"
	mv "boost_${BOOST_VER_MAJ}_${BOOST_VER_MIN}_${BOOST_VER_PAT}" "$BOOST_DIR_BUILD"
else
	echo "[INFO]: Boost build folder exists. Skip."
fi
if [ ! -d "$BOOST_DIR_INSTALL" ]; then
	echo "[INFO]: Building Boost $BOOST_VER_MAJ.$BOOST_VER_MIN.$BOOST_VER_PAT..."
	cd "$BOOST_DIR_BUILD"
	./bootstrap.sh "$BOOST_BOOTSTRAP_FLAGS" --prefix="$(printf "%q" "$BOOST_DIR_INSTALL")"
	./b2 cxxflags="$BOOST_CFLAGS" cflags="$BOOST_CFLAGS" -j$CORES install
	RETURN=$?
	if [ $RETURN -ne 0 ]; then
		echo "[ERROR]: Setup failed. Quit."
		exit $RETURN
	fi
else
	echo "[INFO]: Boost install folder exists. Skip."
fi

# Install Eigen
echo "[INFO]: Installing Eigen $EIGEN_VER_MAJ.$EIGEN_VER_MIN.$EIGEN_VER_PAT..."
if [ ! -d "$EIGEN_DIR_SRC" ]; then
	echo "[INFO]: Downloading Eigen $EIGEN_VER_MAJ.$EIGEN_VER_MIN.$EIGEN_VER_PAT..."
	cd "$TEMP_DIR"
	wget -q --show-progress "$EIGEN_URL"

	echo "[INFO]: Extracting Eigen $EIGEN_VER_MAJ.$EIGEN_VER_MIN.$EIGEN_VER_PAT..."
	tar -xzf "eigen-${EIGEN_VER_MAJ}.${EIGEN_VER_MIN}.${EIGEN_VER_PAT}.tar.gz"
	mv "eigen-${EIGEN_VER_MAJ}.${EIGEN_VER_MIN}.${EIGEN_VER_PAT}" "$EIGEN_DIR_SRC"
else
	echo "[INFO]: Eigen source folder exists. Skip."
fi
if [ ! -d "$EIGEN_DIR_BUILD" ]; then
	echo "[INFO]: Setting up Eigen $EIGEN_VER_MAJ.$EIGEN_VER_MIN.$EIGEN_VER_PAT..."
	mkdir -p "$EIGEN_DIR_BUILD"
	cd "$EIGEN_DIR_BUILD"
	cmake -DCMAKE_INSTALL_PREFIX="$EIGEN_DIR_INSTALL" "$EIGEN_DIR_SRC"
else
	echo "[INFO]: Eigen build folder exists. Skip."
fi
if [ ! -d "$EIGEN_DIR_INSTALL" ]; then
	echo "[INFO]: Building Eigen $EIGEN_VER_MAJ.$EIGEN_VER_MIN.$EIGEN_VER_PAT..."
	cd "$EIGEN_DIR_BUILD"
	make -j$CORES install
	RETURN=$?
	if [ $RETURN -ne 0 ]; then
		echo "[ERROR]: Setup failed. Quit."
		exit $RETURN
	fi
else
	echo "[INFO]: Eigen install folder exists. Skip."
fi

# Install OpenCV
echo "[INFO]: Installing OpenCV $OPENCV_VER_MAJ.$OPENCV_VER_MIN.$OPENCV_VER_PAT..."
if [ ! -d "$OPENCV_DIR_SRC" ]; then
	echo "[INFO]: Downloading OpenCV $OPENCV_VER_MAJ.$OPENCV_VER_MIN.$OPENCV_VER_PAT..."
	cd "$TEMP_DIR"
	wget -q --show-progress "$OPENCV_URL"

	echo "[INFO]: Extracting OpenCV $OPENCV_VER_MAJ.$OPENCV_VER_MIN.$OPENCV_VER_PAT..."
	tar -xzf "${OPENCV_VER_MAJ}.${OPENCV_VER_MIN}.${OPENCV_VER_PAT}.tar.gz"
	mv "opencv-${OPENCV_VER_MAJ}.${OPENCV_VER_MIN}.${OPENCV_VER_PAT}" "$OPENCV_DIR_SRC"
else
	echo "[INFO]: OpenCV source folder exists. Skip."
fi
if [ ! -d "$OPENCV_DIR_BUILD" ]; then
	echo "[INFO]: Setting up OpenCV $OPENCV_VER_MAJ.$OPENCV_VER_MIN.$OPENCV_VER_PAT..."
	mkdir -p "$OPENCV_DIR_BUILD"
	cd "$OPENCV_DIR_BUILD"
	cmake -DCMAKE_INSTALL_PREFIX="$OPENCV_DIR_INSTALL" -DENABLE_PRECOMPILED_HEADERS=OFF "$OPENCV_DIR_SRC"
else
	echo "[INFO]: OpenCV build folder exists. Skip."
fi
if [ ! -d "$OPENCV_DIR_INSTALL" ]; then
	echo "[INFO]: Building OpenCV $OPENCV_VER_MAJ.$OPENCV_VER_MIN.$OPENCV_VER_PAT..."
	cd "$OPENCV_DIR_BUILD"
	make -j$CORES install
	RETURN=$?
	if [ $RETURN -ne 0 ]; then
		echo "[ERROR]: Setup failed. Quit."
		exit $RETURN
	fi
else
	echo "[INFO]: OpenCV install folder exists. Skip."
fi

# Remove temporary folder
if [ -d "$TEMP_DIR" ]; then
	echo "[INFO]: Removing temporary folder..."
	rm -rf "$TEMP_DIR"
fi

# Create Depth Clustering debug project
if [ ! -d "$DC_DIR_BUILD_DB" ]; then
	echo "[INFO]: Creating Depth Clustering debug project..."
	mkdir -p "$DC_DIR_BUILD_DB"
	cd "$DC_DIR_BUILD_DB"
	cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=$DB_TYPE -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j$CORES -DCMAKE_ECLIPSE_VERSION=$ECLIPSE "$SRC_DIR"
	RETURN=$?
	if [ $RETURN -ne 0 ]; then
		echo "[ERROR]: Setup failed. Quit."
		exit $RETURN
	fi
else
	echo "[INFO]: Depth Clustering debug project exists. Skip."
fi

# Create Depth Clustering release project
if [ ! -d "$DC_DIR_BUILD_RL" ]; then
    echo "[INFO]: Creating Depth Clustering release project..."
	mkdir -p "$DC_DIR_BUILD_RL"
	cd "$DC_DIR_BUILD_RL"
	cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=$RL_TYPE -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j$CORES -DCMAKE_ECLIPSE_VERSION=$ECLIPSE "$SRC_DIR"
	RETURN=$?
	if [ $RETURN -ne 0 ]; then
		echo "[ERROR]: Setup failed. Quit."
		exit $RETURN
	fi
else
	echo "[INFO]: Depth Clustering release project exists. Skip."
fi

echo "[INFO]: Setup completed."
