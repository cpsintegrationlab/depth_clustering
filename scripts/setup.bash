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

# Declare Depth Clustering variables
DC_DIR_BUILD_DB=$BUILD_DIR/depth_clustering/$(echo ${DB_TYPE,,})
DC_DIR_BUILD_RL=$BUILD_DIR/depth_clustering/$(echo ${RL_TYPE,,})

# Install required packages
echo "[INFO]: Installing required packages..."
if [ `cat /proc/version | grep -c "Ubuntu"` -gt 0 ] || [ `cat /proc/version | grep -c "Microsoft"` -gt 0 ]; then
    sudo apt-get --assume-yes install cmake
	RETURN=$?
	if [ $RETURN -ne 0 ]; then
		echo "[ERROR]: Setup failed. Quit."
		exit $RETURN
	fi
elif [ `cat /proc/version | grep -c "ARCH"` -gt 0 ]; then
    sudo pacman -S --noconfirm --needed cmake
	RETURN=$?
	if [ $RETURN -ne 0 ]; then
		echo "[ERROR]: Setup failed. Quit."
		exit $RETURN
	fi
elif [ `cat /proc/version | grep -c "Red Hat"` -gt 0 ]; then
	sudo yum --assumeyes install cmake
	RETURN=$?
	if [ $RETURN -ne 0 ]; then
		echo "[ERROR]: Setup failed. Quit."
		exit $RETURN
	fi
else
    echo "[ERROR]: Unsupported Operating System. Quit."
    exit
fi

# Create build folders
echo "[INFO]: Creating build folders..."
if [ -d "$DC_DIR_BUILD_DB" ]; then
    echo "[INFO]: Removing existing Depth Clustering debug build folder..."
    rm -rf "$DC_DIR_BUILD_DB"
fi
if [ -d "$DC_DIR_BUILD_RL" ]; then
    echo "[INFO]: Removing existing Depth Clustering release build folder..."
    rm -rf "$DC_DIR_BUILD_RL"
fi
mkdir -p "$DC_DIR_BUILD_DB"
mkdir -p "$DC_DIR_BUILD_RL"
RETURN=$?
if [ $RETURN -ne 0 ]; then
    echo "[ERROR]: Setup failed. Quit."
    exit $RETURN
fi

# Create temporary folder
echo "[INFO]: Creating temporary folder..."
if [ -d "$TEMP_DIR" ]; then
	echo "[INFO]: Removing existing temporary folders..."
	rm -rf "$TEMP_DIR"
fi
mkdir -p "$TEMP_DIR"
RETURN=$?
if [ $RETURN -ne 0 ]; then
	echo "[ERROR]: Setup failed. Quit."
	exit $RETURN
fi

# Install Boost
echo "[INFO]: Installing Boost $BOOST_VER_MAJ.$BOOST_VER_MIN.$BOOST_VER_PAT..."
if [ ! -d "$BOOST_DIR_BUILD" ]; then
	cd "$TEMP_DIR"

	echo "[INFO]: Downloading Boost $BOOST_VER_MAJ.$BOOST_VER_MIN.$BOOST_VER_PAT..."
	wget -q --show-progress "$BOOST_URL" -O "boost_${BOOST_VER_MAJ}_${BOOST_VER_MIN}_${BOOST_VER_PAT}.tar.gz"

	echo "[INFO]: Extracting Boost $BOOST_VER_MAJ.$BOOST_VER_MIN.$BOOST_VER_PAT..."
	tar -xzf "boost_${BOOST_VER_MAJ}_${BOOST_VER_MIN}_${BOOST_VER_PAT}.tar.gz"
	mv -v "boost_${BOOST_VER_MAJ}_${BOOST_VER_MIN}_${BOOST_VER_PAT}" "$BOOST_DIR_BUILD"

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
	echo "[INFO]: Boost installation exists. Skip."
fi

# Remove temporary folder
if [ -d "$TEMP_DIR" ]; then
	echo "[INFO]: Removing temporary folder..."
	rm -rf "$TEMP_DIR"
fi

# Create Depth Clustering debug project files
echo "[INFO]: Creating Depth Clustering debug project files..."
cd "$DC_DIR_BUILD_DB"
cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=$DB_TYPE -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j$CORES -DCMAKE_ECLIPSE_VERSION=$ECLIPSE "$SRC_DIR"
RETURN=$?
if [ $RETURN -ne 0 ]; then
    echo "[ERROR]: Setup failed. Quit."
    exit $RETURN
fi

# Create Depth Clustering release project file
echo "[INFO]: Creating Depth Clustering release project files..."
cd "$DC_DIR_BUILD_RL"
cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=$RL_TYPE -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j$CORES -DCMAKE_ECLIPSE_VERSION=$ECLIPSE "$SRC_DIR"
RETURN=$?
if [ $RETURN -ne 0 ]; then
    echo "[ERROR]: Setup failed. Quit."
    exit $RETURN
fi

echo "[INFO]: Setup completed."
