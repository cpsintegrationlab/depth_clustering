#!/bin/bash

# Declare global variables
CORES=$(grep -c ^processor /proc/cpuinfo)
ECLIPSE=4.14.0
DB_TYPE=Debug
RL_TYPE=Release

# Declare directory variables
PROJECT_DIR=$(pwd)/..
SRC_DIR=$PROJECT_DIR/src
BUILD_DIR=$PROJECT_DIR/build
DB_DIR=$BUILD_DIR/$(echo ${DB_TYPE,,})
RL_DIR=$BUILD_DIR/$(echo ${RL_TYPE,,})
TEMP_DIR=$PROJECT_DIR/temp

# Create build folders
printf "\nCreating build folders...\n\n"
if [ -d "$BUILD_DIR" ]; then
    printf "\nRemoving existing build folders...\n\n"
    rm -rf "$BUILD_DIR"
fi
mkdir -p "$DB_DIR"
mkdir -p "$RL_DIR"
RETURN=$?
if [ $RETURN -ne 0 ]; then
    printf "\nSetup failed. Quit.\n\n"
    exit $RETURN
fi

cd "$DB_DIR"
cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=$DB_TYPE -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j$CORES -DCMAKE_ECLIPSE_VERSION=$ECLIPSE "$SRC_DIR"
RETURN=$?
if [ $RETURN -ne 0 ]; then
    printf "\nSetup failed. Quit.\n\n"
    exit $RETURN
fi

cd "$RL_DIR"
cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=$RL_TYPE -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j$CORES -DCMAKE_ECLIPSE_VERSION=$ECLIPSE "$SRC_DIR"
RETURN=$?
if [ $RETURN -ne 0 ]; then
    printf "\nSetup failed. Quit.\n\n"
    exit $RETURN
fi

printf "\nSetup completed.\n\n"
