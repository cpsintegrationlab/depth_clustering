#!/bin/bash

CORES=$(grep -c ^processor /proc/cpuinfo)
ECLIPSE=4.20.0
DB_TYPE=Debug
RL_TYPE=Release

PROJECT_DIR=$(pwd)/..
SRC_DIR=$PROJECT_DIR/src
BUILD_DIR=$PROJECT_DIR/build
INSTALL_DIR=$PROJECT_DIR/install
DB_DIR=$BUILD_DIR/$(echo ${DB_TYPE,,})
RL_DIR=$BUILD_DIR/$(echo ${RL_TYPE,,})
TEMP_DIR=$PROJECT_DIR/temp

echo "[INFO]: Creating build folders..."
if [ -d "$BUILD_DIR" ]; then
    echo "[INFO]: Removing existing build folders..."
    rm -rf "$BUILD_DIR"
fi
mkdir -p "$DB_DIR"
mkdir -p "$RL_DIR"
RETURN=$?
if [ $RETURN -ne 0 ]; then
    echo "[ERROR]: Setup failed. Quit."
    exit $RETURN
fi

if [ -d "$INSTALL_DIR" ]; then
    echo "[INFO]: Removing existing install folders..."
    rm -rf "$INSTALL_DIR"
fi
RETURN=$?
if [ $RETURN -ne 0 ]; then
    echo "[ERROR]: Setup failed. Quit."
    exit $RETURN
fi

cd "$DB_DIR"
cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=$DB_TYPE -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j$CORES -DCMAKE_ECLIPSE_VERSION=$ECLIPSE "$SRC_DIR"
RETURN=$?
if [ $RETURN -ne 0 ]; then
    echo "[ERROR]: Setup failed. Quit."
    exit $RETURN
fi

cd "$RL_DIR"
cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=$RL_TYPE -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j$CORES -DCMAKE_ECLIPSE_VERSION=$ECLIPSE "$SRC_DIR"
RETURN=$?
if [ $RETURN -ne 0 ]; then
    echo "[ERROR]: Setup failed. Quit."
    exit $RETURN
fi

echo "[INFO]: Setup completed."
