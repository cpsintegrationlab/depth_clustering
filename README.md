# Depth Clustering 2.0

## Table of Contents

1. [Project Setup](#project-setup)
2. [Command-Line Application](#command-line-application)
3. [Visualizer Application](#visualizer-application)
4. [Cross-Compilation](#cross-compilation)
5. [Installation](#installation)
6. [Batch Execution Script](#batch-execution-script)

## Project Setup

This project depends on the installation of the following essential packages:

```bash
apt install build-essential cmake tar wget
```

With the above packages installed, set up the project as follows:

```bash
cd scripts
./setup.bash
```

## Command-Line Application

With the project set up, build the command-line application as follows:

```bash
cd build/amd64/depth_clustering/release
make
```

To run the command-line application, do as follows:

```bash
cd install/amd64/depth_clustering/release/bin
./depth_clustering dataset_segment_path global_config_file lidar_return_directory
```

Note:
1. Any resulting output files would be stored under the provided `dataset_segment_path` directory. Two example datasets are provided under the `data` folder.
2. The `global_config_file` is an optional parameter that specifies the path to a global configuration file. The global configuration file enables the same set of configurations to be applied across various datasets. Regardless of the presence of the `global_config_file`, the applications would first attempt to load the configuration file under the `dataset_segment_path` folder first. When specified, the global configuration file would be loaded, and the loaded global configurations would then override the existing configurations.
3. The `lidar_return_directory` is an optional parameter that specifies the directory name of the lidar frames of a particular lidar return, such as `first_return`, `second_return`, etc., inside the `dataset_segment_path` directory. The command-line application uses first return lidar frames by default if `lidar_return_directory` is unspecified.

## Visualizer Application

The visualizer application depends on the installation of the following graphical packages:

```bash
apt install freeglut3-dev libqglviewer-dev-qt5 qtbase5-dev
```

The visualizer application is built independently. With the above packages installed, build the visualizer application as follows:

```bash
cd build/amd64/depth_clustering/release
make visualizer
```

To run the visualizer application, do as follows:

```bash
cd install/amd64/depth_clustering/release/bin
./visualizer
```

To launch the visualizer application with a dataset path, do as follows:

```bash
cd install/amd64/depth_clustering/release/bin
./visualizer dataset_segment_path global_config_file layout_config_file
```

Note:
1. Any resulting output files would be stored under the provided `dataset_segment_path` folder.
2. The `global_config_file` is as explained [above](#command-line-application).
3. The `layout_config_file` is an optional parameter that specifies the path to a layout configuration file. The visualizer application takes the layout configuration file and applies the corresponding layout configurations. If any entries in `layout_config_file` conflicts with those in `global_config_file`, entries in `layout_config_file` supersede.

## Cross-Compilation

The cross-compilation of this project depends on the installation of the following package:

```bash
apt install crossbuild-essential-arm64
```

Currently, the project supports two archtectures, amd64 and arm64. The instructions above already demonstrate steps for amd64.

For arm64, set up the project as follows:

```bash
cd scripts
./setup.bash --arch=arm64
```

Then, build the command-line application as follows:

```bash
cd build/arm64/depth_clustering/release
make
```

The cross-compiled binaries and libraries for arm64 would be located in `install/arm64/depth_clustering/release`

Note: the visualizer application is currently not supported for the arm64 architecture.

## Installation

This project can be installed to the system as follows:

```bash
cd build/amd64/depth_clustering/release
make install
```

The installed binaries and scripts, libraries, and headers would be located under `/usr/local/bin`, `/usr/local/lib`, and `/usr/local/include/depth_clustering`, respectively.

Then, refresh the dynamic linker cache to register the newly installed shared libraries as follows:

```bash
ldconfig
```

Note: the visualizer application would not be installed.

## Batch Execution Script

We have included a batch execution script in `scripts` for the command-line application.

The batch execution script exploits the parallelism in multi-core processors and expediate the executions of the command-line application over a large set of dataset segments by concurrently executing multiple instances of the command-line application in parallel. The maximum number of concurrent instances is currently limited to the number of logical CPU cores in the system.

To run the command-line application for a set of dataset segments, do the following:

```bash
cd scripts
./depth_clustering.bash dataset_path arguments
```

Note:
1. The `dataset_path` is the path to the folder containing a set of dataset segments.
2. The `arguments` are passed directly to the command-line application. For multiple arguments, enclose all of them with quotes `""`.
3. The batch execution script expects all dataset segments to be directly under `dataset_path`.