# Depth Clustering

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
./depth_clustering dataset_path global_config_path
```

Note: the `global_config_path` is an optional parameter that specifies the path to a folder containing a global configuration file. The global configuration file enables the same set of configurations to be applied across various datasets. Regardless of the presence of the `global_config_path`, the applications would first attempt to load the configuration file under the `dataset_path` folder first. When specified, the global configuration file would be loaded, and the loaded global configurations would then override the existing configurations.

## Visualizer Application

The visualizer application depends on the installation of the following graphical packages:

```bash
apt install freeglut3-dev libqglviewer-dev-qt5 qtbase5-dev
```

The visualizer application is built independently. With the above packages installed, build the visualizer application as follows:

```bash
cd build/releasebuild/amd64/depth_clustering/release
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
./visualizer dataset_path global_config_path
```

Note: any resulting output JSON files would be stored under the provided `dataset_path` folder. Two example datasets are provided under the `data` folder.

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