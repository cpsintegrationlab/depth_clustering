# Depth Clustering

## Core Applications

The core applications depend on the installation of the following packages:

```bash
apt install cmake libboost-all-dev libeigen3-dev libopencv-dev freeglut3-dev
```

With the above packages installed, set up the project as follows:

```bash
cd scripts
./setup.bash
```

Then, build the core applications as follows:

```bash
cd build/release
make
```

Note: the `global_config_path` is an optional parameter that specifies the path to a folder containing a global configuration file. The global configuration file enables the same set of configurations to be applied across various datasets. Regardless of the presence of the `global_config_path`, the applications below would first attempt to load the configuration file under the `dataset_path` folder first. When specified, the global configuration file would be loaded, and the loaded global configurations would then override the existing configurations.

To run the depth clustering application, do as follows:

```bash
cd install/release/bin
./depth_clustering dataset_path global_config_path
```

To run the ground truth projection application, do as follows:

```bash
cd install/release/bin
./ground_truth_projection dataset_path global_config_path
```

To run the camera visualizer application, do as follows:

```bash
cd install/release/bin
./camera_visualizer dataset_path global_config_path
```

To run the camera visualizer application as a video, do as follows:

```bash
cd install/release/bin
./camera_visualizer dataset_path global_config_path 1
```

## Lidar Visualizer Application

The lidar visualizer application depends on the installation of the following additional packages:

```bash
apt install qtbase5-dev libqglviewer-dev-qt5
```

The lidar visualizer application is built independently. With the above packages installed, build the lidar visualizer application as follows:

```bash
cd build/release
make lidar_visualizer
```

To run the lidar visualizer application, do as follows:

```bash
cd install/release/bin
./lidar_visualizer
```

To run the lidar visualizer application with a dataset path, do as follows:

```bash
cd install/release/bin
./lidar_visualizer dataset_path global_config_path
```

Note: any resulting output JSON files would be stored under the provided `dataset_path` folder. Two example datasets are provided under the `data` folder.
