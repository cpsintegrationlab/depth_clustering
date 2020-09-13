# Depth Clustering

This application depends on the installation of packages as follows:

```bash
apt install libopencv-dev libqglviewer-dev-qt5 freeglut3-dev qtbase5-dev cmake libblkid-dev e2fslibs-dev libboost-all-dev libaudit-dev libeigen3-dev
```

With the above packages installed, navigate under the project folder, then set up the project as follows:

```bash
cd scripts
./setup.bash
```

Then, build the project as follows:

```bash
cd ../build/release
make
```

To build the camera visualizer, use the following make target:

```bash
make camera_visualizer
```

To build the ground truth projection tool, use the following make target:

```bash
make ground_truth_projection
```

Then, run the command line application as follows:

```bash
cd ../../install/bin
./depth_clustering
```

To run with specific dataset, do the following:

```bash
./depth_clustering dataset_path
```

To visualize the detections, run the camera visualizer as follows:

```bash
./camera_visualizer dataset_path
```

To generate projected ground truth, run the ground truth projection tool as follows:

```bash
./ground_truth_projection dataset_path
```

After the executions, any resulting output JSON files would be stored under `dataset_path`.
