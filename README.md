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

Then, run the command line application as follows:

```bash
cd ../../install/bin
./depth_clustering dataset_path
```

To visualize the detections frame by frame, run the camera visualizer as follows:

```bash
./camera_visualizer dataset_path 0
```

To visualize the detections as a video, run the camera visualizer as follows:

```bash
./camera_visualizer dataset_path 1
```

After the executions, any resulting output JSON files would be stored under `dataset_path`.
