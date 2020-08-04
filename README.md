# Depth Clustering

For Ubuntu 14.04, install the following packages:

```bash
apt install libopencv-dev libqglviewer-dev freeglut3-dev libqt4-dev
```

For Ubuntu 16.04, install the following packages:

```bash
apt install libopencv-dev libqglviewer-dev freeglut3-dev libqt5-dev
```

For Ubuntu 18.04, install the following packages:

```bash
apt install libopencv-dev libqglviewer-dev-qt5 freeglut3-dev qtbase5-dev
```

With the above packages installed, navigate under the project folder, then set up and build the project as follows:

```bash
cd scripts
./setup.bash
cd ../build/release
make -j4
```

Then, run the project as follows:

```bash
cd ../../install/bin
./depth_clustering
```