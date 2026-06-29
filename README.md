# [NV-LIOM](https://github.com/dhchung/nv_liom) converter to [HDMapping](https://github.com/MapsHD/HDMapping)

## Hint

Please change branch to:

[Bunker-DVI-Dataset-reg-1](https://github.com/MapsHD/benchmark-NV-LIOM-to-HDMapping/tree/Bunker-DVI-Dataset-reg-1)

for quick experiment.

---

## Intended use

This repository integrates **NV-LIOM** with **HDMapping**.

It contains:

- NV-LIOM workspace
- tested NV-LIOM configuration
- converter for HDMapping output

NV-LIOM publishes:

```
/nv_liom/deskew_cloud
/nv_liom/imu_odometry
```

and converts recorded data into HDMapping format.

---

## Dependencies

```bash
sudo apt install -y nlohmann-json3-dev
```

---

## Build

Clone repository:

```bash
mkdir -p ~/test_ws/src

cd ~/test_ws/src

git clone https://github.com/MapsHD/benchmark-NV-LIOM-to-HDMapping.git --recursive

cd ~/test_ws

catkin_make
```

Source workspace:

```bash
source /opt/ros/noetic/setup.bash
source ~/test_ws/devel/setup.bash
```

---

# Usage

## Start NV-LIOM

Run:

```bash
roslaunch nv_liom run.launch use_sim_time:=true
```

---

## Play dataset

In another terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/test_ws/devel/setup.bash

rosbag play <dataset.bag> --clock
```

---

## Record NV-LIOM output

Record:

```bash
rosbag record \
/nv_liom/deskew_cloud \
/nv_liom/imu_odometry \
-O recorded-nv-liom.bag
```

---

## Convert to HDMapping

After recording:

```bash
source /opt/ros/noetic/setup.bash
source ~/test_ws/devel/setup.bash

rosrun nv-liom-to-hdmapping listener \
recorded-nv-liom.bag \
output_hdmapping
```

Output:

```
output_hdmapping-NV-LIOM/
```

---

## Stop

Stop processes:

```
CTRL+C
```