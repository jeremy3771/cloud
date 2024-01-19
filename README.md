# cloud
> Minhyuk Kim

## 1. mid360_docker
Using MID-360 with Docker in JAO
### 1.1 build container
#### Dockerfile
```shell
FROM arm64v8/ros:humble-perception-jammy #ROS2 in ARM64 arch
```
#### container_run.sh
```shell
docker run --runtime=nvidia \ #nvidia-docker -> docker & --runtime=nvidia
           --privileged -it \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -e NVIDIA_VISIBLE_DEVICES=all \
           --volume="$PROJECT_DIR:/root/ros2_ws/src" \
           --volume=/data/LIDAR_dataset:/root/data \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --net=host \
           --ipc=host \
           --pid=host \
           --shm-size=4gb \
           --name="$CONTAINER_NAME" \
           --env="DISPLAY=$DISPLAY" \
           "$IMAGE_NAME" /bin/bash
```
### 1.2 msg_MID360_launch.py
```shell
xfer_format = 0 # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
```
To use ros2 Pointcloud2 instead customized pointcloud format

## TODO
- [ ] 2D mapping with 3D Lidar
