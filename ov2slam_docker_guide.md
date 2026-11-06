# OV2SLAM Docker Build and Run Guide

This guide covers building the OV2SLAM Docker image, launching it with host networking to access a connected USB camera, and running the node structure with RViz.

## 1. Build the Docker Image

Run the following command from the root of the `ov2slam` repository to build the image (it uses the Dockerfile located in the `docker` directory):

```bash
cd ~/slam/ov2slam
docker build --network=host -t ov2slam_docker -f docker/Dockerfile .
```

> **Note:** Building might take a while as it will compile Eigen, OpenCV, OpenGV, Sophus, Ceres, and the workspace itself.

## 2. Allow X11 forwarding from Docker

To visualize the camera output and SLAM results on RViz, you'll need to allow Docker to access your local X server display:

```bash
xhost +local:root
```
*(If the above fails, you can try `xhost +local:root` or just `xhost +` temporarily)*

## 3. Run the Docker Container (Local & SSH)

If you are running directly on the machine:

```bash
docker run -it --rm \
    --net=host \
    --ipc=host \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device="/dev/video0:/dev/video0" \
    --device="/dev/i2c-1:/dev/i2c-1" \
    --volume="$HOME/robot_logs:/ws/robot_logs:rw" \
    ov2slam_docker \
    bash
```
*(Note: `--device="/dev/i2c-1:/dev/i2c-1"` was added to give container access to the sensors)*

*(Note: `--volume="$HOME/robot_logs:/ws/robot_logs:rw"` - logi przejazdu z `traj_logger` laduja od razu na hoscie w `~/robot_logs/run_*`. Pliki tworzy root z kontenera, wiec ewentualnie `sudo chown -R $USER ~/robot_logs`. Plot: `python3 scripts/plot_run.py ~/robot_logs/run_...`)*

**Running over SSH (with X11 Forwarding):**
If you are connecting via SSH, you must connect with the `-Y` flag (trusted X11 forwarding):
```bash
ssh -Y user@robot_ip
```
And then run the docker container with your local `.Xauthority` mounted so the container can authorize with the forwarded X11 session:
```bash
docker run -it --rm \
    --net=host \
    --ipc=host \
    --env="DISPLAY=$DISPLAY" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device="/dev/video0:/dev/video0" \
    --device="/dev/i2c-1:/dev/i2c-1" \
    --volume="$HOME/robot_logs:/ws/robot_logs:rw" \
    ov2slam_docker \
    bash
```

## 4. Run the ROS2 Nodes (Inside the Container)

Once inside the container, you will be in the `/ws` directory.

### Sourcing the Workspaces
First, source the main ROS distro and your built workspace:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
```

### Launching everything via Bringup

Instead of launching all nodes in separate terminals, we can launch the camera in the background and then use the main bringup file.

1. **Launch the camera (in the background):**
```bash
apt-get update && apt-get install -y ros-humble-v4l2-camera
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:=[640,480] --remap /image_raw:=/cam0/image_raw &
```

2. **Launch all SLAM, Sensors, and RViz nodes at once:**
```bash
ros2 launch ov2slam bringup.launch.py
```

You should now be able to see the SLAM pipeline attempting to output tracking and mapping results based on your live mono camera, with all nodes (IMU, Encoders, EKF, Map2D) running seamlessly!
