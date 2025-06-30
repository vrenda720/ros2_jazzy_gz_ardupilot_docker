# Build:

```
sudo docker build -t ros2_jazzy_gz_ardu .
```


# Run:

```
xhost +local:root
```

```
sudo docker run -it --rm \
-e DISPLAY=$DISPLAY \
-v / tmp/. X11-unix:/tmp/.X11-unix \
ros2_jazzy_gz_ardu
```


# Launch (Example):

```
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```
