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
-v /tmp/.X11-unix:/tmp/.X11-unix \
--gpus 'all' \
ros2_jazzy_gz_ardu
```


```
sudo docker run -it --rm \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--device /dev/dri/card1 \
ros2_jazzy_gz_ardu
```


# Launch (Example):

```
ros2 launch ardupilot_gz_bringup wildthumper_playpen.launch.py
```

```
gz sim -v4 -r wildthumper_runway.sdf
```
```
./ws/src/ardupilot/Tools/autotest/sim_vehicle.py -v Rover -f gazebo-rover --model JSON --console --map --moddebug 3
```


* Gazebo currently doesn't grab the gpus ):
