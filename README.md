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

Launch wildthumper playpen test:
```
ros2 launch ardupilot_gz_bringup wildthumper_playpen.launch.py
```

Launch mavlink test:
```
gz sim -v4 -r wildthumper_runway.sdf
```
```
./ws/src/ardupilot/Tools/autotest/sim_vehicle.py -v Rover -f gazebo-rover --model JSON --console --map --moddebug 3
```

GUIDED > arm throttle force > (Give it a location) > (Hits HOLD) > disarm > GUIDED > arm throttle force

gz topic -t /wamv/thrusters/left/thrust -e

gz topic -t /wamv/thrusters/left/thrust --pub 'data: 10000.0' -m gz.msgs.Double

sudo docker container ls

docker exec -it ${container_name} /ros_entrypoint.sh /bin/bash

VRX test:
```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

* Gazebo currently doesn't grab the gpus ):
