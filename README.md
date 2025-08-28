# Build:

```
sudo docker build -t ros2_jazzy_gz_ardu .
```


# Run:

## Step 1:

```
xhost +local:root
```


## Step 2 (Neither makes GPU work):

### For SIAI Workstation:
```
sudo docker run -it --rm \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--gpus 'all' \
ros2_jazzy_gz_ardu
```

### For the Ubuntu/Windows lab Laptop:
```
sudo docker run -it --rm \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--device /dev/dri/card1 \
ros2_jazzy_gz_ardu
```


# Launching/Using SITL:

## Launch default wildthumper playpen test:
```
ros2 launch ardupilot_gz_bringup wildthumper_playpen.launch.py
```


## Launch default mavlink test:

### Step 1:

```
gz sim -v4 -r wildthumper_runway.sdf
```

### Step 2:

```
./ws/src/ardupilot/Tools/autotest/sim_vehicle.py -v Rover -f gazebo-rover --model JSON --console --map --moddebug 3
```


## Launch VRX with SITL

### Step 1:

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta &
```

### Step 2:

```
./ws/src/ardupilot/Tools/autotest/sim_vehicle.py -v Rover -f gazebo-rover --model JSON --console --map --moddebug 3
```

### Step 3 (Within SITL):

- GUIDED
- arm throttle force
- (Give it a location)
- (Hits HOLD)
- disarm
- GUIDED
- arm throttle force (Should start going to location given)

Might need to redo last 3 steps multiple times and give location again if it hits HOLD again



# Other commands that might be useful

gz topic -t /wamv/thrusters/left/thrust -e

gz topic -t /wamv/thrusters/left/thrust --pub 'data: 10000.0' -m gz.msgs.Double

docker exec -it ${container_name} /ros_entrypoint.sh /bin/bash