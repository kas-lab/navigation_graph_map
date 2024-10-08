# Graph-like navigation
Repository with an example use case of a robot navigating in a graph-like environment.

Based on the use case from [Software architecture and task plan co-adaptation for mobile service robots](https://dl.acm.org/doi/pdf/10.1145/3387939.3391591) and adapted to the RoboCup@Home scenario.

## Install

```Bash
mkdir -p ~/navigation_ws/src
cd ~/navigation_ws/src
git clone https://github.com/kas-lab/navigation_graph_map.git
```

### Docker setup

```Bash
cd ~/navigation_ws/src/navigation_graph_map/
docker build -t navigation .
```

## Running

### Simulation

Start docker container:
```Bash
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigation
```

Then run:
```Bash
ros2 launch navigation_simulation simulation.launch.py
```

**Troubleshoot:** The first time you start the simulation, gazebo downloads some stuff from the internet so it might take a while for it to start. Sometimes, it doesn't work on the first run. In that case, stop the launch (ctrl + c) but keep the container running, then start the simulation again.

### Planning

In another terminal:
```Bash
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigation
```

```Bash
ros2 launch navigation_task_plan navigate_task_plan.launch.py
```

### ROSA

In another terminal:
```Bash
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigation
```

```Bash
typedb server
```

In another terminal:
```Bash
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigation
```

```Bash
ros2 topic pub diagnostics diagnostic_msgs/msg/DiagnosticArray
```

In another terminal:

Publish battery level:
```Bash
ros2 topic pub /diagnostics diagnostic_msgs/msg/DiagnosticArray {status:['{message: "QA status", values:[{key: battery, value: 0.5}]}']}
```

```Bash
ros2 service call /rosa_kb/query ros_typedb_msgs/srv/Query "{query_type: 'fetch', query: 'match \$b isa QualityAttribute, has attribute-name \"battery\"; fetch \$b:attribute;'}"
```
