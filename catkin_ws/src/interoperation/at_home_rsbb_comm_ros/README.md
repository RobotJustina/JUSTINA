RoAH RSBB Comm ROS
==================

This repository contains a ROS package that can be used to communicate
with the ERL-SR RSBB (Referee, Scoring and Benchmarking Box).
To install and know more about the RSBB, see [this repo](https://github.com/rockin-robot-challenge/rsbb).

The repository *RoAH RSBB Comm* from https://github.com/rockin-robot-challenge/at_home_rsbb_comm is included as a git submodule.

:warning: Please remember to always update right before the competitions!
```bash
git pull
git submodule update --init
```


## Dependencies

You need to have installed a C++11 compiler, CMake, Boost, Protobuf
and OpenSSL.

If you are using Ubuntu, install the dependencies with:
```bash
sudo apt-get install build-essential cmake libboost-all-dev libprotoc-dev protobuf-compiler libssl-dev
```

Furthermore, you need to use at least ROS Hydro, follow the
instructions at http://wiki.ros.org/ROS/Installation/ .

This was tested with Ubuntu 12.04.5 LTS (Precise Pangolin) and
14.04.1 LTS (Trusty Tahr).


## Compiling

After `git clone` and after every `git pull`, please do:
```bash
git submodule update --init
```

Compile as a normal ROS package in your Catkin workspace.


## Running

Use the launch file `test.launch` as a base to run the node in your
launch files. All parameters except `team_name` and `rsbb_key` have
sensible defaults. You might also want to set `robot_name`. It can
be simplified, you migth use just something like this:
```xml
  <node pkg="roah_rsbb_comm_ros" type="comm" name="roah_rsbb_comm" respawn="true">
    <param name="team_name" type="string" value="WinnerTeam"/>
    <param name="robot_name" type="string" value="Ronaldo"/>
    <param name="rsbb_key" type="string" value="TEAM_PRIVATE_PASSWORD"/>
  </node>
```

To simply test the node, you can use:
```bash
roslaunch roah_rsbb_comm_ros test.launch --screen
```

To launch a dummy robot implementation to check communication use:
```bash
roslaunch roah_rsbb_comm_ros dummy.launch --screen
```

Both the test and the dummy launch files can take the address of the RSBB as a parameter:
`rsbb_host:=10.2.0.255` . This should be set to the `Bcast` of the interface you want to use, as reported by `ifconfig`. Alternatively, you can use the address of the RSBB directly.


## Using the Node

During operation, the node always publishes some topics.

| Topic                        | Type                              | Notes                                              |
|:----------------------------:|:---------------------------------:|:--------------------------------------------------:|
| `/roah_rsbb/benchmark`       | `roah_rsbb_comm_ros/Benchmark`    | Latched, always published, defaults to `NONE`      |
| `/roah_rsbb/devices/bell`    | `std_msgs/Empty`                  | Not latched, published when the bell rings         |
| `/roah_rsbb/devices/state`   | `roah_rsbb_comm_ros/DevicesState` | Latched, published on reception                    |
| `/roah_rsbb/tablet/call`     | `std_msgs/Empty`                  | Not latched, published when granny calls           |
| `/roah_rsbb/tablet/position` | `geometry_msgs/Pose2D`            | Not latched, published when a position is selected |
| `/roah_rsbb/tablet/state`    | `roah_rsbb_comm_ros/TabletState`  | Latched, published on reception                    |

Topics and services for the benchmarks are only available when the
benchmark starts and `/roah_rsbb/benchmark` becomes something not
`NONE`.

Some topics are common to all benchmarks:

| Topic                        | Type                                | Notes                                              |
|:----------------------------:|:-----------------------------------:|:--------------------------------------------------:|
| `/roah_rsbb/messages_saved`  | `std_msgs/UInt32`                   | Never published, only received.                    |
| `/roah_rsbb/benchmark/state` | `roah_rsbb_comm_ros/BenchmarkState` | Latched, always published, defaults to `STOP`      |

Topic `/roah_rsbb/messages_saved` should be used to send to the RSBB
some kind of metric that shows that offline messages are being
saved. Number of messages or size of the bag file can be used.
This information is only used to issue a warning as soon as
possible if data is not being recorded.

Topic `/roah_rsbb/benchmark/state` has 3 possibilities:
- **STOP**: Stop the robot.
- **PREPARE**: Prepare for benchmark. Move to position.
- **EXECUTE**: Execute goal.

After `EXECUTE`, for multiple goals (in functionality benchmarks)
`PREPARE` will be issued again until there are no more goals.

The robot can signal that `PREPARE` or `EXECUTE` are complete using
the services:

| Service                  | Type             | Notes                           |
|:------------------------:|:----------------:|:-------------------------------:|
| `/roah_rsbb/end_prepare` | `std_srvs/Empty` | Only available during `PREPARE` |
| `/roah_rsbb/end_execute` | `std_srvs/Empty` | Only available during `EXECUTE` |


#### Getting to know my home

No special topics or services.


#### Welcoming visitors

| Service                       | Type                        | Notes       |
|:-----------------------------:|:---------------------------:|:-----------:|
| `/roah_rsbb/activation_event` | `roah_rsbb_comm_ros/String` | Online data |
| `/roah_rsbb/notifications`    | `roah_rsbb_comm_ros/String` | Online data |
| `/roah_rsbb/visitor`          | `roah_rsbb_comm_ros/String` | Online data |


#### Catering for Granny Annie’s Comfort

| Service                           | Type                            | Notes         |
|:---------------------------------:|:-------------------------------:|:-------------:|
| `/roah_rsbb/devices/blinds/max`   | `std_srvs/Empty`                |               |
| `/roah_rsbb/devices/blinds/min`   | `std_srvs/Empty`                |               |
| `/roah_rsbb/devices/blinds/set`   | `roah_rsbb_comm_ros/Percentage` | 0 <= % <= 100 |
| `/roah_rsbb/devices/dimmer/max`   | `std_srvs/Empty`                |               |
| `/roah_rsbb/devices/dimmer/min`   | `std_srvs/Empty`                |               |
| `/roah_rsbb/devices/dimmer/set`   | `roah_rsbb_comm_ros/Percentage` | 0 <= % <= 100 |
| `/roah_rsbb/devices/switch_1/off` | `std_srvs/Empty`                |               |
| `/roah_rsbb/devices/switch_1/on`  | `std_srvs/Empty`                |               |
| `/roah_rsbb/devices/switch_1/set` | `roah_rsbb_comm_ros/Bool`       |               |
| `/roah_rsbb/devices/switch_2/off` | `std_srvs/Empty`                |               |
| `/roah_rsbb/devices/switch_2/on`  | `std_srvs/Empty`                |               |
| `/roah_rsbb/devices/switch_2/set` | `roah_rsbb_comm_ros/Bool`       |               |
| `/roah_rsbb/devices/switch_3/off` | `std_srvs/Empty`                |               |
| `/roah_rsbb/devices/switch_3/on`  | `std_srvs/Empty`                |               |
| `/roah_rsbb/devices/switch_3/set` | `roah_rsbb_comm_ros/Bool`       |               |
| `/roah_rsbb/tablet/buttons`       | `std_srvs/Empty`                |               |
| `/roah_rsbb/tablet/map`           | `std_srvs/Empty`                |               |
| `/roah_rsbb/final_command`        | `roah_rsbb_comm_ros/String`     | Online data   |


#### Object Perception Functionality

This benchmark has a result, therefore the `/roah_rsbb/end_execute`
service has a different type.

| Service                  | Type                            | Notes                           |
|:------------------------:|:-------------------------------:|:-------------------------------:|
| `/roah_rsbb/end_execute` | `roah_rsbb_comm_ros/ResultHOPF` | Only available during `EXECUTE` |

<!---
#### Object Manipulation Functionality

This benchmark has specific goals, therefore a new topic is needed.

| Topic             | Type                           | Notes                           |
|:-----------------:|:------------------------------:|:-------------------------------:|
| `/roah_rsbb/goal` | `roah_rsbb_comm_ros/Benchmark` | Latched, when goal is available |

Note that `/roah_rsbb/benchmark/state` and `/roah_rsbb/goal` change
almost simultaneously but in an undefined order. For this benchmark
alone you can assume that whenever a goal is published the state
has changed to `EXECUTE`. Therefore, you only need to receive the
`STOP` and `PREPARE` states on the state topic.
--->


#### Navigation Functionality
`TODO`

#### Speech Understanding Functionality

No special topics or services.


## Overriding benchmark selection for testing

The override service can be used to simulate RSBB input, for testing.

| Service               | Type                          | Notes            |
|:---------------------:|:-----------------------------:|:----------------:|
| `/roah_rsbb/override` | `roah_rsbb_comm_ros/Override` | Always available |

To know if the proper responses are being sent back to the RSBB you
have to use `rqt_logger_level` to set `ros.roah_rsbb_comm_ros` to the
Debug level. Then use `rqt_console` to see if the messages contain
the appropriate content. The robot states and benchmark states used
for communication are more complex than the ones presented here, but
they do not need to be fully understood to check the messages.
Still, full information is available here: https://github.com/joaocgreis/roah_rsbb_comm .

To revert the node to normal operation use:
```bash
rosservice call /roah_rsbb/override "{benchmark_type: 0}"
```

#### Getting to know my home

The benchmarks all follow the same structure.

- Start the benchmark in stop state:
```bash
rosservice call /roah_rsbb/override "{benchmark_type: 1, benchmark_state: 0}"
```

- Start preparing:
```bash
rosservice call /roah_rsbb/override "{benchmark_type: 1, benchmark_state: 1}"
```

- The robot should prepare and then call the end_prepare service as if by
`rosservice call /roah_rsbb/end_prepare`

- Send the goal (nothing in this case, just a start signal):
```bash
rosservice call /roah_rsbb/override "{benchmark_type: 1, benchmark_state: 2}"
```

- As soon as the RSBB detects the robot is executing, it should start
waiting for the result:
```bash
rosservice call /roah_rsbb/override "{benchmark_type: 1, benchmark_state: 3}"
```

The robot should run and then call the end_execute service as if by
`rosservice call /roah_rsbb/end_execute` to complete the goal.

For benchmarks with more goals, the RSBB asks to start preparing
until complete, then stop.

If a timeout occurs, a stop is sent.


#### Welcoming visitors

```bash
rosservice call /roah_rsbb/override "{benchmark_type: 2, benchmark_state: 0}"
rosservice call /roah_rsbb/override "{benchmark_type: 2, benchmark_state: 1}"
rosservice call /roah_rsbb/override "{benchmark_type: 2, benchmark_state: 2}"
rosservice call /roah_rsbb/override "{benchmark_type: 2, benchmark_state: 3}"
```


#### Catering for Granny Annie’s Comfort

```bash
rosservice call /roah_rsbb/override "{benchmark_type: 3, benchmark_state: 0}"
rosservice call /roah_rsbb/override "{benchmark_type: 3, benchmark_state: 1}"
rosservice call /roah_rsbb/override "{benchmark_type: 3, benchmark_state: 2}"
rosservice call /roah_rsbb/override "{benchmark_type: 3, benchmark_state: 3}"
```


#### Object Perception Functionality

```bash
rosservice call /roah_rsbb/override "{benchmark_type: 4, benchmark_state: 0}"
rosservice call /roah_rsbb/override "{benchmark_type: 4, benchmark_state: 1}"
rosservice call /roah_rsbb/override "{benchmark_type: 4, benchmark_state: 2}"
rosservice call /roah_rsbb/override "{benchmark_type: 4, benchmark_state: 3}"
```

The goal should be completed with something like:

```bash
rosservice call /roah_rsbb/end_execute "{object_class: cups, object_name: red_cup, object_pose: {x: 0.1, y: 0.2, theta: 1.23} }"
```

<!--
#### Object Manipulation Functionality

```bash
rosservice call /roah_rsbb/override "{benchmark_type: 5, benchmark_state: 0}"
rosservice call /roah_rsbb/override "{benchmark_type: 5, benchmark_state: 1}"
rosservice call /roah_rsbb/override "{benchmark_type: 5, benchmark_state: 2, initial_state: [false, true, false, true, true, false, true, false, false, true], switches: [4, 3, 1, 7, 8]}"
rosservice call /roah_rsbb/override "{benchmark_type: 5, benchmark_state: 3}"
```
-->

#### Speech Understanding Functionality

```bash
rosservice call /roah_rsbb/override "{benchmark_type: 6, benchmark_state: 0}"
rosservice call /roah_rsbb/override "{benchmark_type: 6, benchmark_state: 1}"
rosservice call /roah_rsbb/override "{benchmark_type: 6, benchmark_state: 2}"
rosservice call /roah_rsbb/override "{benchmark_type: 6, benchmark_state: 3}"
```
