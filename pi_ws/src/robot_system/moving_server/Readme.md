# moving_server

Single motion gateway of the robot. Everything that makes the base move
goes through one action; the server picks the executor matching the mode
of the goal and publishes on the `cmd_vel_mux` input matching that
behaviour.

The base is a **mecanum drive**: all closed loop controllers here are
holonomic, they command `linear.x`, `linear.y` and `angular.z`.

```
             /moving_server/move_command  (MoveCommandAction)
                              │
              ┌───────────────┼────────────────┬───────────────┐
              ▼               ▼                ▼               ▼
      MOVE_TO_POINT     TRACK_PERSON        DOCKING        UNDOCKING
              │               │                │               │
        move_base       person pose      1. move_base     straight
        action client   topic (sub)         → pre-dock     translation
              │               │           2. DetectDock       out of
              │               │              service          the dock
              │               ▼           3. trajectory        │
              │        PersonFollow          tracking          │
              │        Controller               │              │
              ▼               ▼                 ▼              ▼
      /cmd_vel/navi   /cmd_vel/tracking   /cmd_vel/docking (+ same)
              └───────────────┴─────────────────┴──────────────┘
                                     │
                                cmd_vel_mux
                                     │
                                  /cmd_vel
```

## Packages

| Package | Content |
| --- | --- |
| `moving_server_msgs` | `MoveCommand.action`, `DetectDock.srv`, `MovingServerState.msg` |
| `moving_server` | node, executors, control algorithms |

Interfaces live in their own package so a client (fleet manager, UI,
dock detector) can depend on them without pulling in the implementation.

## Interface

### Action `/moving_server/move_command`

**Goal**

| Field | Used by | Meaning |
| --- | --- | --- |
| `mode` | all | `MODE_MOVE_TO_POINT`, `MODE_TRACK_PERSON`, `MODE_DOCKING`, `MODE_UNDOCKING` |
| `target_pose` | move to point, docking | goal pose / pre-dock pose (docking falls back to the `docks` parameter) |
| `target_topic` | track person | `PoseStamped` topic of the person, empty → `~person_pose_topic` |
| `follow_distance` | track person | distance to keep [m], 0 → parameter |
| `hold_time` | track person | success once the distance is held that long, 0 → track until preempted |
| `dock_id` | docking | forwarded to the detection service |
| `timeout` | all | 0 → `~default_timeout` |

**Result**: `success`, `result_code` (`RESULT_*`), `message`, `final_pose`.

**Feedback**: `state` / `state_name` (`MOVE_BASE`, `TRACKING_PERSON`,
`GOTO_PREDOCK`, `DETECTING_DOCK`, `TRAJ_TRACKING`, `UNDOCKING`, …),
`distance_remaining`, `heading_error`, `progress`.

Only one goal runs at a time: sending a new one preempts the current one.

**Stopping a running goal** — three equivalent ways:

```bash
rosservice call /moving_server/stop            # or /emergency_stop
rostopic pub -1 /moving_server/move_command/cancel actionlib_msgs/GoalID '{}'
# or client.cancel_goal() from your own action client
```

All three end up cancelling the underlying `move_base` goal too. That
matters for `MOVE_TO_POINT` and for the pre-dock leg: `move_base`
publishes its own velocity on its own mux input, so zeroing the topics
of the moving_server alone would not stop the robot — its goal has to be
cancelled.

### Service `~detect_dock_service` (`DetectDock`)

Implemented **outside** this package by the charger detector. Returns the
dock pose, the approach path to follow and a confidence. A mock
implementation for bring-up is provided in
`scripts/mock_dock_detector_node.py`.

### Other interfaces

| Name | Type | Role |
| --- | --- | --- |
| `/moving_server/state` | `MovingServerState` | latched state of the server |
| `/moving_server/stop` | `std_srvs/Trigger` | stops the running goal, whatever its mode |
| `/moving_server/docking/align_path` | `nav_msgs/Path` | leg 1 of the approach, latched, odom frame |
| `/moving_server/docking/approach_path` | `nav_msgs/Path` | leg 2, the straight run in |
| `/moving_server/docking/dock_pose` | `geometry_msgs/PoseStamped` | charger as detected |
| `/moving_server/docking/align_target` | `geometry_msgs/PoseStamped` | target 1, the alignment point |
| `/moving_server/docking/final_target` | `geometry_msgs/PoseStamped` | target 2, end of the approach |
| `/moving_server/emergency_stop` | `std_srvs/Trigger` | stops the motion, aborts the goal, rejects new ones |
| `/moving_server/clear_emergency_stop` | `std_srvs/Trigger` | releases it |
| `~odom_topic` | `nav_msgs/Odometry` | pose and measured velocity of the closed loop controllers |
| `~reset_odom_service` | `std_srvs/Trigger` or `Empty` | odometry reset of the base driver, called before the dock approach |

Odometry is read from the **topic**, not from TF, because the driver
resets it mid-sequence and the topic is what carries the new origin —
together with the measured velocity.

**When the odometry source cannot be reset** (Gazebo, or a driver
without that service), run the relay:

```bash
roslaunch moving_server moving_server.launch use_odom_reset_relay:=true
```

`odom_reset_node.py` subscribes to the raw `/odom`, republishes it on
`/odom_reset` relative to the origin captured at the last reset, and
serves `/reset_odom` itself. It broadcasts `odom → odom_reset` so RViz
still displays the docking paths, and touches nothing else in the TF
tree. The launch file then points moving_server at the relayed topic and
frame.

## Modes

### MODE_MOVE_TO_POINT

Pure forwarding to `move_base`: the moving_server stays the single entry
point, `move_base` keeps doing the planning and publishes its own
velocity on the navigation input of the mux.

> For a mecanum base, `move_base` must be configured accordingly:
> `holonomic_robot: true` on the local planner (`base_local_planner`) or
> a holonomic planner such as `teb_local_planner` with
> `max_vel_y > 0`. Otherwise strafing is only used by the two closed loop
> modes below.

### MODE_TRACK_PERSON

Sequence: subscribe to the person topic → transform the pose into
`~base_frame` → run the tracking algorithm → publish on
`/cmd_vel/tracking`.

Algorithm: `controllers/person_follow_controller.py`, a **point to
point** regulator. It knows only the current target position, has no
path and no history. On a mecanum base it translates straight to the
standoff point (`vx`, `vy` at once) while rotating to face the person,
instead of the turn-then-drive behaviour of a differential robot. Set
`person_tracking/holonomic: false` for that fallback.

Aborts with `RESULT_TARGET_LOST` if no pose arrives for
`person_tracking/target_timeout`.

### MODE_DOCKING

```
DETECTING_DOCK DetectDock service (retries, confidence check); the dock
               pose is frozen in the robot frame
RESET_ODOM     odometry reset of the base driver
ALIGNING       leg 1: path to the alignment point, align_distance
               (40-50 cm) in front of the charger, parallel to it
TRAJ_TRACKING  leg 2: straight path into the charger
```

**The robot must already be in the pre-dock area**: getting there is a
separate `MOVE_TO_POINT` goal, not part of this mode. A robot that is
already a few dozen centimetres from the charger has no business making
a move_base plan, and only the caller knows whether that leg is needed
at all — so the caller sequences the two goals.

The whole mode runs **in the odom frame**, on the odometry topic. The
dock pose is captured in the robot frame *before* the reset — the only
frame both sides of the reset agree on — then re-expressed in the fresh
odom frame, so the approach carries none of the drift accumulated during
the navigation before it.

Algorithm for both legs: `controllers/trajectory_tracking_controller.py`,
a **trajectory tracking** regulator — a different problem from person
following: the robot has to stay *on the path* and finish on an exact
pose. Because a mecanum base decouples position and orientation, the
controller drives the translation towards a look-ahead point, corrects
the cross track error sideways, and holds the dock orientation the whole
way instead of steering into it. The measured velocity from the odometry
seeds the slew rate limiter.

Splitting the approach in two legs is what keeps the final run straight:
leg 1 absorbs the whole lateral offset and the alignment (it is tuned
loosely, `align_tracking`), so leg 2 enters the charger with a tight
`max_cross_track_error` and no sideways motion left. Both abort with
`RESULT_TRACKING_FAIL` past that limit.

Both targets are rebuilt at runtime from the *detected* dock pose, whose
yaw points **outwards**, away from the charger:
`docking/align_distance` for the alignment point,
`docking/final_offset` for how far short of the dock pose to stop.
`~docks/<dock_id>/dock_pose` is not used by this mode — it is the map of
the chargers, for the detector and for whoever sends the `MOVE_TO_POINT`
goal to the pre-dock area. With `docking/reverse: true` the
robot keeps its back to the charger for the whole approach — a mecanum
base never has to turn around.

### MODE_UNDOCKING

Straight translation out of the dock over `docking/undock_distance`,
heading untouched — a mecanum base does not need to turn around.

## Adding a mode

1. add the value to `MoveCommand.action`
2. write an executor in `src/moving_server/executors/`, implementing
   `start(goal)`, `update(dt)` and `stop()` from `executors/base.py`
3. register it in `MODE_NAMES` and `self.executors` of the node

Executors are non blocking state machines: the node calls `update(dt)`
at `~control_rate` and turns the outcome into feedback / result, so
preemption, timeout and emergency stop are handled once for every mode.
Control algorithms stay in `controllers/`, ROS free and unit testable.

## Run

```bash
roslaunch moving_server moving_server.launch use_mock_dock_detector:=true

rosrun moving_server move_command_client.py point 1.0 2.0 0.0
rosrun moving_server move_command_client.py track --distance 1.2
rosrun moving_server move_command_client.py dock dock_01
rosrun moving_server move_command_client.py undock
```

Parameters: `config/moving_server.yaml`.
