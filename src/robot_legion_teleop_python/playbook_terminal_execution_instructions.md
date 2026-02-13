# Playbook Terminal Execution Instructions

This guide shows how to run playbooks from a laptop terminal against multiple robots, starting from fresh terminals and fresh SSH sessions.

Assumptions:
- ROS 2 Jazzy on Ubuntu 24.04.
- Robot name is derived from Linux username (`$USER`) by `robot_bringup.launch.py`.
- Linux username on each robot should match a key in `config/robot_profiles.yaml` (for example: `robot1`, `robot2`, `robot3`).
- All machines are on the same network and use the same ROS domain ID.

## 1) On each robot Pi (fresh SSH session)

Open one SSH terminal per robot, then run:

```bash
cd "$HOME/ros2_ws"
source /opt/ros/"${ROS_DISTRO:-jazzy}"/setup.bash
colcon build --packages-select robot_legion_teleop_python fleet_orchestrator_interfaces
source "$HOME/ros2_ws/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export PROFILES_PATH="${PROFILES_PATH:-$HOME/ros2_ws/src/robot_legion_teleop_python/config/robot_profiles.yaml}"
```

Important:
- If a shell already has `ROS_DOMAIN_ID=17`, the `:-42` pattern will keep `17`.
- To force consistent values every time, use this instead:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export PROFILES_PATH="$HOME/ros2_ws/src/robot_legion_teleop_python/config/robot_profiles.yaml"
```

Note:
- `ROBOT_NAME` is not consumed by current `robot_bringup.launch.py`.
- Runtime robot namespace/action path is based on Linux username (`$USER`), e.g. `/<user>/execute_playbook`.
- If `$USER` does not exist in `robot_profiles.yaml`, motor driver may fall back to mock mode and the robot will not physically move.

Verify robot shell environment:

```bash
env | grep -E 'ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|ROS_AUTOMATIC_DISCOVERY_RANGE|ROS_STATIC_PEERS|RMW_IMPLEMENTATION|ROS_DISTRO|USER'
```

Launch robot nodes (motor driver, heartbeat, executor):

```bash
ros2 launch robot_legion_teleop_python robot_bringup.launch.py \
  profiles_path:="$PROFILES_PATH" \
  use_camera:=false
```

Expected in logs:
- `UnitExecutor ready`
- `action=/<robot_name>/execute_playbook`

## 2) On laptop (fresh terminal)

```bash
cd "$HOME/ros2_ws"
source /opt/ros/"${ROS_DISTRO:-jazzy}"/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
```

To force known-good values (recommended), run:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

Verify laptop shell environment:

```bash
env | grep -E 'ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|ROS_AUTOMATIC_DISCOVERY_RANGE|ROS_STATIC_PEERS|RMW_IMPLEMENTATION|ROS_DISTRO|USER'
```

Discover action servers:

```bash
mapfile -t ACTIONS < <(ros2 action list | grep -E '^/.+/execute_playbook$')
echo "Found ${#ACTIONS[@]} execute_playbook servers"
printf '%s\n' "${ACTIONS[@]}"
```

If count is `0`, check:
- Same `ROS_DOMAIN_ID` on laptop and all robots.
- Robot launch logs show executor started.

## 3) Example playbook commands

### Example A: all robots move forward (transit primitive)

What it does:
- Sends `command_id: transit` to each discovered robot.
- Uses `direction=forward`, `duration_s=2.0`, `speed=0.8`.

```bash
for action in "${ACTIONS[@]}"; do
  robot="${action%/execute_playbook}"; robot="${robot#/}"
  ros2 action send_goal --feedback "$action" fleet_orchestrator_interfaces/action/ExecutePlaybook \
    "{intent_id: test_transit_${robot}, command_id: transit, parameters_json: \"{\\\"direction\\\":\\\"forward\\\",\\\"duration_s\\\":2.0,\\\"speed\\\":0.8}\"}" &
done
wait
```

### Example B: all robots move backward (same primitive, reverse direction)

What it does:
- Same as Example A, but `direction=backward`.

```bash
for action in "${ACTIONS[@]}"; do
  robot="${action%/execute_playbook}"; robot="${robot#/}"
  ros2 action send_goal --feedback "$action" fleet_orchestrator_interfaces/action/ExecutePlaybook \
    "{intent_id: test_transit_back_${robot}, command_id: transit, parameters_json: \"{\\\"direction\\\":\\\"backward\\\",\\\"duration_s\\\":2.0,\\\"speed\\\":0.8}\"}" &
done
wait
```

### Example C: all robots move forward, then backward after forward completes

What it does:
- First dispatches forward transit to all robots and waits for completion.
- Then dispatches backward transit to all robots.

```bash
for action in "${ACTIONS[@]}"; do
  (
    robot="${action%/execute_playbook}"; robot="${robot#/}"

    # forward for this robot
    ros2 action send_goal --feedback "$action" fleet_orchestrator_interfaces/action/ExecutePlaybook \
      "{intent_id: test_seq_fwd_${robot}, command_id: transit, parameters_json: \"{\\\"direction\\\":\\\"forward\\\",\\\"duration_s\\\":2.0,\\\"speed\\\":0.8}\"}"

    # then backward for this same robot
    ros2 action send_goal --feedback "$action" fleet_orchestrator_interfaces/action/ExecutePlaybook \
      "{intent_id: test_seq_back_${robot}, command_id: transit, parameters_json: \"{\\\"direction\\\":\\\"backward\\\",\\\"duration_s\\\":2.0,\\\"speed\\\":0.8}\"}"
  ) &
done
wait

```

### Example D: all robots rotate left ~45 degrees, then move forward 1.5 seconds

What it does:
- Step 1: rotate left.
- Step 2: move forward.

Notes:
- Rotation command is duration-based, not angle-based.
- With default rotate speed `2.0 rad/s`, 45 degrees (`pi/4`) is about `0.39 s`.

```bash
# Step 1: rotate left ~45 deg (duration-based approximation)
for action in "${ACTIONS[@]}"; do
  robot="${action%/execute_playbook}"; robot="${robot#/}"
  ros2 action send_goal --feedback "$action" fleet_orchestrator_interfaces/action/ExecutePlaybook \
    "{intent_id: test_rot45_${robot}, command_id: rotate, parameters_json: \"{\\\"direction\\\":\\\"left\\\",\\\"duration_s\\\":0.39,\\\"speed\\\":1.0}\"}" &
done
wait

# Step 2: move forward 1.5 s
for action in "${ACTIONS[@]}"; do
  robot="${action%/execute_playbook}"; robot="${robot#/}"
  ros2 action send_goal --feedback "$action" fleet_orchestrator_interfaces/action/ExecutePlaybook \
    "{intent_id: test_after_rot_fwd_${robot}, command_id: transit, parameters_json: \"{\\\"direction\\\":\\\"forward\\\",\\\"duration_s\\\":1.5,\\\"speed\\\":0.8}\"}" &
done
wait
```

## 4) Optional: monitor action and robot state

On laptop:

```bash
ros2 action list | grep execute_playbook
```

On a robot terminal:

```bash
tail -f /tmp/robot_"${USER}"_audit.jsonl
```
