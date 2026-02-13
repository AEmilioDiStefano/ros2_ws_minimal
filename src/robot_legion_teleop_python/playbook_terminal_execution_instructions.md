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
colcon build --packages-select robot_legion_teleop_python fleet_orchestrator_interfaces
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

## 3) Run `terminal_orchestrator.py` (recommended)

Start the terminal UI:

```bash
ros2 run robot_legion_teleop_python terminal_orchestrator
```

You should see:
- `Detected robots: <N>`
- robot list under `Robots:`

UI controls:
- `t` choose target robots (`all` or one robot)
- `1` Coordinated 3-robot XY planner (randomized 2-leg linear algebra)
- `2` Execute ALL commands (validation sweep)
- `3` Transit distance (meters -> duration estimate)
- `4` Rotate degrees (degrees -> duration estimate)
- `r` refresh discovery
- `q` quit

Execution flow:
1. Choose target robots.
2. Choose a playbook.
3. Enter requested values.
4. Review `EXECUTION SUMMARY`.
5. Confirm with `y` to dispatch.

Notes:
- Dispatch is sent to all selected robots in parallel (simultaneous start intent).
- Distance/time is intentionally approximate and consistent across robots.
- For `Move objective (x, y)`: `x` maps to forward/back, `y` maps to right/left.

## 4) Manual CLI playbook commands (alternative)

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

### Example E: all robots move to an objective point (`transit_xy`)

What it does:
- Sends the same objective-point command to each robot (`north_m`, `east_m`).
- Robot-specific motion strategy is selected robot-side from drive/hardware profile.
- Current field strategy executes axis legs using forward+rotate behavior (no strafe dependency):
  first leg, then ~90 deg rotate, then second leg.

Notes:
- `east_m > 0` means objective is to robot-right.
- `east_m < 0` means objective is to robot-left.

```bash
for action in "${ACTIONS[@]}"; do
  robot="${action%/execute_playbook}"; robot="${robot#/}"
  ros2 action send_goal --feedback "$action" fleet_orchestrator_interfaces/action/ExecutePlaybook \
    "{intent_id: test_objective_xy_${robot}, command_id: transit_xy, parameters_json: \"{\\\"north_m\\\":1.5,\\\"east_m\\\":-0.75,\\\"speed\\\":1.0}\"}" &
done
wait
```

Design guidance:
- Keep terminal commands platform-agnostic when possible (single `transit_xy` contract).
- Put per-drive behavior (for example, mecanum forward-then-strafe vs differential axis/heading strategy) in playbook strategy code, not shell branching.

## 5) Optional: monitor action and robot state

On laptop:

```bash
ros2 action list | grep execute_playbook
```

On a robot terminal:

```bash
tail -f /tmp/robot_"${USER}"_audit.jsonl
```

## 6) Interpreting `terminal_orchestrator` JSON output

After you confirm `EXECUTE? y/n`, `terminal_orchestrator` prints structured JSON events.
These are intended to be copy/paste friendly for future Fleet Orchestrator integration work.

Event types you will see:
- `type: "playbook_command"`: dispatch intent per robot (task-style event).
- `type: "audit_event"`: execution outcome per robot (result-style event).

For option `1` specifically, you will see:
- one high-level planning event with `command_id: "transit_xy_randomized_formation"`
- then per-phase dispatch events (`rotate`, `transit`, `rotate`, `transit`) for each robot
- then per-phase `audit_event` results for each robot

### A) Task-style dispatch event (`playbook_command`)

Use this to verify command mapping before execution:
- `intent_id`: unique command instance ID.
- `command_id`: primitive being sent (`transit`, `rotate`, `transit_xy`, ...).
- `targets`: robot receiving the command.
- `parameters`: normalized command values sent to robot executor.
- `preview.strategy_id`: strategy selected for that robot type.
- `preview.steps`: ordered movement phases (direction + duration).

Example:

```json
{
  "command_id": "transit_xy",
  "intent_id": "term_xy_robot2_1770000000",
  "parameters": {
    "east_m": 0.5,
    "north_m": 0.2,
    "speed": 1.0
  },
  "platform": {
    "adapter_type": "terminal_orchestrator",
    "platform_family": "robot_legion_teleop_python"
  },
  "preview": {
    "cardinal_mode": "relative_fallback",
    "drive_type": "mecanum_drive",
    "hardware": "dual_tb6612_mecanum",
    "notes": [
      "north/east uses robot body frame without heading sensor",
      "robots with different starting headings diverge in world frame"
    ],
    "profile_name": "mecanum_drive",
    "profile_source": "registry",
    "robot": "robot2",
    "steps": [
      {
        "duration_s": 0.5,
        "index": 1,
        "motion": "forward",
        "status_text": "omni forward leg 0.20m"
      },
      {
        "duration_s": 0.785,
        "index": 2,
        "motion": "rotate_right",
        "status_text": "omni rotate to lateral leg"
      },
      {
        "duration_s": 1.25,
        "index": 3,
        "motion": "forward",
        "status_text": "omni lateral leg right=0.50m"
      }
    ],
    "strategy_id": "omni_axis_turn_xy"
  },
  "targets": "robot2",
  "trace": {
    "confidence": 1.0,
    "explanation": "manual terminal playbook dispatch",
    "translator": "terminal_orchestrator"
  },
  "type": "playbook_command"
}
```

### B) Audit-style result event (`audit_event`)

Use this to verify execution completion:
- `status`: `succeeded` or `failed`.
- `accepted`: whether the robot action server accepted execution.
- `reason`: robot-side result reason string.
- `intent_id`: links result back to the original dispatch event.

Example:

```json
{
  "accepted": true,
  "intent_id": "term_xy_robot2_1770000000",
  "reason": "ok (omni_axis_turn_xy)",
  "robot": "robot2",
  "stage": "execute_playbook_dispatch",
  "status": "succeeded",
  "type": "audit_event"
}
```

### C) Optional topic publishing mode

By default, JSON is printed to terminal only.
To also publish JSON to Fleet-Orchestrator-style topics:

```bash
ros2 run robot_legion_teleop_python terminal_orchestrator --ros-args -p publish_fleet_topics:=true
```

Then monitor:

```bash
ros2 topic echo /fo/task
ros2 topic echo /fo/audit
```

## 7) Coordinated 3-Robot XY mode (option `1`)

`1) Move objective (x,y)` now runs a coordinated planner intended for three robots.

What it does:
1. Requires exactly 3 selected robots.
2. Asks for:
   - base objective (`x`, `y`)
   - speed scale
   - robot spacing (default `0.45 m`)
   - formation direction in radians (relative to anchor robot)
   - turn side (`left` or `right`) shared by all robots
   - `MAX PATH OFFSET m` (default `0.8`)
   - random seed (or auto)
   - anchor robot index
3. Uses linear algebra + randomized heading pairs to compute two forward legs per robot.
   - A shared second heading is enforced so all robots finish facing the same direction.
4. Executes in synchronized phases:
   - phase 1: rotate
   - phase 2: forward leg 1
   - phase 3: rotate
   - phase 4: forward leg 2

### Linear algebra summary

For each robot, the planner solves:

```text
l1 * [cos(theta1), sin(theta1)] + l2 * [cos(theta2), sin(theta2)] = [dx, dy]
```

Where:
- `[dx, dy]` is that robot's destination vector in robot-relative frame.
- `theta1`, `theta2` are randomized heading angles (same turn side for all robots).
- `theta2` is shared across robots in a run (common final facing direction).
- `l1`, `l2` are segment lengths solved from the 2x2 linear system.

The planner rejects a sampled solution if:
- matrix is singular,
- any leg length is too small/non-forward,
- detour from straight-line path exceeds `MAX PATH OFFSET m`.

### Why random seed is included

The random seed changes the sampled heading pairs, so trajectories vary while still
respecting:
- same turn-side constraint,
- max detour bound,
- exact endpoint solution of the linear system.

Use a fixed seed to reproduce the same trajectory set later.

### Important frame note

All vectors are robot-relative unless heading/global mode is added later.
So this mode is best used when robots are staged with aligned initial heading.
