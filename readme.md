# hand_safety_pkg — Inspire Hand Safety Layer

A ROS 2 package that filters unsafe commands before they reach the Inspire dexterous hand. It sits between the teleoperation controller and the hand driver, intercepting dangerous angle/force/velocity commands in real time.

## Architecture

```
Teleoperation UI
      │
      ▼
/safe/inspire_hand/raw/cmd/l(r)    ← raw commands (input)
      │
      ▼
┌──────────────────┐
│ hand_safety_node │   ← C++ node, filters through 4 safety rules
│ (or .py version) │
└──┬───────────┬───┘
   │           │
   ▼           ▼
/inspire_hand/        /safe/inspire_hand/
 ctrl/l(r)             trigger
 (filtered output)     (String, rule events)
                           │
                           ▼
                    ┌─────────────────────┐
                    │ hand_record_node     │  ← Python node, logs on trigger
                    │ (sliding window log) │
                    └─────────────────────┘
```

## Safety Rules

| Rule | Condition | Action |
|------|-----------|--------|
| **Rule 1** (Low-angle lock) | Joints 3/4/5 all in [0, 200] for 3 consecutive frames | Clamp joints 3/4/5 to 200 |
| **Rule 2** (Overload protection) | Any joint `force_act > 3000` | Overwrite that joint with previous frame's value |
| **Rule 3** (Boundary clamp) | Any joint angle outside [0, 1000] | Clamp to 0 or 1000 |
| **Rule 4** (Smoothing) | Any joint's adjacent-frame difference > 200 within a 5-frame window | Replace with 5-frame average |

Execution order: Rule 1 → Rule 2 → Rule 3 → Rule 4.

## Nodes

### hand_safety_node (C++)

The core safety filter node. Subscribes to raw commands and hand state, applies the 4 rules, publishes filtered commands.

**Subscribes:**
- `/safe/inspire_hand/raw/cmd/{l,r}` (`InspireHandCtrl`)
- `/inspire_hand/state/{l,r}` (`InspireHandState`)

**Publishes:**
- `/inspire_hand/ctrl/{l,r}` (`InspireHandCtrl`) — filtered output
- `/safe/inspire_hand/trigger` (`String`) — rule trigger events

### hand_safety_record_node (Python)

A recording node that maintains a 10-frame sliding window on all 4 data topics. When a rule trigger is detected, it dumps the last 5 frames × 2 topics × 2 hands = 20 frames to a timestamped `.log` file.

**Subscribes:**
- `/safe/inspire_hand/raw/cmd/{l,r}`
- `/inspire_hand/ctrl/{l,r}`
- `/safe/inspire_hand/trigger`

**Parameter:**
- `log_dir` (string, default `~/safety_logs`) — directory for log files

## Build

```bash
cd hand_safety_ws
colcon build --packages-select hand_safety_pkg
source install/setup.bash
```

## Run

```bash
# Terminal 1: safety node
ros2 run hand_safety_pkg hand_safety_node

# Terminal 2: record node (optional)
ros2 run hand_safety_pkg hand_safety_record_node
# or specify a custom log directory:
ros2 run hand_safety_pkg hand_safety_record_node --ros-args -p log_dir:=/tmp/my_logs
```

## Testing

```bash
# Make sure hand_safety_node is running first
python3 src/hand_safety_pkg/test/bug_test.py
```

44 test cases covering:
- Single-rule edge cases (Rule 1-4)
- Cross-rule interactions (C1-C5)
- System boundary tests (empty input, short arrays, left/right isolation, high frequency, state interruption)

## Log Output

Trigger logs are saved to `~/safety_logs/safety_trigger_YYYYMMDD_HHMMSS_NNNN.log`. Each log contains:
1. Trigger info (rule, side, timestamp)
2. 20 frames of raw data (angle/force/pos/speed/mode)
3. Input vs output diff analysis highlighting modified joints
