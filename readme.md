# hand_safety_pkg - Inspire Hand Safety Layer

ROS 2 safety package for the Inspire dexterous hand. It places a software safety layer between teleoperation/control sources and the real hand driver, filters unsafe commands, supports a gamepad-triggered emergency stop, and can announce stop/clear states through the Unitree G1 voice API.

## Architecture

```plain
Teleoperation / console / other command sources
      |
      v
/safe/inspire_hand/raw/cmd/{l,r}        InspireHandCtrl
      |
      v
hand_safety_node
      | normal mode: filtered command
      | estop mode: blocks raw commands and publishes open-hand commands
      v
/inspire_hand/ctrl/{l,r}                InspireHandCtrl

Unitree G1 /lowstate
      |
      v
hand_gamepad_estop
      | short press L1+R1: true
      | press F1: false
      v
/safe/inspire_hand/estop                std_msgs/Bool
      |
      +--> hand_safety_node             latch/clear hand estop
      |
      +--> hand_safety_voice_node       TTS prompt through /api/voice/request

/safe/inspire_hand/trigger              std_msgs/String
      |
      v
hand_safety_record_node                 trigger-window logs
```

Important: `/inspire_hand/ctrl/{l,r}` should have a single final publisher in normal use: `hand_safety_node`. Any other node that publishes directly to `/inspire_hand/ctrl/{l,r}` can bypass the safety layer and may override estop behavior.

## Command Message

`InspireHandCtrl` uses bounded 6-element arrays:

```plain
pos_set
angle_set
force_set
speed_set
mode
```

The open-hand estop command is:

```plain
pos_set   = [0, 0, 0, 0, 0, 0]
angle_set = [1000, 1000, 1000, 1000, 1000, 1000]
force_set = [3000, 3000, 3000, 3000, 3000, 3000]
speed_set = [1000, 1000, 1000, 1000, 1000, 1000]
mode      = 0b0001
```

`0b0001` means angle-control mode.

## Safety Rules

`hand_safety_node` filters raw commands in this order:

| Rule | Condition | Action |
|------|-----------|--------|
| Rule 1: low-angle lock | Joints 3/4/5 are all in `[0, 200]` for 3 consecutive frames | Clamp joints 3/4/5 to `200` |
| Rule 2: overload protection | Any joint `force_act > 3000` | Overwrite that joint with the previous output frame |
| Rule 3: boundary clamp | Any `angle_set` outside `[0, 1000]` | Clamp to `0` or `1000` |
| Rule 4: smoothing | Any adjacent-frame angle jump `> 200` within a 5-frame window | Replace that joint with the 5-frame average |

When a safety rule triggers, the node publishes a `String` event on `/safe/inspire_hand/trigger`.

## Emergency Stop

The hand estop path has two parts:

- `hand_gamepad_estop` subscribes to Unitree G1 `/lowstate`, parses `wireless_remote[40]`, and publishes `/safe/inspire_hand/estop`.
- `hand_safety_node` subscribes to `/safe/inspire_hand/estop`, latches the estop state, blocks normal raw commands, and sends 5 open-hand frames at 50 Hz.

Gamepad actions:

| Action | Effect |
|--------|--------|
| Short press `L1+R1` | Publish `/safe/inspire_hand/estop=true` |
| Hold `L1+R1` for 2 seconds or longer | Reserved for the robot damping/estop node; hand short-press estop will not fire |
| Press `F1` | Publish `/safe/inspire_hand/estop=false` and clear the hand estop latch |

Manual estop control:

```bash
ros2 topic pub --once /safe/inspire_hand/estop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /safe/inspire_hand/estop std_msgs/msg/Bool "{data: false}"
```

## Voice Prompt

`hand_safety_voice_node` subscribes to `/safe/inspire_hand/estop`.

| Estop state | TTS text |
|-------------|----------|
| `true` | `灵巧手急停已触发` |
| `false` | `灵巧手急停已解除` |

It sends Unitree G1 voice requests to:

```plain
/api/voice/request
```

and listens for results on:

```plain
/api/voice/response
```

The TTS API id is `1001`. The request `parameter` is JSON with `index`, `text`, and `speaker_id`.

## Nodes

### hand_safety_node

Subscribes:

- `/safe/inspire_hand/raw/cmd/{l,r}` (`InspireHandCtrl`)
- `/inspire_hand/state/{l,r}` (`InspireHandState`)
- `/safe/inspire_hand/estop` (`std_msgs/Bool`)

Publishes:

- `/inspire_hand/ctrl/{l,r}` (`InspireHandCtrl`)
- `/safe/inspire_hand/trigger` (`std_msgs/String`)

### hand_gamepad_estop

Subscribes:

- `/lowstate` (`unitree_hg/msg/LowState`)

Publishes:

- `/safe/inspire_hand/estop` (`std_msgs/Bool`)

Parameters:

- `lowstate_topic`, default `/lowstate`
- `estop_topic`, default `/safe/inspire_hand/estop`
- `short_press_min_seconds`, default `0.05`
- `long_press_seconds`, default `2.0`

### hand_safety_voice_node

Subscribes:

- `/safe/inspire_hand/estop` (`std_msgs/Bool`)
- `/api/voice/response` (`unitree_api/msg/Response`)

Publishes:

- `/api/voice/request` (`unitree_api/msg/Request`)

Parameters:

- `estop_topic`, default `/safe/inspire_hand/estop`
- `voice_request_topic`, default `/api/voice/request`
- `voice_response_topic`, default `/api/voice/response`
- `speaker_id`, default `0`

### hand_safety_record_node

Subscribes:

- `/safe/inspire_hand/raw/cmd/{l,r}`
- `/inspire_hand/ctrl/{l,r}`
- `/safe/inspire_hand/trigger`

Parameter:

- `log_dir`, default `~/safety_logs`

## Build

For the gamepad and voice nodes, source the Unitree ROS 2 environment first so `unitree_hg` and `unitree_api` are available.

```bash
source <your_repo_root>/unitree_ros2/setup.sh
cd <your_repo_root>/ztx_dexhand_console/hand_safety_ws
colcon build --packages-select hand_safety_pkg
source install/setup.bash
```

## Run

```bash
# Terminal 1: safety filter and estop executor
source <your_repo_root>/unitree_ros2/setup.sh
source <your_repo_root>/ztx_dexhand_console/hand_safety_ws/install/setup.bash
ros2 run hand_safety_pkg hand_safety_node

# Terminal 2: Unitree gamepad estop bridge
source <your_repo_root>/unitree_ros2/setup.sh
source <your_repo_root>/ztx_dexhand_console/hand_safety_ws/install/setup.bash
ros2 run hand_safety_pkg hand_gamepad_estop

# Terminal 3: G1 voice prompts, optional but recommended on real robot
source <your_repo_root>/unitree_ros2/setup.sh
source <your_repo_root>/ztx_dexhand_console/hand_safety_ws/install/setup.bash
ros2 run hand_safety_pkg hand_safety_voice_node

# Terminal 4: trigger-window logging, optional
source <your_repo_root>/ztx_dexhand_console/hand_safety_ws/install/setup.bash
ros2 run hand_safety_pkg hand_safety_record_node
```

## Useful Checks

```bash
ros2 topic hz /lowstate
ros2 topic echo /safe/inspire_hand/estop
ros2 topic echo /api/voice/response
ros2 pkg executables hand_safety_pkg
```

## Tests

```bash
cd <your_repo_root>/ztx_dexhand_console/hand_safety_ws
source install/setup.bash
python3 src/hand_safety_pkg/test/bug_test.py
```

The existing tests cover rule edge cases, cross-rule interactions, short/empty input, left/right isolation, high-frequency input, and state interruption.

## Logs

Trigger logs are saved to `~/safety_logs/safety_trigger_YYYYMMDD_HHMMSS_NNNN.log` by default. Each log contains:

1. Trigger information
2. Recent raw/output frames for both hands
3. Input vs output diff analysis
