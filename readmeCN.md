# hand_safety_pkg - Inspire 灵巧手安全层

这是 Inspire 灵巧手的 ROS 2 安全工作区。它位于遥操作/上位机和灵巧手驱动之间，负责过滤危险指令、执行手柄急停、解除急停锁存，并通过 Unitree G1 的语音 API 播报急停状态。

## 整体架构

```plain
遥操作 / 上位机 / 其他普通指令来源
      |
      v
/safe/inspire_hand/raw/cmd/{l,r}        InspireHandCtrl
      |
      v
hand_safety_node
      | 普通状态：发布过滤后的安全指令
      | 急停状态：屏蔽普通指令，发布全 1000 张开指令
      v
/inspire_hand/ctrl/{l,r}                InspireHandCtrl

Unitree G1 /lowstate
      |
      v
hand_gamepad_estop
      | 短按 L1+R1：true
      | 按下 F1：false
      v
/safe/inspire_hand/estop                std_msgs/Bool
      |
      +--> hand_safety_node             急停锁存 / 解除锁存
      |
      +--> hand_safety_voice_node       通过 /api/voice/request 语音播报

/safe/inspire_hand/trigger              std_msgs/String
      |
      v
hand_safety_record_node                 触发时记录滑动窗口日志
```

重要：正常使用时，`/inspire_hand/ctrl/{l,r}` 应该只由 `hand_safety_node` 作为最终出口发布。其他节点如果直接发布 `/inspire_hand/ctrl/{l,r}`，就会绕过安全层，也可能覆盖急停指令。

## 控制消息

`InspireHandCtrl` 包含：

```plain
pos_set
angle_set
force_set
speed_set
mode
```

灵巧手急停时发送的“全部伸直/张开”指令是：

```plain
pos_set   = [0, 0, 0, 0, 0, 0]
angle_set = [1000, 1000, 1000, 1000, 1000, 1000]
force_set = [3000, 3000, 3000, 3000, 3000, 3000]
speed_set = [1000, 1000, 1000, 1000, 1000, 1000]
mode      = 0b0001
```

`0b0001` 表示角度控制模式。

## 安全规则

`hand_safety_node` 对普通 raw 指令按以下顺序过滤：

| 规则 | 触发条件 | 动作 |
|------|----------|------|
| Rule 1：低位保护 | 连续 3 帧中关节 3/4/5 都在 `[0, 200]` | 将关节 3/4/5 钳位到 `200` |
| Rule 2：过载保护 | 任意关节 `force_act > 3000` | 该关节用上一帧输出覆盖 |
| Rule 3：边界限幅 | 任意 `angle_set` 超出 `[0, 1000]` | 小于 0 改为 0，大于 1000 改为 1000 |
| Rule 4：突变平滑 | 5 帧窗口内任意相邻帧角度差 `> 200` | 该关节取 5 帧平均 |

规则触发时，节点会向 `/safe/inspire_hand/trigger` 发布 `String` 事件，供日志节点记录。

## 手柄急停

手柄急停由两个节点配合完成：

- `hand_gamepad_estop`：订阅 Unitree G1 的 `/lowstate`，解析 `wireless_remote[40]`，发布 `/safe/inspire_hand/estop`。
- `hand_safety_node`：订阅 `/safe/inspire_hand/estop`，收到 `true` 后进入急停锁存，屏蔽普通 raw 指令，并以 50 Hz 发布 5 帧全 1000 张开指令。

手柄操作：

| 操作 | 效果 |
|------|------|
| 短按 `L1+R1` | 发布 `/safe/inspire_hand/estop=true`，触发灵巧手急停 |
| 长按 `L1+R1` 超过 2 秒 | 预留给机器人阻尼/整机急停节点，灵巧手短按急停不会触发 |
| 按下 `F1` | 发布 `/safe/inspire_hand/estop=false`，解除灵巧手急停锁存 |

手动触发/解除：

```bash
ros2 topic pub --once /safe/inspire_hand/estop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /safe/inspire_hand/estop std_msgs/msg/Bool "{data: false}"
```

## 语音提示

`hand_safety_voice_node` 订阅 `/safe/inspire_hand/estop`。

| 急停状态 | 播报内容 |
|----------|----------|
| `true` | `灵巧手急停已触发` |
| `false` | `灵巧手急停已解除` |

它通过 Unitree G1 语音 API 发送请求：

```plain
/api/voice/request
```

并监听结果：

```plain
/api/voice/response
```

TTS 的 API ID 是 `1001`，请求参数是 JSON，包含 `index`、`text`、`speaker_id`。

## 节点说明

### hand_safety_node

订阅：

- `/safe/inspire_hand/raw/cmd/{l,r}`（`InspireHandCtrl`）
- `/inspire_hand/state/{l,r}`（`InspireHandState`）
- `/safe/inspire_hand/estop`（`std_msgs/Bool`）

发布：

- `/inspire_hand/ctrl/{l,r}`（`InspireHandCtrl`）
- `/safe/inspire_hand/trigger`（`std_msgs/String`）

### hand_gamepad_estop

订阅：

- `/lowstate`（`unitree_hg/msg/LowState`）

发布：

- `/safe/inspire_hand/estop`（`std_msgs/Bool`）

参数：

- `lowstate_topic`，默认 `/lowstate`
- `estop_topic`，默认 `/safe/inspire_hand/estop`
- `short_press_min_seconds`，默认 `0.05`
- `long_press_seconds`，默认 `2.0`

### hand_safety_voice_node

订阅：

- `/safe/inspire_hand/estop`（`std_msgs/Bool`）
- `/api/voice/response`（`unitree_api/msg/Response`）

发布：

- `/api/voice/request`（`unitree_api/msg/Request`）

参数：

- `estop_topic`，默认 `/safe/inspire_hand/estop`
- `voice_request_topic`，默认 `/api/voice/request`
- `voice_response_topic`，默认 `/api/voice/response`
- `speaker_id`，默认 `0`

### hand_safety_record_node

订阅：

- `/safe/inspire_hand/raw/cmd/{l,r}`
- `/inspire_hand/ctrl/{l,r}`
- `/safe/inspire_hand/trigger`

参数：

- `log_dir`，默认 `~/safety_logs`

## 编译

由于手柄节点依赖 `unitree_hg`，语音节点依赖 `unitree_api`，编译前要先 source Unitree ROS 2 环境。

```bash
source <your_repo_root>/unitree_ros2/setup.sh
cd <your_repo_root>/ztx_dexhand_console/hand_safety_ws
colcon build --packages-select hand_safety_pkg
source install/setup.bash
```

## 运行

```bash
# 终端 1：安全过滤与急停执行节点
source <your_repo_root>/unitree_ros2/setup.sh
source <your_repo_root>/ztx_dexhand_console/hand_safety_ws/install/setup.bash
ros2 run hand_safety_pkg hand_safety_node

# 终端 2：Unitree 手柄急停桥接节点
source <your_repo_root>/unitree_ros2/setup.sh
source <your_repo_root>/ztx_dexhand_console/hand_safety_ws/install/setup.bash
ros2 run hand_safety_pkg hand_gamepad_estop

# 终端 3：G1 语音提示节点，真机上建议开启
source <your_repo_root>/unitree_ros2/setup.sh
source <your_repo_root>/ztx_dexhand_console/hand_safety_ws/install/setup.bash
ros2 run hand_safety_pkg hand_safety_voice_node

# 终端 4：触发日志节点，可选
source <your_repo_root>/ztx_dexhand_console/hand_safety_ws/install/setup.bash
ros2 run hand_safety_pkg hand_safety_record_node
```

## 常用检查命令

```bash
ros2 topic hz /lowstate
ros2 topic echo /safe/inspire_hand/estop
ros2 topic echo /api/voice/response
ros2 pkg executables hand_safety_pkg
```

## 测试

```bash
cd <your_repo_root>/ztx_dexhand_console/hand_safety_ws
source install/setup.bash
python3 src/hand_safety_pkg/test/bug_test.py
```

现有测试覆盖单规则边界、规则交叉、空输入/短数组、左右手隔离、高频输入、状态中断等情况。

## 日志输出

触发日志默认保存在：

```plain
~/safety_logs/safety_trigger_YYYYMMDD_HHMMSS_NNNN.log
```

每份日志包含：

1. 触发信息
2. 左右手最近 raw/output 数据帧
3. 输入和输出的差异分析
