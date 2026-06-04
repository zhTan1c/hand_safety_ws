# hand_safety_pkg — Inspire 灵巧手安全层

一个 ROS 2 功能包，在指令到达灵巧手之前对其进行安全过滤。它位于遥操作控制器和灵巧手驱动之间，实时拦截不安全的角度/力矩/速度指令。

## 架构

```plain
遥操作 / 上位机
      │
      ▼
/safe/inspire_hand/raw/cmd/l(r)    ← 原始指令（输入）
      │
      ▼
┌──────────────────┐
│ hand_safety_node │   ← C++ 节点，4 条安全规则过滤
│ （或 .py 版本）   │
└──┬───────────┬───┘
   │            --------------│
   ▼                          ▼
/inspire_hand/        /safe/inspire_hand/
 ctrl/l(r)             trigger
 （过滤后输出）         （String，规则触发事件）
                           │
                           ▼
                    ┌─────────────────────┐
                    │ hand_record_node     │  ← Python 节点，触发时记录日志
                    │ （滑动窗口日志记录）   │
                    └─────────────────────┘
```

## 安全规则

| 规则 | 触发条件 | 安全动作 |
|------|---------|---------|
| **Rule 1**（低位保护） | 连续 3 帧中关节 3/4/5 均 ∈[0, 200] | 将关节 3/4/5 钳位到 200 |
| **Rule 2**（过载保护） | 任意关节 `force_act > 3000` | 用上一帧的值覆盖该关节 |
| **Rule 3**（边界限幅） | 任意关节角度超出 [0, 1000] | 钳位到 0 或 1000 |
| **Rule 4**（突变平滑） | 5 帧窗口内相邻帧差 > 200 | 取 5 帧平均值 |

执行顺序：Rule 1 → Rule 2 → Rule 3 → Rule 4。

## 节点说明

### hand_safety_node（C++）

核心安全过滤节点。订阅原始指令和灵巧手状态，应用 4 条规则，发布过滤后的指令。

**订阅：**
- `/safe/inspire_hand/raw/cmd/{l,r}`（`InspireHandCtrl`）
- `/inspire_hand/state/{l,r}`（`InspireHandState`）

**发布：**
- `/inspire_hand/ctrl/{l,r}`（`InspireHandCtrl`）— 过滤后输出
- `/safe/inspire_hand/trigger`（`String`）— 规则触发事件

### hand_safety_record_node（Python）

录制节点。在 4 个数据话题上维护 maxlen=10 的滑动窗口，当收到规则触发事件时，将最近 5 帧 × 2 话题 × 2 手 = 20 帧数据写入带时间戳的 `.log` 文件。

**订阅：**
- `/safe/inspire_hand/raw/cmd/{l,r}`
- `/inspire_hand/ctrl/{l,r}`
- `/safe/inspire_hand/trigger`

**参数：**
- `log_dir`（string，默认 `~/safety_logs`）— 日志输出目录

## 编译

```bash
cd hand_safety_ws
colcon build  # ros2会自动识别msg包并优先编译，这样不会出现节点编译缺少msg包的报错
source install/setup.bash
```

## 运行

```bash
# 终端 1：安全节点
ros2 run hand_safety_pkg hand_safety_node

# 终端 2：录制节点（可选）
ros2 run hand_safety_pkg hand_safety_record_node
# 或指定日志目录：
ros2 run hand_safety_pkg hand_safety_record_node --ros-args -p log_dir:=/tmp/my_logs
```

## 测试

```bash
# 确保 hand_safety_node 已启动
python3 src/hand_safety_pkg/test/continuous_publish.py  # 这个测试脚本以50hz的频率周期发送700和800交替指令，如果灵巧手跟着动说明节点通路成功建立
```

## 日志输出

触发日志保存在 `~/safety_logs/safety_trigger_YYYYMMDD_HHMMSS_NNNN.log`，每份日志包含：
1. 触发信息（规则、左右手、时间戳）
2. 20 帧完整数据（angle/force/pos/speed/mode）
3. 输入 vs 输出 DIFF 分析，高亮被规则修改的关节
