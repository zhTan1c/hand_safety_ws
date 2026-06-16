/**
 * hand_safety_node.cpp — C++ 版安全节点
 *
 * 逻辑与 Python 版 hand_safety_node.py 完全一致：
 *   Rule 1: 连续3帧 joint[3]/[4]/[5] ∈[0,200] → 钳位到 200
 *   Rule 2: force_act[idx] > 3000 → 用上一帧 angle 覆盖
 *   Rule 3: angle_set[idx] < 0 → 0, > 1000 → 1000
 *   Rule 4: 5帧窗口相邻帧差 > 200 → 取5帧均值
 *
 * 触发时发布 /safe/inspire_hand/trigger (std_msgs/String)
 * 收到 /safe/inspire_hand/estop=true 时进入急停锁存，停止普通指令输出。
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <inspire_hand_msgs/msg/inspire_hand_ctrl.hpp>
#include <inspire_hand_msgs/msg/inspire_hand_state.hpp>

#include <deque>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>

using InspireHandCtrl = inspire_hand_msgs::msg::InspireHandCtrl;
using InspireHandState = inspire_hand_msgs::msg::InspireHandState;
using Bool = std_msgs::msg::Bool;
using String = std_msgs::msg::String;

// BoundedVector<int16_t, 6> 的便捷别名
using AngleVec = decltype(InspireHandCtrl::angle_set);

// 辅助：把 BoundedVector 转成 std::vector<int16_t>
static std::vector<int16_t> to_vec(const AngleVec & v) {
    return std::vector<int16_t>(v.begin(), v.end());
}

// 辅助：把 std::vector<int16_t> 转回 BoundedVector（用 initializer_list 赋值）
static AngleVec from_vec(const std::vector<int16_t> & v) {
    AngleVec result;
    for (size_t i = 0; i < v.size() && i < 6; ++i) {
        result.push_back(v[i]);
    }
    return result;
}

class HandSafetyNode : public rclcpp::Node
{
public:
    HandSafetyNode()
    : Node("hand_safety_node")
    {
        trigger_pub_ = this->create_publisher<String>(
            "/safe/inspire_hand/trigger", 10);
        estop_sub_ = this->create_subscription<Bool>(
            "/safe/inspire_hand/estop", 10,
            [this](const Bool & msg) {
                this->estop_callback(msg);
            });
        estop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            [this]() {
                this->estop_timer_callback();
            });

        for (const auto & side : {"l", "r"}) {
            std::string s(side);
            auto sub_topic = "/safe/inspire_hand/raw/cmd/" + s;
            auto pub_topic = "/inspire_hand/ctrl/" + s;
            auto state_topic = "/inspire_hand/state/" + s;

            // 用 lambda 捕获 side，避免 std::bind 的签名问题
            cmd_subs_[s] = this->create_subscription<InspireHandCtrl>(
                sub_topic, 10,
                [this, s](const InspireHandCtrl & msg) {
                    this->cmd_callback(msg, s);
                });

            cmd_pubs_[s] = this->create_publisher<InspireHandCtrl>(
                pub_topic, 10);

            state_subs_[s] = this->create_subscription<InspireHandState>(
                state_topic, 10,
                [this, s](const InspireHandState & msg) {
                    this->state_callback(msg, s);
                });

            RCLCPP_INFO(this->get_logger(),
                "Safety node initialized for side %s: %s -> %s, state: %s",
                s.c_str(), sub_topic.c_str(), pub_topic.c_str(),
                state_topic.c_str());
        }
    }

private:
    // ── 回调 ──

    void state_callback(const InspireHandState & msg, const std::string & side)
    {
        latest_state_[side] = std::make_shared<InspireHandState>(msg);
    }

    void cmd_callback(const InspireHandCtrl & msg, const std::string & side)
    {
        if (estop_active_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[%s] Estop active, raw command ignored.", side.c_str());
            return;
        }

        auto out = process_msg(msg, side);
        cmd_pubs_[side]->publish(out);
        history_[side].push_back(std::make_shared<InspireHandCtrl>(out));
    }

    void estop_callback(const Bool & msg)
    {
        if (msg.data) {
            estop_active_ = true;
            estop_pending_frames_ = 5;
            RCLCPP_ERROR(this->get_logger(),
                "Hand estop latched: publishing 5 frames at 50 Hz and blocking normal commands.");
            publish_estop_command();
        } else {
            estop_active_ = false;
            estop_pending_frames_ = 0;
            RCLCPP_WARN(this->get_logger(),
                "Hand estop cleared: normal safety-filtered commands are enabled again.");
        }
    }

    void estop_timer_callback()
    {
        if (!estop_active_ || estop_pending_frames_ <= 0) {
            return;
        }
        publish_estop_command();
    }

    void publish_estop_command()
    {
        InspireHandCtrl cmd;
        cmd.pos_set = {0, 0, 0, 0, 0, 0};
        cmd.angle_set = {1000, 1000, 1000, 1000, 1000, 1000};
        cmd.force_set = {3000, 3000, 3000, 3000, 3000, 3000};
        cmd.speed_set = {1000, 1000, 1000, 1000, 1000, 1000};
        cmd.mode = 0b0001;

        for (const auto & side : {"l", "r"}) {
            std::string s(side);
            auto it = cmd_pubs_.find(s);
            if (it != cmd_pubs_.end() && it->second) {
                it->second->publish(cmd);
            }
        }

        --estop_pending_frames_;
    }

    // ── 核心安全逻辑 ──

    InspireHandCtrl process_msg(
        const InspireHandCtrl & msg,
        const std::string & side)
    {
        auto & hist = history_[side];
        std::vector<std::string> triggered_rules;

        // 当前帧 angle_set → std::vector 方便修改
        std::vector<int16_t> angle_set(msg.angle_set.begin(),
                                       msg.angle_set.end());

        // 上一帧或 fallback
        InspireHandCtrl prev_frame;
        if (!hist.empty()) {
            prev_frame = *hist.back();
        } else {
            prev_frame.angle_set = {1000, 1000, 1000, 1000, 1000, 1000};
            prev_frame.pos_set   = {0, 0, 0, 0, 0, 0};
            prev_frame.force_set = {3000, 3000, 3000, 3000, 3000, 3000};
            prev_frame.speed_set = {1000, 1000, 1000, 1000, 1000, 1000};
            prev_frame.mode = 13;
        }

        // frames_angles: deque 历史 + 当前帧
        std::vector<std::vector<int16_t>> frames_angles;
        for (auto & f : hist) {
            frames_angles.emplace_back(f->angle_set.begin(),
                                       f->angle_set.end());
        }
        frames_angles.push_back(angle_set);

        // ── Rule 1 ──
        if (frames_angles.size() >= 3) {
            auto & f0 = frames_angles[frames_angles.size() - 3];
            auto & f1 = frames_angles[frames_angles.size() - 2];
            auto & f2 = frames_angles[frames_angles.size() - 1];
            if (f0.size() > 5 && f1.size() > 5 && f2.size() > 5) {
                if (f0[3] >= 0 && f0[3] <= 200 && f0[4] >= 0 && f0[4] <= 200 &&
                    f0[5] >= 0 && f0[5] <= 200 &&
                    f1[3] >= 0 && f1[3] <= 200 && f1[4] >= 0 && f1[4] <= 200 &&
                    f1[5] >= 0 && f1[5] <= 200 &&
                    f2[3] >= 0 && f2[3] <= 200 && f2[4] >= 0 && f2[4] <= 200 &&
                    f2[5] >= 0 && f2[5] <= 200)
                {
                    angle_set[3] = 200;
                    angle_set[4] = 200;
                    angle_set[5] = 200;
                    frames_angles.back()[3] = 200;
                    frames_angles.back()[4] = 200;
                    frames_angles.back()[5] = 200;
                    triggered_rules.push_back(
                        "Rule 1 (joint 4/5/6 <= 200 -> clamp to 200)");
                }
            }
        }

        // ── Rule 2 ──
        std::vector<int16_t> forces;
        auto it = latest_state_.find(side);
        if (it != latest_state_.end() && it->second &&
            !it->second->force_act.empty())
        {
            forces.assign(it->second->force_act.begin(),
                          it->second->force_act.end());
        } else {
            forces.assign(msg.force_set.begin(), msg.force_set.end());
        }

        for (size_t idx = 0; idx < forces.size(); ++idx) {
            if (forces[idx] > 3000) {
                if (prev_frame.angle_set.size() > idx) {
                    angle_set[idx] = prev_frame.angle_set[idx];
                }
                if (frames_angles.back().size() > idx) {
                    frames_angles.back()[idx] = angle_set[idx];
                }
                std::ostringstream oss;
                oss << "Rule 2 (joint " << idx
                    << " force_act > 3000 overwrite)";
                triggered_rules.push_back(oss.str());
            }
        }

        // ── Rule 3 ──
        for (size_t idx = 0; idx < angle_set.size(); ++idx) {
            if (angle_set[idx] < 0 || angle_set[idx] > 1000) {
                if (angle_set[idx] < 0) {
                    angle_set[idx] = 0;
                } else {
                    angle_set[idx] = 1000;
                }
                if (frames_angles.back().size() > idx) {
                    frames_angles.back()[idx] = angle_set[idx];
                }
                std::ostringstream oss;
                oss << "Rule 3 (joint " << idx
                    << " angle out of bounds clamp)";
                triggered_rules.push_back(oss.str());
            }
        }

        // ── Rule 4 ──
        if (frames_angles.size() == 5) {
            size_t num_dofs = frames_angles[0].size();
            for (auto & fa : frames_angles) {
                num_dofs = std::min(num_dofs, fa.size());
            }
            for (size_t idx = 0; idx < num_dofs; ++idx) {
                bool has_large_change = false;
                for (int k = 0; k < 4; ++k) {
                    if (std::abs(static_cast<int>(frames_angles[k][idx]) -
                                 static_cast<int>(frames_angles[k + 1][idx]))
                        > 200)
                    {
                        has_large_change = true;
                        break;
                    }
                }
                if (has_large_change) {
                    int sum = 0;
                    for (auto & fa : frames_angles) {
                        sum += fa[idx];
                    }
                    angle_set[idx] = static_cast<int16_t>(
                        std::round(sum / 5.0));
                    std::ostringstream oss;
                    oss << "Rule 4 (joint " << idx << " diff > 200 avg)";
                    triggered_rules.push_back(oss.str());
                }
            }
        }

        // ── 日志 + 触发发布 ──
        if (!triggered_rules.empty()) {
            std::ostringstream oss;
            for (size_t i = 0; i < triggered_rules.size(); ++i) {
                if (i > 0) oss << ", ";
                oss << triggered_rules[i];
            }
            RCLCPP_WARN(this->get_logger(),
                "[%s] Safety rule(s) triggered: %s",
                side.c_str(), oss.str().c_str());

            for (auto & rule : triggered_rules) {
                auto trigger_msg = String();
                trigger_msg.data = rule + ":" + side;
                trigger_pub_->publish(trigger_msg);
            }
        }

        // ── 构建输出 ──
        InspireHandCtrl out_msg;
        out_msg.angle_set = from_vec(angle_set);
        out_msg.pos_set   = {0, 0, 0, 0, 0, 0};
        out_msg.force_set = {3000, 3000, 3000, 3000, 3000, 3000};
        out_msg.speed_set = {1000, 1000, 1000, 1000, 1000, 1000};
        out_msg.mode = 0b1101;
        return out_msg;
    }

    // ── 成员变量 ──

    rclcpp::Publisher<String>::SharedPtr trigger_pub_;
    rclcpp::Subscription<Bool>::SharedPtr estop_sub_;
    rclcpp::TimerBase::SharedPtr estop_timer_;
    std::map<std::string, rclcpp::Publisher<InspireHandCtrl>::SharedPtr> cmd_pubs_;
    std::map<std::string, rclcpp::Subscription<InspireHandCtrl>::SharedPtr> cmd_subs_;
    std::map<std::string, rclcpp::Subscription<InspireHandState>::SharedPtr> state_subs_;

    // 历史队列（每侧最多 5 帧）
    std::map<std::string, std::deque<std::shared_ptr<InspireHandCtrl>>> history_;

    // 最新 state
    std::map<std::string, std::shared_ptr<InspireHandState>> latest_state_;

    bool estop_active_{false};
    int estop_pending_frames_{0};
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HandSafetyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
