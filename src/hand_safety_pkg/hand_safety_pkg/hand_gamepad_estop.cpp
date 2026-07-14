#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <unitree_hg/msg/low_state.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

using Bool = std_msgs::msg::Bool;
using LowState = unitree_hg::msg::LowState;

namespace unitree_gamepad
{

typedef union {
    struct {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value;
} KeySwitch;

typedef struct {
    uint8_t head[2];
    KeySwitch btn;
    float lx;
    float rx;
    float ry;
    float L2;
    float ly;
    uint8_t idle[16];
} RockerBtnData;

typedef union {
    RockerBtnData data;
    uint8_t buff[40];
} RemoteData;

class Button
{
public:
    void update(bool state)
    {
        on_press = state ? state != pressed : false;
        on_release = state ? false : state != pressed;
        pressed = state;
    }

    bool pressed = false;
    bool on_press = false;
    bool on_release = false;
};

class Gamepad
{
public:
    void update(const RockerBtnData & key_data)
    {
        lx = lx * (1 - smooth) +
             (std::fabs(key_data.lx) < dead_zone ? 0.0F : key_data.lx) * smooth;
        rx = rx * (1 - smooth) +
             (std::fabs(key_data.rx) < dead_zone ? 0.0F : key_data.rx) * smooth;
        ry = ry * (1 - smooth) +
             (std::fabs(key_data.ry) < dead_zone ? 0.0F : key_data.ry) * smooth;
        l2 = l2 * (1 - smooth) +
             (std::fabs(key_data.L2) < dead_zone ? 0.0F : key_data.L2) * smooth;
        ly = ly * (1 - smooth) +
             (std::fabs(key_data.ly) < dead_zone ? 0.0F : key_data.ly) * smooth;

        R1.update(key_data.btn.components.R1);
        L1.update(key_data.btn.components.L1);
        start.update(key_data.btn.components.start);
        select.update(key_data.btn.components.select);
        R2.update(key_data.btn.components.R2);
        L2.update(key_data.btn.components.L2);
        F1.update(key_data.btn.components.F1);
        F2.update(key_data.btn.components.F2);
        A.update(key_data.btn.components.A);
        B.update(key_data.btn.components.B);
        X.update(key_data.btn.components.X);
        Y.update(key_data.btn.components.Y);
        up.update(key_data.btn.components.up);
        right.update(key_data.btn.components.right);
        down.update(key_data.btn.components.down);
        left.update(key_data.btn.components.left);
    }

    float lx = 0.0F;
    float rx = 0.0F;
    float ry = 0.0F;
    float l2 = 0.0F;
    float ly = 0.0F;

    float smooth = 0.03F;
    float dead_zone = 0.01F;

    Button R1;
    Button L1;
    Button start;
    Button select;
    Button R2;
    Button L2;
    Button F1;
    Button F2;
    Button A;
    Button B;
    Button X;
    Button Y;
    Button up;
    Button right;
    Button down;
    Button left;
};

}  // namespace unitree_gamepad

class HandGamepadEstop : public rclcpp::Node
{
public:
    HandGamepadEstop()
    : Node("hand_gamepad_estop")
    {
        lowstate_topic_ = this->declare_parameter<std::string>(
            "lowstate_topic", "/lowstate");
        estop_topic_ = this->declare_parameter<std::string>(
            "estop_topic", "/safe/inspire_hand/estop");
        squat_lock_topic_ = this->declare_parameter<std::string>(
            "squat_lock_topic", "/safe/inspire_hand/squat_lock");
        short_press_min_seconds_ = this->declare_parameter<double>(
            "short_press_min_seconds", 0.05);
        long_press_seconds_ = this->declare_parameter<double>(
            "long_press_seconds", 2.0);

        lowstate_sub_ = this->create_subscription<LowState>(
            lowstate_topic_, 10,
            [this](const LowState::SharedPtr msg) {
                this->lowstate_callback(msg);
            });

        estop_pub_ = this->create_publisher<Bool>(estop_topic_, 10);
        squat_lock_pub_ = this->create_publisher<Bool>(
            squat_lock_topic_, 10);

        RCLCPP_INFO(this->get_logger(),
            "Hand gamepad estop initialized: short press L1+R1 -> %s, "
            "long press threshold %.2f s is reserved for robot estop.",
            estop_topic_.c_str(), long_press_seconds_);
    }

private:
    void lowstate_callback(const LowState::SharedPtr msg)
    {
        unitree_gamepad::RemoteData remote{};
        std::memcpy(remote.buff, msg->wireless_remote.data(), remote_size_);
        gamepad_.update(remote.data);

        if (gamepad_.F1.on_press) {
            clear_hand_estop();
        }

        const bool combo_pressed = gamepad_.L1.pressed && gamepad_.R1.pressed;
        const auto now = this->now();

        if (combo_pressed && !combo_active_) {
            combo_active_ = true;
            combo_long_press_seen_ = false;
            combo_start_time_ = now;
            return;
        }

        if (combo_pressed && combo_active_) {
            const double held_seconds = (now - combo_start_time_).seconds();
            if (!combo_long_press_seen_ && held_seconds >= long_press_seconds_) {
                combo_long_press_seen_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "L1+R1 held for %.2f s: treat as long press, hand short-press "
                    "estop will not fire on release.",
                    held_seconds);
            }
            return;
        }

        if (!combo_pressed && combo_active_) {
            const double held_seconds = (now - combo_start_time_).seconds();
            combo_active_ = false;

            if (!combo_long_press_seen_ &&
                held_seconds >= short_press_min_seconds_ &&
                held_seconds < long_press_seconds_)
            {
                RCLCPP_WARN(this->get_logger(),
                    "Short press L1+R1 detected (%.2f s): opening both hands.",
                    held_seconds);
                trigger_hand_estop();
            } else {
                RCLCPP_INFO(this->get_logger(),
                    "L1+R1 release ignored by hand estop (held %.2f s).",
                    held_seconds);
            }
        }
    }

    void trigger_hand_estop()
    {
        auto msg = Bool();
        msg.data = true;
        estop_pub_->publish(msg);
    }

    void clear_hand_estop()
    {
        auto msg = Bool();
        msg.data = false;
        estop_pub_->publish(msg);
        publish_squat_lock(false);
        RCLCPP_WARN(this->get_logger(),
            "F1 pressed: hand estop and squat lock clear requested.");
    }

    void publish_squat_lock(bool locked)
    {
        auto msg = Bool();
        msg.data = locked;
        squat_lock_pub_->publish(msg);
    }

    static constexpr size_t remote_size_ = 40;

    std::string lowstate_topic_;
    std::string estop_topic_;
    std::string squat_lock_topic_;
    double short_press_min_seconds_{0.05};
    double long_press_seconds_{2.0};

    rclcpp::Subscription<LowState>::SharedPtr lowstate_sub_;
    rclcpp::Publisher<Bool>::SharedPtr estop_pub_;
    rclcpp::Publisher<Bool>::SharedPtr squat_lock_pub_;

    unitree_gamepad::Gamepad gamepad_;
    bool combo_active_{false};
    bool combo_long_press_seen_{false};
    rclcpp::Time combo_start_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HandGamepadEstop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
