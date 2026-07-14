#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_api/msg/response.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>

using Bool = std_msgs::msg::Bool;
using Request = unitree_api::msg::Request;
using Response = unitree_api::msg::Response;

namespace
{
constexpr int64_t kGetFsmIdApiId = 7001;

int64_t monotonic_time_ns()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

std::string trim(const std::string & text)
{
    const auto first = text.find_first_not_of(" \t\r\n\"");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = text.find_last_not_of(" \t\r\n\"");
    return text.substr(first, last - first + 1);
}

bool parse_data_int(const std::string & data, int & value)
{
    const auto direct = trim(data);
    if (!direct.empty() &&
        (std::isdigit(direct.front()) || direct.front() == '-'))
    {
        char * end = nullptr;
        const long parsed = std::strtol(direct.c_str(), &end, 10);
        if (end != direct.c_str()) {
            value = static_cast<int>(parsed);
            return true;
        }
    }

    auto key_pos = data.find("\"data\"");
    if (key_pos == std::string::npos) {
        key_pos = data.find("data");
    }
    if (key_pos == std::string::npos) {
        return false;
    }
    const auto colon_pos = data.find(':', key_pos);
    if (colon_pos == std::string::npos) {
        return false;
    }

    auto value_start = data.find_first_not_of(" \t\r\n\"", colon_pos + 1);
    if (value_start == std::string::npos) {
        return false;
    }
    auto value_end = data.find_first_of(",}\" \t\r\n", value_start);
    if (value_end == std::string::npos) {
        value_end = data.size();
    }
    const auto token = data.substr(value_start, value_end - value_start);
    if (token.empty()) {
        return false;
    }

    char * end = nullptr;
    const long parsed = std::strtol(token.c_str(), &end, 10);
    if (end == token.c_str()) {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

bool contains_id(const std::vector<int64_t> & ids, int fsm_id)
{
    return std::find(ids.begin(), ids.end(), static_cast<int64_t>(fsm_id)) !=
           ids.end();
}
}  // namespace

class HandRobotStateMonitor : public rclcpp::Node
{
public:
    HandRobotStateMonitor()
    : Node("hand_robot_state_monitor")
    {
        request_topic_ = this->declare_parameter<std::string>(
            "sport_request_topic", "/api/sport/request");
        response_topic_ = this->declare_parameter<std::string>(
            "sport_response_topic", "/api/sport/response");
        lock_topic_ = this->declare_parameter<std::string>(
            "squat_lock_topic", "/safe/inspire_hand/squat_lock");
        poll_hz_ = this->declare_parameter<double>("poll_hz", 5.0);
        response_timeout_sec_ = this->declare_parameter<double>(
            "response_timeout_sec", 1.0);
        lock_fsm_ids_ = this->declare_parameter<std::vector<int64_t>>(
            "lock_fsm_ids", std::vector<int64_t>{706});
        unlock_fsm_ids_ = this->declare_parameter<std::vector<int64_t>>(
            "unlock_fsm_ids", std::vector<int64_t>{501, 801});
        auto_clear_when_safe_ = this->declare_parameter<bool>(
            "auto_clear_when_safe", false);

        request_pub_ = this->create_publisher<Request>(request_topic_, 1);
        lock_pub_ = this->create_publisher<Bool>(lock_topic_, 10);
        response_sub_ = this->create_subscription<Response>(
            response_topic_, 10,
            [this](const Response::SharedPtr msg) {
                this->response_callback(msg);
            });

        const auto period_ms = static_cast<int>(
            1000.0 / std::max(0.1, poll_hz_));
        poll_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            [this]() {
                this->poll_timer_callback();
            });

        RCLCPP_INFO(this->get_logger(),
            "Robot state monitor initialized: %s -> %s, lock topic %s",
            request_topic_.c_str(), response_topic_.c_str(), lock_topic_.c_str());
    }

private:
    void poll_timer_callback()
    {
        const auto now = this->now();
        if (request_pending_) {
            const double age = (now - request_time_).seconds();
            if (age <= response_timeout_sec_) {
                return;
            }
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "GetFsmId request timed out after %.2f s.", age);
            request_pending_ = false;
        }

        Request req;
        req.header.identity.id = monotonic_time_ns();
        req.header.identity.api_id = kGetFsmIdApiId;
        latest_request_id_ = req.header.identity.id;
        request_time_ = now;
        request_pending_ = true;
        request_pub_->publish(req);
    }

    void response_callback(const Response::SharedPtr msg)
    {
        if (!request_pending_ ||
            msg->header.identity.id != latest_request_id_)
        {
            return;
        }
        request_pending_ = false;

        if (msg->header.status.code != 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "GetFsmId response status code: %d",
                msg->header.status.code);
            return;
        }

        int fsm_id = 0;
        if (!parse_data_int(msg->data, fsm_id)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Failed to parse GetFsmId response data: %s",
                msg->data.c_str());
            return;
        }

        if (!has_last_fsm_id_ || fsm_id != last_fsm_id_) {
            RCLCPP_INFO(this->get_logger(), "G1 fsm_id: %d", fsm_id);
            has_last_fsm_id_ = true;
            last_fsm_id_ = fsm_id;
        }

        if (contains_id(lock_fsm_ids_, fsm_id)) {
            if (!lock_published_) {
                publish_lock(true);
                RCLCPP_WARN(this->get_logger(),
                    "fsm_id %d is squat/stand transition state: lock requested.",
                    fsm_id);
            }
            return;
        }

        if (auto_clear_when_safe_ && lock_published_ &&
            contains_id(unlock_fsm_ids_, fsm_id))
        {
            publish_lock(false);
            RCLCPP_WARN(this->get_logger(),
                "fsm_id %d is normal motion state: lock clear requested.",
                fsm_id);
        }
    }

    void publish_lock(bool locked)
    {
        Bool msg;
        msg.data = locked;
        lock_pub_->publish(msg);
        lock_published_ = locked;
    }

    std::string request_topic_;
    std::string response_topic_;
    std::string lock_topic_;
    double poll_hz_{5.0};
    double response_timeout_sec_{1.0};
    std::vector<int64_t> lock_fsm_ids_;
    std::vector<int64_t> unlock_fsm_ids_;
    bool auto_clear_when_safe_{false};

    rclcpp::Publisher<Request>::SharedPtr request_pub_;
    rclcpp::Publisher<Bool>::SharedPtr lock_pub_;
    rclcpp::Subscription<Response>::SharedPtr response_sub_;
    rclcpp::TimerBase::SharedPtr poll_timer_;

    bool request_pending_{false};
    rclcpp::Time request_time_;
    int64_t latest_request_id_{0};
    bool has_last_fsm_id_{false};
    int last_fsm_id_{0};
    bool lock_published_{false};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HandRobotStateMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
