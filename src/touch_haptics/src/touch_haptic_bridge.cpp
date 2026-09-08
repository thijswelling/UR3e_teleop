#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <mutex>
#include <chrono>

static hduVector3Dd g_cmd_force(0.0, 0.0, 0.0);
static hduVector3Dd g_current_pos(0.0, 0.0, 0.0);
static hduVector3Dd g_current_gimbal(0.0, 0.0, 0.0);
static int g_current_buttons = 0;
static bool g_has_data = false;
static std::mutex g_force_mutex;
static std::mutex g_data_mutex;

HDCallbackCode HDCALLBACK hapticLoopCallback(void *pUserData) {
    (void)pUserData;
    hdBeginFrame(hdGetCurrentDevice());

    hduVector3Dd f;
    {
        std::lock_guard<std::mutex> lock(g_force_mutex);
        f = g_cmd_force;
    }
    hdSetDoublev(HD_CURRENT_FORCE, f);

    hduVector3Dd pos, gimbal;
    int btns = 0;
    hdGetDoublev(HD_CURRENT_POSITION, pos);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal);
    hdGetIntegerv(HD_CURRENT_BUTTONS, &btns);

    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        g_current_pos = pos;
        g_current_gimbal = gimbal;
        g_current_buttons = btns;
        g_has_data = true;
    }

    hdEndFrame(hdGetCurrentDevice());
    return HD_CALLBACK_CONTINUE;
}

class TouchHapticBridge : public rclcpp::Node {
public:
    TouchHapticBridge() : Node("touch_haptic_bridge"), hHD_(HD_INVALID_HANDLE), hCallback_(HD_INVALID_HANDLE) {
        hHD_ = hdInitDevice(HD_DEFAULT_DEVICE);
        if (HD_DEVICE_ERROR(hdGetError())) {
            RCLCPP_ERROR(this->get_logger(), "Kon Touch haptic device niet initialiseren!");
            return;
        }

        hdEnable(HD_FORCE_OUTPUT);
        hdStartScheduler();

        hCallback_ = hdScheduleAsynchronous(hapticLoopCallback, nullptr, HD_DEFAULT_SCHEDULER_PRIORITY);

        raw_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/touch/raw_pose", 10);
        btn_pub_ = this->create_publisher<std_msgs::msg::Int32>("/touch/buttons", 10);

        force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/touch/cmd_force", 10,
            std::bind(&TouchHapticBridge::forceCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&TouchHapticBridge::publishTimer, this));

        RCLCPP_INFO(this->get_logger(), ">>> C++ Touch Hardware Bridge ACTIEF (1000 Hz loop) <<<");
    }

    ~TouchHapticBridge() {
        if (hCallback_ != HD_INVALID_HANDLE) hdUnschedule(hCallback_);
        hdStopScheduler();
        if (hHD_ != HD_INVALID_HANDLE) {
            hdDisable(HD_FORCE_OUTPUT);
            hdDisableDevice(hHD_);
        }
    }

private:
    void forceCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(g_force_mutex);
        g_cmd_force[0] = msg->wrench.force.x;
        g_cmd_force[1] = msg->wrench.force.y;
        g_cmd_force[2] = msg->wrench.force.z;
    }

    void publishTimer() {
        hduVector3Dd pos, gimbal;
        int btns = 0;
        bool ready = false;
        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            ready = g_has_data;
            pos = g_current_pos;
            gimbal = g_current_gimbal;
            btns = g_current_buttons;
        }

        // Publiceer pas als de Touch daadwerkelijk reële meetdata levert
        if (!ready) {
            return;
        }

        auto p_msg = geometry_msgs::msg::PoseStamped();
        p_msg.header.stamp = this->now();
        p_msg.header.frame_id = "touch_base";
        p_msg.pose.position.x = pos[0];
        p_msg.pose.position.y = pos[1];
        p_msg.pose.position.z = pos[2];
        p_msg.pose.orientation.x = gimbal[0];
        p_msg.pose.orientation.y = gimbal[1];
        p_msg.pose.orientation.z = gimbal[2];
        raw_pose_pub_->publish(p_msg);

        auto b_msg = std_msgs::msg::Int32();
        b_msg.data = btns;
        btn_pub_->publish(b_msg);
    }

    HHD hHD_;
    HDSchedulerHandle hCallback_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr raw_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr btn_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TouchHapticBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
