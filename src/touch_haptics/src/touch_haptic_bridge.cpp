#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <mutex>
#include <chrono>
#include <cmath>

static hduVector3Dd g_cmd_force(0.0, 0.0, 0.0);
static hduVector3Dd g_current_pos(0.0, 0.0, 0.0);
static double g_current_transform[16]; 
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

    hduVector3Dd pos;
    double transform[16];
    int btns = 0;
    hdGetDoublev(HD_CURRENT_POSITION, pos);
    hdGetDoublev(HD_CURRENT_TRANSFORM, transform); // Absolute ruimtelijke data
    hdGetIntegerv(HD_CURRENT_BUTTONS, &btns);

    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        g_current_pos = pos;
        std::copy(std::begin(transform), std::end(transform), std::begin(g_current_transform));
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
        if (HD_DEVICE_ERROR(hdGetError())) return;

        hdEnable(HD_FORCE_OUTPUT);
        hdStartScheduler();
        hCallback_ = hdScheduleAsynchronous(hapticLoopCallback, nullptr, HD_DEFAULT_SCHEDULER_PRIORITY);

        raw_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/touch/raw_pose", 10);
        btn_pub_ = this->create_publisher<std_msgs::msg::Int32>("/touch/buttons", 10);
        force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/touch/cmd_force", 10, std::bind(&TouchHapticBridge::forceCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&TouchHapticBridge::publishTimer, this));
    }

    ~TouchHapticBridge() {
        if (hCallback_ != HD_INVALID_HANDLE) hdUnschedule(hCallback_);
        hdStopScheduler();
        if (hHD_ != HD_INVALID_HANDLE) { hdDisable(HD_FORCE_OUTPUT); hdDisableDevice(hHD_); }
    }

private:
    void forceCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(g_force_mutex);
        g_cmd_force[0] = msg->wrench.force.x;
        g_cmd_force[1] = msg->wrench.force.y;
        g_cmd_force[2] = msg->wrench.force.z;
    }

    void publishTimer() {
        hduVector3Dd pos;
        double transform[16];
        int btns = 0;
        bool ready = false;
        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            ready = g_has_data;
            pos = g_current_pos;
            std::copy(std::begin(g_current_transform), std::end(g_current_transform), std::begin(transform));
            btns = g_current_buttons;
        }

        if (!ready) return;

        // Converteer 4x4 matrix naar een zuivere Quaternion voor ROS
        double tr = transform[0] + transform[5] + transform[10];
        double qw, qx, qy, qz;
        if (tr > 0) {
            double S = sqrt(tr + 1.0) * 2;
            qw = 0.25 * S; qx = (transform[6] - transform[9]) / S;
            qy = (transform[8] - transform[2]) / S; qz = (transform[1] - transform[4]) / S;
        } else if ((transform[0] > transform[5]) && (transform[0] > transform[10])) {
            double S = sqrt(1.0 + transform[0] - transform[5] - transform[10]) * 2;
            qw = (transform[6] - transform[9]) / S; qx = 0.25 * S;
            qy = (transform[1] + transform[4]) / S; qz = (transform[8] + transform[2]) / S;
        } else if (transform[5] > transform[10]) {
            double S = sqrt(1.0 + transform[5] - transform[0] - transform[10]) * 2;
            qw = (transform[8] - transform[2]) / S; qx = (transform[1] + transform[4]) / S;
            qy = 0.25 * S; qz = (transform[6] + transform[9]) / S;
        } else {
            double S = sqrt(1.0 + transform[10] - transform[0] - transform[5]) * 2;
            qw = (transform[1] - transform[4]) / S; qx = (transform[8] + transform[2]) / S;
            qy = (transform[6] + transform[9]) / S; qz = 0.25 * S;
        }

        auto p_msg = geometry_msgs::msg::PoseStamped();
        p_msg.header.stamp = this->now();
        p_msg.header.frame_id = "touch_base";
        p_msg.pose.position.x = pos[0]; p_msg.pose.position.y = pos[1]; p_msg.pose.position.z = pos[2];
        p_msg.pose.orientation.w = qw; p_msg.pose.orientation.x = qx;
        p_msg.pose.orientation.y = qy; p_msg.pose.orientation.z = qz;
        raw_pose_pub_->publish(p_msg);

        auto b_msg = std_msgs::msg::Int32();
        b_msg.data = btns;
        btn_pub_->publish(b_msg);
    }

    HHD hHD_; HDSchedulerHandle hCallback_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr raw_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr btn_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TouchHapticBridge>();
    rclcpp::spin(node); rclcpp::shutdown(); return 0;
}