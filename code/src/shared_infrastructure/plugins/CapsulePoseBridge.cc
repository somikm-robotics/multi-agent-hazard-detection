#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <ignition/transport/Node.hh> // Fortress (Ignition)
#include <ignition/msgs/pose_v.pb.h>  // ignition.msgs.Pose_V

#include <unordered_map>
#include <string>
#include <algorithm>

class CapsulePoseBridge : public rclcpp::Node
{
public:
    CapsulePoseBridge()
        : rclcpp::Node("capsule_pose_bridge")
    {
        capsule_name_ = declare_parameter<std::string>("capsule_name", "asbestos_sample_capsule");
        world_topic_ = declare_parameter<std::string>("world_topic", "/world/mining_world/pose/info");
        frame_id_ = declare_parameter<std::string>("frame_id", "map");

        // cache the patch model world pose (odom/map)
        bool have_patch_{false};
        double patch_x_{0.0}, patch_y_{0.0}, patch_z_{0.0}, patch_yaw_{0.0};

        // do NOT declare use_sim_time here (set it from launch/CLI)

        // legacy single-topic mirror (center capsule)
        pub_single_ = create_publisher<geometry_msgs::msg::PoseStamped>("/capsule/pose", 10);

        if (!ign_node_.Subscribe(world_topic_, &CapsulePoseBridge::onPoseV, this))
        {
            RCLCPP_FATAL(get_logger(), "Failed to subscribe to %s", world_topic_.c_str());
            throw std::runtime_error("ign subscribe failed");
        }
        RCLCPP_INFO(get_logger(),
                    "Bridging %s → /capsule/<name>/pose  (frame=%s)", world_topic_.c_str(), frame_id_.c_str());
    }

private:
    // per-capsule publisher cache: /capsule/<short_name>/pose
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    getOrCreatePub(const std::string &short_name)
    {
        auto it = pubs_.find(short_name);
        if (it != pubs_.end())
            return it->second;

        const std::string topic = "/capsule/" + short_name + "/pose";
        auto pub = create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
        pubs_.emplace(short_name, pub);
        RCLCPP_INFO(get_logger(), "Publishing %s", topic.c_str());
        return pub;
    }

    bool isCenterName(const std::string &full) const
    {
        return (full == capsule_name_) || (full == "asbestos_patch::cap_c");
    }

    void onPoseV(const ignition::msgs::Pose_V &msg)
    {
        auto starts_with = [](const std::string &s, const std::string &p)
        {
            return s.rfind(p, 0) == 0; // C++20: s.starts_with(p)
        };

        auto last_segment = [](std::string s) -> std::string
        {
            for (size_t pos = 0; (pos = s.find("::", pos)) != std::string::npos;)
                s.replace(pos, 2, "/"), ++pos;
            const size_t slash = s.rfind('/');
            return (slash == std::string::npos) ? s : s.substr(slash + 1);
        };

        // Cache patch world pose (odom/map) if present in this message
        bool have_patch = false;
        double px = 0, py = 0, pz = 0;         // patch world translation
        double qw = 1, qx = 0, qy = 0, qz = 0; // patch world orientation

        for (int i = 0; i < msg.pose_size(); ++i)
        {
            const auto &po = msg.pose(i);
            // match by model name (last token equals "asbestos_patch")
            if (last_segment(po.name()) == "asbestos_patch")
            {
                px = po.position().x();
                py = po.position().y();
                pz = po.position().z();
                qw = po.orientation().w();
                qx = po.orientation().x();
                qy = po.orientation().y();
                qz = po.orientation().z();
                have_patch = true;
                break;
            }
        }

        // Precompute rotation matrix R from patch quaternion (world ← patch)
        double r00 = 1, r01 = 0, r02 = 0, r10 = 0, r11 = 1, r12 = 0, r20 = 0, r21 = 0, r22 = 1;
        if (have_patch)
        {
            const double xx = qx * qx, yy = qy * qy, zz = qz * qz;
            const double xy = qx * qy, xz = qx * qz, yz = qy * qz;
            const double wx = qw * qx, wy = qw * qy, wz = qw * qz;
            r00 = 1.0 - 2.0 * (yy + zz);
            r01 = 2.0 * (xy - wz);
            r02 = 2.0 * (xz + wy);
            r10 = 2.0 * (xy + wz);
            r11 = 1.0 - 2.0 * (xx + zz);
            r12 = 2.0 * (yz - wx);
            r20 = 2.0 * (xz - wy);
            r21 = 2.0 * (yz + wx);
            r22 = 1.0 - 2.0 * (xx + yy);
        }

        // Publish each capsule in WORLD/ODOM by composing local (patch frame) with patch world pose
        for (int i = 0; i < msg.pose_size(); ++i)
        {
            const auto &po = msg.pose(i);
            const std::string token = last_segment(po.name()); // "cap_0", "cap_c", etc.

            if (!starts_with(token, "cap_"))
                continue; // ignore non-caps

            // Local to the patch
            const double lx = po.position().x();
            const double ly = po.position().y();
            const double lz = po.position().z();

            // Compose: world = patch_T * local
            double wx = lx, wy = ly, wz = lz;
            if (have_patch)
            {
                wx = px + r00 * lx + r01 * ly + r02 * lz;
                wy = py + r10 * lx + r11 * ly + r12 * lz;
                wz = pz + r20 * lx + r21 * ly + r22 * lz;
            }

            geometry_msgs::msg::PoseStamped out;
            out.header.stamp = this->now();  // sim time
            out.header.frame_id = frame_id_; // "odom"
            out.pose.position.x = wx;
            out.pose.position.y = wy;
            out.pose.position.z = wz;
            out.pose.orientation.w = 1.0; // orientation not needed for pick

            getOrCreatePub(token)->publish(out); // /capsule/cap_X/pose
            if (token == "cap_c")
                pub_single_->publish(out); // legacy /capsule/pose
        }
    }

    // members that were missing
    std::string capsule_name_;
    std::string world_topic_;
    std::string frame_id_;
    ignition::transport::Node ign_node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_single_;
    std::unordered_map<std::string,
                       rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
        pubs_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CapsulePoseBridge>());
    rclcpp::shutdown();
    return 0;
}
