#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <string>
#include <vector>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

class UdpRosNode : public rclcpp::Node {
public:
    UdpRosNode() : Node("udp_ros_node") {
        headset_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("headset", 10);
        left_hand_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("left_hand", 10);
        right_hand_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("right_hand", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        udp_thread_ = std::thread(&UdpRosNode::udp_server, this, 5005);
    }

    ~UdpRosNode() {
        running_ = false;
        if (udp_thread_.joinable()) udp_thread_.join();
    }

private:
    void udp_server(int port) {
        int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in servaddr{};
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(port);

        bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr));
        char buffer[8192];

        while (rclcpp::ok() && running_) {
            ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer)-1, 0, nullptr, nullptr);
            if (n > 0) {
                buffer[n] = '\0';
                std::string message(buffer);
                std::vector<std::string> parts = split(message, ';');

                std::string left_pos, left_rot, right_pos, right_rot, head_pos, head_rot;
                for (const auto& part : parts) {
                    if (part.find("LeftHandPos") == 0) left_pos = part;
                    else if (part.find("LeftHandRot") == 0) left_rot = part;
                    else if (part.find("RightHandPos") == 0) right_pos = part;
                    else if (part.find("RightHandRot") == 0) right_rot = part;
                    else if (part.find("HeadsetPos") == 0) head_pos = part;
                    else if (part.find("HeadsetRot") == 0) head_rot = part;
                }

                if (!left_pos.empty() && !left_rot.empty()) {
                    auto pose = parse_pose(left_pos, left_rot, "left_hand");
                    left_hand_pub_->publish(pose);
                    publish_tf(pose, "world", "left_hand");
                }
                if (!right_pos.empty() && !right_rot.empty()) {
                    auto pose = parse_pose(right_pos, right_rot, "right_hand");
                    right_hand_pub_->publish(pose);
                    publish_tf(pose, "world", "right_hand");
                }
                if (!head_pos.empty() && !head_rot.empty()) {
                    auto pose = parse_pose(head_pos, head_rot, "headset");
                    headset_pub_->publish(pose);
                    publish_tf(pose, "world", "headset");
                }
            }
        }
        close(sockfd);
    }

    // Helper to split string by delimiter
    std::vector<std::string> split(const std::string& s, char delimiter) {
        std::vector<std::string> tokens;
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delimiter)) {
            tokens.push_back(item);
        }
        return tokens;
    }

    bool parse_position(const std::string& str, geometry_msgs::msg::Point& pos) {
        auto colon = str.find(':');
        if (colon == std::string::npos) return false;
        std::string values = str.substr(colon + 1);
        std::stringstream ss(values);
        std::string item;
        std::vector<double> vals;
        while (std::getline(ss, item, ',')) {
            vals.push_back(std::stod(item));
        }
        if (vals.size() != 3) return false;

        // Unity (x, y, z)
        tf2::Vector3 unity_vec(vals[0], vals[1], vals[2]);

        // Rotation: -90 deg about X, then -90 deg about Z
        tf2::Quaternion rot_x, rot_z;
        rot_x.setRPY(M_PI_2, 0, 0); // -90° about X
        rot_z.setRPY(0, 0, M_PI_2); // -90° about Z

        tf2::Quaternion total_rot = rot_z * rot_x;
        tf2::Vector3 ros_vec = tf2::quatRotate(total_rot, unity_vec);

        pos.x = ros_vec.x();
        pos.y = -ros_vec.y();
        pos.z = ros_vec.z();
        return true;
    }

    // Helper: parse rotation string "NameRot:x,y,z,w" and convert Unity -> ROS
    bool parse_rotation(const std::string& str, geometry_msgs::msg::Quaternion& quat, std::string type) {
        auto colon = str.find(':');
        if (colon == std::string::npos) return false;
        std::string values = str.substr(colon + 1);
        std::stringstream ss(values);
        std::string item;
        std::vector<double> vals;
        while (std::getline(ss, item, ',')) {
            vals.push_back(std::stod(item));
        }
        if (vals.size() != 4) return false;

        // Unity quaternion (x, y, z, w)
        tf2::Quaternion unity_q(vals[0], vals[1], vals[2], vals[3]);

        if (type == "headset") {
            tf2::Quaternion ros_q = unity_q;

            quat.x = -ros_q.z();
            quat.y = ros_q.x();
            quat.z = -ros_q.y();
            quat.w = ros_q.w();
        }
        else if (type == "left_hand" || type == "right_hand") {
            // Rotation: +90 deg about X, then -90 deg about Z
            tf2::Quaternion rot_x, rot_z;
            rot_x.setRPY(M_PI_2, 0, 0);  // +90° about X

            // Apply rotation: ROS_q = rot_z * rot_x * unity_q
            tf2::Quaternion ros_q = rot_x * unity_q;

            quat.x = ros_q.y();
            quat.y = ros_q.x();
            quat.z = -ros_q.z();
            quat.w = ros_q.w();
        }

        return true;
    }

    // Updated: parse_pose now takes both pos and rot strings
    geometry_msgs::msg::PoseStamped parse_pose(const std::string& pos_str, const std::string& rot_str, std::string type) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = "world";
        parse_position(pos_str, pose.pose.position);
        parse_rotation(rot_str, pose.pose.orientation, type);
        return pose;
    }

    void publish_tf(const geometry_msgs::msg::PoseStamped& pose, const std::string& parent, const std::string& child) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = pose.header.stamp;
        tf_msg.header.frame_id = parent;
        tf_msg.child_frame_id = child;
        tf_msg.transform.translation.x = pose.pose.position.x;
        tf_msg.transform.translation.y = pose.pose.position.y;
        tf_msg.transform.translation.z = pose.pose.position.z;
        tf_msg.transform.rotation = pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    std::thread udp_thread_;
    std::atomic<bool> running_{true};
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr headset_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_hand_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_hand_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UdpRosNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}