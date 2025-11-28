#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <string>

class MapOdomBroadcaster : public rclcpp::Node {
public:
    MapOdomBroadcaster() : Node("map_odom_broadcaster") {
        pose_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
            "/robot_pose", 10,
            std::bind(&MapOdomBroadcaster::robotPoseCallback, this, std::placeholders::_1));

        tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 100,
            std::bind(&MapOdomBroadcaster::tfCallback, this, std::placeholders::_1));

        tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
    }

private:
    tf2::Transform map_to_base_;
    tf2::Transform odom_to_base_;
    rclcpp::Clock clock_;


    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

    void robotPoseCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped t = *msg;
        RCLCPP_INFO(this->get_logger(), "%s to %s", t.header.frame_id.c_str(), t.child_frame_id.c_str()); 
        if (t.header.frame_id == "map" && t.child_frame_id == "base_link") {
            tf2::fromMsg(t.transform, map_to_base_);
            computeAndPublish();
        }
    }

    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        for (const auto &t : msg->transforms) {
            if (t.header.frame_id == "odom" && t.child_frame_id == "base_link") {
                tf2::fromMsg(t.transform, odom_to_base_);
                computeAndPublish();
                break;
            }
        }
    }

    void computeAndPublish() {
        tf2::Transform map_to_odom = map_to_base_ * odom_to_base_.inverse();

        geometry_msgs::msg::TransformStamped out;
        clock_ = rclcpp::Clock(RCL_ROS_TIME);
        out.header.stamp = this->now();
        out.header.frame_id = "map";
        out.child_frame_id = "odom";
        out.transform = tf2::toMsg(map_to_odom);

        tf2_msgs::msg::TFMessage msg;
        msg.transforms.push_back(out);
        tf_pub_->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOdomBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
