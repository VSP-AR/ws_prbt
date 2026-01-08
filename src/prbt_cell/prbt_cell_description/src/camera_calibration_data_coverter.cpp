#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rigid_transform_node");
    tf2_ros::Buffer tfBuffer(node->get_clock());
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster tfBroadcaster(node);

    geometry_msgs::msg::TransformStamped transformA, transformB, transformC, transformD;

    // Assume you get the transforms into transformA, B and C.
    try
    {
        transformA =
            tfBuffer.lookupTransform("prbt_tool0", "camera_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(5.0));
        transformB = tfBuffer.lookupTransform("camera_link", "camera_color_optical_frame", rclcpp::Time(0),
                                              rclcpp::Duration::from_seconds(5.0));
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(node->get_logger(), "%s", ex.what());
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), "prbt_tool0 to camera_link: %s", geometry_msgs::msg::to_yaml(transformA).c_str());
    RCLCPP_INFO(node->get_logger(), "camera_link to camera_color_optical_frame: %s",
                geometry_msgs::msg::to_yaml(transformB).c_str());

    transformC.header.stamp = node->now(); // Current time
    transformC.header.frame_id = "prbt_tool0";
    transformC.child_frame_id = "camera_color_optical_frame";
    transformC.transform.translation.x = -0.0298741;
    transformC.transform.translation.y = -0.0595027;
    transformC.transform.translation.z = 0.0357333;
    transformC.transform.rotation.x = -0.0147517;
    transformC.transform.rotation.y = -0.000349953;
    transformC.transform.rotation.z = 0.00112926;
    transformC.transform.rotation.w = 0.99989;

    // Compute D
    tf2::Transform A, B, C, D;
    tf2::fromMsg(transformA.transform, A);
    tf2::fromMsg(transformB.transform, B);
    tf2::fromMsg(transformC.transform, C);
    D = C * B.inverse();

    // Set the details for transformD
    transformD.header.stamp = node->now();
    transformD.header.frame_id = "prbt_tool0";
    transformD.child_frame_id = "camera_base";
    transformD.transform = tf2::toMsg(D);

    RCLCPP_INFO(node->get_logger(), "new prbt_tool0 to camera_link: %s",
                geometry_msgs::msg::to_yaml(transformD).c_str());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
