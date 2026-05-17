#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "moveit_grasp_utils/workspace_validator.hpp"

using namespace std::chrono_literals;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::Pose;

class WorkspaceValidatorNode : public rclcpp::Node
{
public:
    WorkspaceValidatorNode()
    : Node("workspace_validator")
    {
        // Declare + read parameters
        this->declare_parameter("max_reach",   0.82);
        this->declare_parameter("min_reach",   0.18);
        this->declare_parameter("max_height",  0.98);
        this->declare_parameter("min_height", -0.05);

        moveit_grasp_utils::WorkspaceValidator::Params p;
        p.max_reach  = this->get_parameter("max_reach").as_double();
        p.min_reach  = this->get_parameter("min_reach").as_double();
        p.max_height = this->get_parameter("max_height").as_double();
        p.min_height = this->get_parameter("min_height").as_double();
        validator_ = std::make_unique<moveit_grasp_utils::WorkspaceValidator>(p);

        // Subscribers
        detections_sub_ = this->create_subscription<MarkerArray>(
            "/detected_objects", 10,
            std::bind(&WorkspaceValidatorNode::detections_cb, this,
                      std::placeholders::_1));

        // Publishers
        valid_pub_  = this->create_publisher<PoseArray>("/validated_targets", 10);
        ws_viz_pub_ = this->create_publisher<Marker>("/workspace_visualization", 10);

        // Periodic workspace visualisation
        viz_timer_ = this->create_wall_timer(
            2s, std::bind(&WorkspaceValidatorNode::publish_workspace_viz, this));

        RCLCPP_INFO(this->get_logger(),
            "WorkspaceValidator ready  reach=[%.2f, %.2f]m  height=[%.2f, %.2f]m",
            p.min_reach, p.max_reach, p.min_height, p.max_height);
    }

private:
    // ── callbacks ──────────────────────────────────────────────────────────
    void detections_cb(const MarkerArray::SharedPtr msg)
    {
        PoseArray valid_out;
        valid_out.header.stamp    = this->get_clock()->now();
        valid_out.header.frame_id = "panda_link0";

        int accepted = 0;
        int rejected = 0;

        for (const auto & marker : msg->markers) {
            double x = marker.pose.position.x;
            double y = marker.pose.position.y;
            double z = marker.pose.position.z;

            if (validator_->is_reachable(x, y, z)) {
                Pose p;
                p.position    = marker.pose.position;
                p.orientation.w = 1.0;
                valid_out.poses.push_back(p);
                ++accepted;

                RCLCPP_DEBUG(this->get_logger(),
                    "  ACCEPTED  (%.3f, %.3f, %.3f)  ns=%s",
                    x, y, z, marker.ns.c_str());
            } else {
                ++rejected;
                RCLCPP_WARN(this->get_logger(),
                    "  REJECTED  (%.3f, %.3f, %.3f)  reason: %s",
                    x, y, z,
                    validator_->rejection_reason(x, y, z).c_str());
            }
        }

        if (!valid_out.poses.empty()) {
            valid_pub_->publish(valid_out);
        }

        if (accepted + rejected > 0) {
            RCLCPP_DEBUG(this->get_logger(),
                "Validated %d/%d objects as reachable",
                accepted, accepted + rejected);
        }
    }

    // ── workspace visualisation ────────────────────────────────────────────
    void publish_workspace_viz()
    {
        const auto & p = validator_->params();

        // Semi-transparent cylinder showing the reachable workspace shell
        auto outer = build_cylinder_marker(
            0, p.max_reach * 2.0, p.max_height - p.min_height,
            (p.max_height + p.min_height) / 2.0,
            0.0f, 0.8f, 0.0f, 0.08f);
        ws_viz_pub_->publish(outer);

        // Darker inner exclusion zone
        auto inner = build_cylinder_marker(
            1, p.min_reach * 2.0, p.max_height - p.min_height,
            (p.max_height + p.min_height) / 2.0,
            0.8f, 0.2f, 0.0f, 0.12f);
        ws_viz_pub_->publish(inner);
    }

    Marker build_cylinder_marker(int id, double diameter, double height,
                                  double centre_z,
                                  float r, float g, float b, float a)
    {
        Marker m;
        m.header.stamp    = this->get_clock()->now();
        m.header.frame_id = "panda_link0";
        m.ns              = "workspace";
        m.id              = id;
        m.type            = Marker::CYLINDER;
        m.action          = Marker::ADD;

        m.pose.position.z    = centre_z;
        m.pose.orientation.w = 1.0;

        m.scale.x = diameter;
        m.scale.y = diameter;
        m.scale.z = height;

        m.color.r = r;  m.color.g = g;
        m.color.b = b;  m.color.a = a;

        m.lifetime = rclcpp::Duration(3s);
        return m;
    }

    // ── members ────────────────────────────────────────────────────────────
    std::unique_ptr<moveit_grasp_utils::WorkspaceValidator> validator_;

    rclcpp::Subscription<MarkerArray>::SharedPtr detections_sub_;
    rclcpp::Publisher<PoseArray>::SharedPtr      valid_pub_;
    rclcpp::Publisher<Marker>::SharedPtr         ws_viz_pub_;
    rclcpp::TimerBase::SharedPtr                 viz_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WorkspaceValidatorNode>());
    rclcpp::shutdown();
    return 0;
}
