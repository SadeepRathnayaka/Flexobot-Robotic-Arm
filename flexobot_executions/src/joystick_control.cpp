#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>

using std::placeholders::_1;

class JoystickControl : public rclcpp::Node
{
public:
    JoystickControl() : Node("Joystick_Control"),
                        move_group_interface(std::make_shared<rclcpp::Node>("moveit_node"), "arm")
    {
        sub_ = create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoystickControl::msgCallback, this, _1));

        move_group_interface.setPlanningTime(5.0);
        move_group_interface.setPoseReferenceFrame("base_link");
        move_group_interface.setGoalPositionTolerance(0.015);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void msgCallback(const sensor_msgs::msg::Joy &msg)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform("base_link", "end_eff", rclcpp::Time(0));
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform base_link to end_eff: %s", ex.what());
            return;
        }

        Eigen::Isometry3d current_transform = tf2::transformToEigen(transform_stamped);
        Eigen::Vector3d current_translation = current_transform.translation();
        Eigen::Matrix3d current_rotation = current_transform.rotation();
        RCLCPP_INFO(this->get_logger(), "Current Pose: Translation = [%.3f, %.3f, %.3f], Rotation = [%.3f, %.3f, %.3f]",
                    current_translation.x(), current_translation.y(), current_translation.z(),
                    current_rotation(0, 0), current_rotation(0, 1), current_rotation(0, 2));

        // Define movement step
        Eigen::Vector3d movement_step(0.03, 0.0, 0.0);

        // Handle forward/backward movement on axis[1]
        if (msg.axes.at(1) > 0.2)
        {
            // Move forward along the x-axis
            Eigen::Vector3d global_translation = current_transform.rotation() * movement_step;
            Eigen::Vector3d new_position = current_transform.translation() + global_translation;
            RCLCPP_INFO(this->get_logger(), "Moving Forward: Target Pose Translation = [%.3f, %.3f, %.3f]",
                        new_position.x(), new_position.y(), new_position.z());
            move_group_interface.setStartStateToCurrentState();
            move_group_interface.setPositionTarget(new_position.x(), new_position.y(), new_position.z(), "link_6");
        }
        else if (msg.axes.at(1) < -0.2)
        {
            // Move backward along the x-axis
            Eigen::Vector3d global_translation = current_transform.rotation() * (-movement_step);
            Eigen::Vector3d new_position = current_transform.translation() + global_translation;
            RCLCPP_INFO(this->get_logger(), "Moving Backward: Target Pose Translation = [%.3f, %.3f, %.3f]",
                        new_position.x(), new_position.y(), new_position.z());
            move_group_interface.setStartStateToCurrentState();
            move_group_interface.setPositionTarget(new_position.x(), new_position.y(), new_position.z(), "link_6");
        }
        // Handle left/right movement on axis[0]
        else if (msg.axes.at(0) > 0.2)
        {
            // Move left along the y-axis (negative direction of y)
            movement_step = Eigen::Vector3d(0.0, -0.03, 0.0);
            Eigen::Vector3d global_translation = current_transform.rotation() * movement_step;
            Eigen::Vector3d new_position = current_transform.translation() + global_translation;
            RCLCPP_INFO(this->get_logger(), "Moving Left: Target Pose Translation = [%.3f, %.3f, %.3f]",
                        new_position.x(), new_position.y(), new_position.z());
            move_group_interface.setStartStateToCurrentState();
            move_group_interface.setPositionTarget(new_position.x(), new_position.y(), new_position.z(), "link_6");
        }
        else if (msg.axes.at(0) < -0.2)
        {
            // Move right along the y-axis (positive direction of y)
            movement_step = Eigen::Vector3d(0.0, 0.03, 0.0);
            Eigen::Vector3d global_translation = current_transform.rotation() * movement_step;
            Eigen::Vector3d new_position = current_transform.translation() + global_translation;
            RCLCPP_INFO(this->get_logger(), "Moving Right: Target Pose Translation = [%.3f, %.3f, %.3f]",
                        new_position.x(), new_position.y(), new_position.z());
            move_group_interface.setStartStateToCurrentState();
            move_group_interface.setPositionTarget(new_position.x(), new_position.y(), new_position.z(), "link_6");
        }

        // Execute the move
        moveit::planning_interface::MoveItErrorCode success = move_group_interface.move();
        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Move succeeded!");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Move failed or timeout reached!");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickControl>());
    rclcpp::shutdown();
    return 0;
}
