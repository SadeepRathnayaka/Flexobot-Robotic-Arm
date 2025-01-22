#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>


using std::placeholders::_1 ;

class JointControl : public rclcpp::Node
{
public:
    JointControl() : Node("Joint_Control"),
                        move_group_interface(std::make_shared<rclcpp::Node>("moveit_node"), "arm"),
                        move_gripper_interface(std::make_shared<rclcpp::Node>("moveit_gripper_node"), "gripper")
    {
        sub_ = create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JointControl::msgCallback, this, _1));

    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_ ;
    moveit::planning_interface::MoveGroupInterface move_group_interface ;
    moveit::planning_interface::MoveGroupInterface move_gripper_interface ;

    double joint_1 = 0.0;
    double joint_2 = 0.0;
    double joint_3 = 0.0;
    double joint_4 = 0.0;
    double joint_5 = 0.0;

    void msgCallback(const sensor_msgs::msg::Joy &msg)
    {   
        if (msg.axes.at(0) > 0.2)
        {   
            joint_1 += 0.0872 ;
            move_group_interface.setJointValueTarget("joint_1", joint_1);
            move_group_interface.setMaxVelocityScalingFactor(0.5);
            move_group_interface.move();
            
        }

        else if (msg.axes.at(0) < -0.2)
        {   
            joint_1 -= 0.0872 ;
            move_group_interface.setJointValueTarget("joint_1", joint_1);
            move_group_interface.setMaxVelocityScalingFactor(0.5);
            move_group_interface.move();
            
        }

        else if (msg.axes.at(1) > 0.2)
        {   
            joint_2 += 0.0872 ;
            move_group_interface.setJointValueTarget("joint_2", joint_2);
            move_group_interface.setMaxVelocityScalingFactor(0.5);
            move_group_interface.move();
            
        }

        else if (msg.axes.at(1) < -0.2)
        {   
            joint_2 -= 0.0872 ;
            move_group_interface.setJointValueTarget("joint_2", joint_2);
            move_group_interface.setMaxVelocityScalingFactor(0.5);
            move_group_interface.move();
            
        }

        else if (msg.axes.at(4) > 0.2)
        {   
            joint_3 -= 0.0872 ;
            move_group_interface.setJointValueTarget("joint_3", joint_3);
            move_group_interface.setMaxVelocityScalingFactor(0.5);
            move_group_interface.move();
            
        }

        else if (msg.axes.at(4) < -0.2)
        {   
            joint_3 += 0.0872 ;
            move_group_interface.setJointValueTarget("joint_3", joint_3);
            move_group_interface.setMaxVelocityScalingFactor(0.5);
            move_group_interface.move();
            
        }

        else if (msg.axes.at(3) > 0.2)
        {   
            joint_4 += 0.0872 ;
            move_group_interface.setJointValueTarget("joint_4", joint_4);
            move_group_interface.setMaxVelocityScalingFactor(0.5);
            move_group_interface.move();
            
        }

        else if (msg.axes.at(3) < -0.2)
        {   
            joint_4 -= 0.0872 ;
            move_group_interface.setJointValueTarget("joint_4", joint_4);
            move_group_interface.setMaxVelocityScalingFactor(0.5);
            move_group_interface.move();
            
        }

        else if (msg.axes.at(7) > 0.2)
        {   
            joint_5 -= 0.0872 ;
            move_group_interface.setJointValueTarget("joint_5", joint_5);
            move_group_interface.setMaxVelocityScalingFactor(0.5);
            move_group_interface.move();
            
        }

        else if (msg.axes.at(7) < -0.2)
        {   
            joint_5 += 0.0872 ;
            move_group_interface.setJointValueTarget("joint_5", joint_5);
            move_group_interface.setMaxVelocityScalingFactor(0.5);
            move_group_interface.move();
            
        }

        else if (msg.buttons.at(2) == 1 )
        {   
            move_gripper_interface.setJointValueTarget("joint_6", -0.6);
            move_gripper_interface.move();
            
        }

        else if (msg.buttons.at(1) == 1 )
        {   
            move_gripper_interface.setJointValueTarget("joint_6", 0.0);
            move_gripper_interface.move();
            
        }

        else if (msg.buttons.at(8) == 1 )
        {   
            move_group_interface.setJointValueTarget("joint_1", 0.0);
            move_group_interface.setJointValueTarget("joint_2", 0.0);
            move_group_interface.setJointValueTarget("joint_3", 0.0);
            move_group_interface.setJointValueTarget("joint_4", 0.0);
            move_group_interface.setJointValueTarget("joint_5", 0.0);
            move_gripper_interface.setJointValueTarget("joint_6", 0.0);
            move_group_interface.move();
            move_gripper_interface.move();
            
        }

    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointControl>());
    rclcpp::shutdown();
    return 0;
}