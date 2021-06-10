/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser
   This code was taken from the moveit 2 repository and mofied to work with rrbot_arm
   https://github.com/ros-planning/moveit2/tree/main/moveit_demo_nodes/run_moveit_cpp/src
   Desc: A simple demo node running MoveItCpp for planning and execution
*/

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// tf
// #include <chrono>
// #include <geometry_msgs/msg/pose_stamped.h>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/transform_stamped.h>
// #include <tf2/buffer_core.h>
// #include <tf2/time.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/buffer_interface.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/visibility_control.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_demo");

// double get_distance(geometry_msgs::msg::TransformStamped start, geometry_msgs::msg::TransformStamped now) {
  // double xs = start.transform.translation.x;
  // double ys = start.transform.translation.y;
  // double zs = start.transform.translation.z;
  // double xn = now.transform.translation.x;
  // double yn = now.transform.translation.y;
  // double zn = now.transform.translation.z;
  // return pow(pow(xs - xn, 2) + pow(ys - yn, 2) + pow(zs - zn, 2), 0.5);
// }

class MoveItCppDemo {
public:
  MoveItCppDemo(const rclcpp::Node::SharedPtr &node)
      : node_(node),
        robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1)) {
  }
  void moveit_execution(moveit::planning_interface::PlanningComponent &arm) {
    arm.execute();
    execution_completed_ = true;
  }
  void run() {
    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService(); // let RViz display query PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    moveit::planning_interface::PlanningComponent arm("ur10_arm", moveit_cpp_);

    // tf
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // A little delay before running the plan
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Create collision object, planning shouldn't be too easy
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "box";
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {0.03, 0.6, 1.6};
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.3;
    box_pose.position.y = 0.8;
    box_pose.position.z = 1.4;
    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    geometry_msgs::msg::PoseStamped target_pose1;
    target_pose1.header.frame_id = "world";
    target_pose1.pose.orientation.x = 0.0;
    target_pose1.pose.orientation.y = 0.0;
    target_pose1.pose.orientation.z = 0.7068252;
    target_pose1.pose.orientation.w = 0.7073883;
    target_pose1.pose.position.x = -0.4;
    target_pose1.pose.position.y = 0.8;
    target_pose1.pose.position.z = 1.4;
    geometry_msgs::msg::PoseStamped target_pose2;
    target_pose2.header.frame_id = "world";
    target_pose2.pose.orientation.x = 0.0;
    target_pose2.pose.orientation.y = 0.0;
    target_pose2.pose.orientation.z = 0.7068252;
    target_pose2.pose.orientation.w = 0.7073883;
    target_pose2.pose.position.x = 0.8;
    target_pose2.pose.position.y = 0.8;
    target_pose2.pose.position.z = 1.4;

    // Set joint state goal
    arm.setGoal(target_pose1, "ee_link");
    // Run actual plan
    const auto plan_solution = arm.plan();
    if (plan_solution)
      arm.execute();

    // // Add object to planning scene
    { // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object);
    } // Unlock PlanningScene

    arm.setGoal(target_pose2, "ee_link");
    const auto plan_solution2 = arm.plan();
    if (plan_solution2)
      arm.execute();

    // Remove object to planning scene
    { // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
      scene->removeAllCollisionObjects();
    } // Unlock PlanningScene

    arm.setGoal(target_pose1, "ee_link");
    // Run actual plan
    const auto plan_solution3 = arm.plan();
    if (plan_solution3)
      arm.execute();
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;
  //
  // rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  // rclcpp::TimeSource timesource;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  bool execution_completed_ = false;
};

int main(int argc, char **argv) {
  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

  MoveItCppDemo demo(node);
  std::thread run_demo([&demo]() {
    // Let RViz initialize before running demo
    // TODO(henningkayser): use lifecycle events to launch node
    rclcpp::sleep_for(std::chrono::seconds(5));
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();

  return 0;
}
