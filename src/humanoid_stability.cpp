/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
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
 *   * Neither the name of the JSK, The University of Tokyo nor the names of its
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

/* Author: Dave Coleman
   Desc:   Wrapper for hrl_kinematics for calculing COM
*/

#include <moveit_humanoid_stability/humanoid_stability.h>

namespace moveit_humanoid_stability
{

HumanoidStability::HumanoidStability(bool verbose, const moveit::core::RobotState &robot_state,
                                     const moveit_visual_tools::VisualToolsPtr &visual_tools)
  : verbose_(verbose)
  , visual_tools_(visual_tools)
  , normal_vector_(0.0, 0.0, 1.0)
  , support_mode_(hrl_kinematics::Kinematics::SUPPORT_SINGLE_LEFT) // TODO choose smartly
{
  normal_vector_.normalize(); // TODO is this necessary?

  ROS_INFO_STREAM_NAMED("stability","HumanoidStability Ready.");

  // Load the torso (vjoint) bounds from yaml file
  ros::NodeHandle nh("~");
  static const std::string group_name = "hrp2jsknt_torso"; // TODO remove name
  nh.param(group_name + "/min_x", min_x_, 0.0);
  nh.param(group_name + "/max_x", max_x_, 0.0);
  nh.param(group_name + "/min_y", min_y_, 0.0);
  nh.param(group_name + "/max_y", max_y_, 0.0);
  nh.param(group_name + "/min_z", min_z_, 0.0);
  nh.param(group_name + "/max_z", max_z_, 0.0);
  // Add padding to all limits just in case
  const double vjoint_limit_padding = 0.1;
  min_x_ += vjoint_limit_padding;
  max_x_ += vjoint_limit_padding;
  min_y_ += vjoint_limit_padding;
  max_y_ += vjoint_limit_padding;
  min_z_ += vjoint_limit_padding;
  max_z_ += vjoint_limit_padding;

  // Get the offset between the foot and the torso to allow correct positioning of bounding box
  moveit::core::RobotState temp_state(robot_state);
  temp_state.setToDefaultValues();
  Eigen::Affine3d left_foot_to_torso = temp_state.getGlobalLinkTransform("LLEG_LINK5"); // TODO remove name

  // Move the x & y bounds over from foot frame of reference to torso frame of reference. z is always w.r.t. ground (0)
  max_x_ -= left_foot_to_torso.translation().x();
  max_y_ -= left_foot_to_torso.translation().y();
  min_x_ -= left_foot_to_torso.translation().x();
  min_y_ -= left_foot_to_torso.translation().y();

  // Setup HRL Kinematics Balance Constraint
  robot_joint_group_ = robot_state.getRobotModel()->getJointModelGroup("robot_joints"); // TODO remove name
  //hrl_kinematics::Kinematics::FootSupport support_mode_ = hrl_kinematics::Kinematics::SUPPORT_DOUBLE;

  // Preload map for less memory usage
  for (std::size_t i = 0; i < robot_joint_group_->getVariableCount(); ++i)
  {
    // Intitialize empty
    joint_positions_map_.insert(std::make_pair(robot_joint_group_->getJointModels()[i]->getName(), 0));
  }

  // Error check
  if (!visual_tools_ && verbose_)
  {
    ROS_ERROR_STREAM_NAMED("temp","No visual_tools passed in when in verbose mode, turning off verbose");
    verbose_ = false;
  }
}

bool HumanoidStability::isValid(const robot_state::RobotState &robot_state, bool verbose)
{
  //verbose_ = verbose; // TODO - this should be enabled so we obey the planner

  // Publish state
  if (verbose_)
  {
    visual_tools_->publishRobotState(robot_state);
    ros::Duration(0.25).sleep();
  }

  // Check torso
  if (!isApproximateValidBase(robot_state))
  {
    if (verbose_)
      ROS_WARN_STREAM_NAMED("temp","Invalid because of approximate base location");
    return false;
  }

  // Check COM
  if (!isApproximateValidFoot(robot_state))
  {
    if (verbose_)
      ROS_WARN_STREAM_NAMED("temp","Invalid because of approximate foot location");
    return false;
  }

  // Check COM
  if (!isValidCOM(robot_state))
  {
    if (verbose_)
      ROS_WARN_STREAM_NAMED("temp","Invalid because of COM");
    return false;
  }

  return true;
}

bool HumanoidStability::isApproximateValidBase(const robot_state::RobotState &robot_state)
{
  // Check if vjoint (torso) is within reasonable limits
  const robot_model::JointModel *vjoint = robot_state.getJointModel("virtual_joint"); // TODO unhard code
  const double* vjoint_positions = robot_state.getJointPositions(vjoint);
  if (verbose_)
  {
    /*
      std::cout << "Vjoint: " << std::endl;
      std::cout << "  X: " << boost::format("%8.4f") % min_x_ << boost::format("%8.4f") % vjoint_positions[0]
      << boost::format("%8.4f") % max_x_ << std::endl;
      std::cout << "  Y: " << boost::format("%8.4f") % min_y_ << boost::format("%8.4f") % vjoint_positions[1]
      << boost::format("%8.4f") % max_y_ << std::endl;
      std::cout << "  Z: " << boost::format("%8.4f") % min_z_ << boost::format("%8.4f") % vjoint_positions[2]
      << boost::format("%8.4f") % max_z_ << std::endl;
    */
    visual_tools_->deleteAllMarkers();
    displayBoundingBox(robot_state.getFakeBaseTransform());
  }

  if (vjoint_positions[0] < min_x_ + robot_state.getFakeBaseTransform().translation().x() ||
      vjoint_positions[0] > max_x_ + robot_state.getFakeBaseTransform().translation().x() ||
      vjoint_positions[1] < min_y_ + robot_state.getFakeBaseTransform().translation().y() ||
      vjoint_positions[1] > max_y_ + robot_state.getFakeBaseTransform().translation().y() ||
      vjoint_positions[2] < min_z_ + robot_state.getFakeBaseTransform().translation().z() ||
      vjoint_positions[2] > max_z_ + robot_state.getFakeBaseTransform().translation().z() )
  {
    return false;
  }
  return true;
}

bool HumanoidStability::isApproximateValidFoot(const robot_state::RobotState &robot_state)
{
  const robot_model::LinkModel  *rfoot  = robot_state.getRobotModel()->getLinkModel("RLEG_LINK5"); // TODO move

  if (robot_state.getGlobalLinkTransform(rfoot).translation().z() < 0)
    return false;

  return true;
}

bool HumanoidStability::isValidCOM(const robot_state::RobotState &robot_state)
{
  // Copy robot state to map
  for (std::size_t i = 0; i < robot_joint_group_->getVariableCount(); ++i)
  {
    joint_positions_map_[robot_joint_group_->getJointModels()[i]->getName()] =
      robot_state.getJointPositions(robot_joint_group_->getJointModels()[i])[0];
  }

  bool stable = test_stability_.isPoseStable(joint_positions_map_, support_mode_, normal_vector_);

  bool show_com_makers = false;

  // Publish COM marker
  if (verbose_ && show_com_makers)
  {
    visualization_msgs::Marker com_marker = test_stability_.getCOMMarker();
    // Translate to world frame
    com_marker.pose =
      moveit_visual_tools::VisualTools::convertPose(moveit_visual_tools::VisualTools::convertPose(com_marker.pose) *
                                                    robot_state.getGlobalLinkTransform(com_marker.header.frame_id));

    // Change frame name to world fame
    com_marker.header.frame_id = robot_state.getRobotModel()->getModelFrame(); // odom
    com_marker.ns = "COM";
    visual_tools_->publishMarker(com_marker);
  }

  // Publish footprint polygon
  if (verbose_ && show_com_makers)
  {
    const geometry_msgs::PolygonStamped polygon_msg = test_stability_.getSupportPolygon();

    // Change polygon points to world frame
    std::vector<geometry_msgs::Point> points;
    for (std::size_t i = 0; i < polygon_msg.polygon.points.size(); ++i)
    {
      Eigen::Affine3d temp_pose = moveit_visual_tools::VisualTools::convertPoint32ToPose(polygon_msg.polygon.points[i]);

      //temp_pose.translation().z() = temp_pose.translation().z() - 0.1;
      //robot_state.printTransform(robot_state.getGlobalLinkTransform("BODY"));
      temp_pose = temp_pose * robot_state.getGlobalLinkTransform("BODY");

      points.push_back( visual_tools_->convertPose(temp_pose).position );
    }
    points.push_back(points.front()); // connect first and last points for last line

    visual_tools_->publishPath(points);
  }

  return stable;
}

bool HumanoidStability::displayBoundingBox(const Eigen::Affine3d &translation) const
{
  if (!visual_tools_)
  {
    ROS_WARN_STREAM_NAMED("temp","visual tools not loaded");
    return false;
  }

  geometry_msgs::Point point1;
  geometry_msgs::Point point2;
  point1.x = max_x_ + translation.translation().x();
  point1.y = max_y_ + translation.translation().y();
  point1.z = max_z_ + translation.translation().z();
  point2.x = min_x_ + translation.translation().x();
  point2.y = min_y_ + translation.translation().y();
  point2.z = min_z_ + translation.translation().z();

  return visual_tools_->publishRectangle(point1, point2, moveit_visual_tools::TRANSLUCENT);
}

void HumanoidStability::printVirtualJointExtremes() const
{
  std::cout << "Virtual Joint Extremes: " << std::endl;
  std::cout << "  min_x: " << min_x_ << std::endl;
  std::cout << "  max_x: " << max_x_ << std::endl;
  std::cout << "  min_y: " << min_y_ << std::endl;
  std::cout << "  max_y: " << max_y_ << std::endl;
  std::cout << "  min_z: " << min_z_ << std::endl;
  std::cout << "  max_z: " << max_z_ << std::endl;
}

} // end namespace
