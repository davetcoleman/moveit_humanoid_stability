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

#ifndef MOVEIT_HUMANOID_STABILITY__HUMANOID_STABILITY
#define MOVEIT_HUMANOID_STABILITY__HUMANOID_STABILITY

// ROS
#include <ros/ros.h>

// Helper for Rviz
#include <moveit_visual_tools/visual_tools.h>

// Humanoid balance constraint tester
#include <hrl_kinematics/TestStability.h>

namespace moveit_humanoid_stability
{

class HumanoidStability
{
private:

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // For visualizing things in rviz
  moveit_visual_tools::VisualToolsPtr visual_tools_;

  // Bounds for estimating virtual joint contraint
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double min_z_;
  double max_z_;

  // Model semantics
  const robot_model::LinkModel* left_foot_;
  const robot_model::LinkModel* right_foot_;
  const robot_model::JointModelGroup* all_joints_group_;

  // Stability checker
  hrl_kinematics::TestStability test_stability_;
  std::map<std::string, double> joint_positions_map_;
  hrl_kinematics::Kinematics::FootSupport support_mode_;
  tf::Vector3 normal_vector_;

public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   * \param robot_state
   * \param visual_tools - shared visual tools ptr for debugging (optional)
   */
  HumanoidStability(bool verbose, const moveit::core::RobotState &robot_state, 
                    const moveit_visual_tools::VisualToolsPtr &visual_tools = moveit_visual_tools::VisualToolsPtr());

  /**
   * \brief Destructor
   */
  ~HumanoidStability() {};

  /**
   * \brief Helper for setting a planning scene state validator function
   * \return boost function pointer for use in PlanningScene
   */
  planning_scene::StateFeasibilityFn getStateFeasibilityFn()
  {
    return boost::bind(&HumanoidStability::isValid, this, _1, _2);
  }

  /**
   * \brief Check if a robot state is stable/balanced based on all checks including heuristics
   *        NOTE: this function can be used as a planning_scene StateFeasibilityFn
   * \param robot_state - input state to check
   * \return true if stable
   */
  bool isValid(const robot_state::RobotState &robot_state, bool verbose);

  /**
   * \brief Run quick/low-cost virtual joint bounding box test on state to see if vjoint is within rough bounds
   * \param robot_state - input state to check
   * \return true if within bounds
   */
  bool isApproximateValidBase(const robot_state::RobotState &robot_state);

  /**
   * \brief Run quick/low-cost other leg test to see if its above the z 0 value, e.g. above ground
   * \param robot_state - input state to check
   * \return true if within bounds
   */
  bool isApproximateValidFoot(const robot_state::RobotState &robot_state);

  /**
   * \brief Run HRL kinematics COM calculation
   * \param robot_state - input state to check
   * \return true if stable
   */
  bool isValidCOM(const robot_state::RobotState &robot_state);

  /**
   * \brief Debug function for showing the course-grain constraint enforcement of torso
   * \return true on success
   */
  bool displayBoundingBox(const Eigen::Affine3d &translation = Eigen::Affine3d::Identity()) const;

  /**
   * \brief For debugging
   */
  void printVirtualJointExtremes() const;

  /**
   * \brief Getter for Verbose
   */ 
  bool getVerbose()
  {
    return verbose_;
  }
  
  /**
   * \brief Setter for Verbose
   */
  void setVerbose(bool verbose)
  {
    verbose_ = verbose;
  }
  

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<HumanoidStability> HumanoidStabilityPtr;
typedef boost::shared_ptr<const HumanoidStability> HumanoidStabilityConstPtr;

} // end namespace

#endif
