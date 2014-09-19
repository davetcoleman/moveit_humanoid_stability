/*********************************************************************
 * Software License Agreement ("Modified BSD License")
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
   Desc:   Samples 1 leg joint random, checks for partial validity, 
           then samples rest of joints randomly adn checks for validity again
*/

#ifndef MOVEIT_HUMANOID_STABILITY__HUMANOID_CONSTRAINT_SAMPLER_
#define MOVEIT_HUMANOID_STABILITY__HUMANOID_CONSTRAINT_SAMPLER_

#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit/constraint_samplers/constraint_sampler_allocator.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <random_numbers/random_numbers.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// Stability checker
#include <moveit_humanoid_stability/humanoid_stability.h>

// Helper for Rviz
#include <moveit_visual_tools/visual_tools.h>

namespace moveit_humanoid_stability
{

/**
 * \brief HumanoidConstraintSampler is a class that allows the sampling
 * of joints in a particular group of the robot, subject to a set of individual joint constraints.
 *
 * The set of individual joint constraint reduce the allowable bounds
 * used in the sampling.  Unconstrained values will be sampled within
 * their limits.
 *
 */
class HumanoidConstraintSampler : public constraint_samplers::ConstraintSampler
{
public:

  /**
   * Constructor
   *
   * @param [in] scene The planning scene used to check the constraint
   *
   * @param [in] group_name The group name associated with the
   * constraint.  Will be invalid if no group name is passed in or the
   * joint model group cannot be found in the kinematic model
   *
   */
  HumanoidConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene,
                             const std::string &group_name)
    : ConstraintSampler(scene, group_name)
  {
  }

  /**
   * \brief Configures a joint constraint given a Constraints message.
   *
   * If more than one constraint for a particular joint is specified,
   * the most restrictive set of bounds will be used (highest minimum
   * value, lowest maximum value).  For the configuration to be
   * successful, the following condition must be met, in addition to
   * the conditions specified in \ref configure(const std::vector<kinematic_constraints::JointConstraint> &jc) :

   * \li The Constraints message must contain one or more valid joint
   * constraints (where validity is judged by the ability to configure
   * a \ref JointConstraint)
   *
   * @param [in] constr The message containing the constraints
   *
   * @return True if the conditions are met, otherwise false
   */
  virtual bool configure(const moveit_msgs::Constraints &constr);

  /**
   * \brief Helper for setting up the visualizer in Rviz
   */
  void loadVisualTools();

  /**
   * \brief Configures a joint constraint given a vector of constraints.
   *
   * If more than one constraint for a particular joint is specified,
   * the most restrictive set of bounds will be used (highest minimum
   * value, lowest_maximum value.  For the configuration to be
   * successful, the following conditions must be met:

   * \li The vector must contain one or more valid, enabled joint
   * constraints
   *
   * \li At least one constraint must reference a joint in the
   * indicated group.  If no additional bounds exist for this group,
   * then RobotState::setToRandomPositions() can be
   * used to generate a sample independently from the
   * constraint_samplers infrastructure.
   *
   * \li The constraints must allow a sampleable region for all
   * joints, where the most restrictive minimum bound is less than the
   * most restrictive maximum bound
   *
   * @param [in] jc The vector of joint constraints
   *
   * @return True if the conditions are met, otherwise false
   */
  bool configureJoint(const std::vector<kinematic_constraints::JointConstraint> &jc);

  virtual bool sample(robot_state::RobotState &robot_state, const robot_state::RobotState &ks,
                      unsigned int max_attempts);

  bool sampleJoints(robot_state::RobotState &robot_state);

  virtual bool project(robot_state::RobotState &robot_state,
                       unsigned int max_attempts);

  /**
   * \brief Gets the number of constrained joints - joints that have an
   * additional bound beyond the joint limits.
   *
   *
   * @return The number of constrained joints.
   */
  std::size_t getConstrainedJointCount() const
  {
    return bounds_.size();
  }

  /**
   * \brief Gets the number of unconstrained joints - joint that have
   * no additional bound beyond the joint limits.
   *
   * @return The number of unconstrained joints.
   */
  std::size_t getUnconstrainedJointCount() const
  {
    return unbounded_.size();
  }

  /**
   * \brief Get the name of the constraint sampler, for debugging purposes
   * should be in CamelCase format.
   * \return string of name
   */
  virtual const std::string& getName() const
  {
    //printVirtualJointExtremes();
    static const std::string SAMPLER_NAME = "HumanoidConstraintSampler";
    return SAMPLER_NAME;
  }
  
  /**
   * \brief Override so we can set verbose flag in sub components
   */
  virtual void setVerbose(bool verbose);
  
  private:


  protected:

  /// \brief An internal structure used for maintaining constraints on a particular joint
  struct JointInfo
  {
    /**
     * \brief Constructor
     *
     * @return
     */
    JointInfo()
    {
      min_bound_ = -std::numeric_limits<double>::max();
      max_bound_ = std::numeric_limits<double>::max();
    }

    /**
     * \brief Function that adjusts the joints only if they are more
     * restrictive.  This means that the min limit is higher than the
     * current limit, or the max limit is lower than the current max
     * limit.
     *
     * @param min The min limit for potential adjustment
     * @param max The max limit for potential adjustment
     */
    void potentiallyAdjustMinMaxBounds(double min, double max)
    {
      min_bound_ = std::max(min, min_bound_);
      max_bound_ = std::min(max, max_bound_);
    }

    double min_bound_;          /**< The most restrictive min value of those set */
    double max_bound_;          /**< The most restrictive max value of those set */
    std::size_t index_;         /**< The index within the joint state vector for this joint */
  };

  virtual void clear();

  random_numbers::RandomNumberGenerator           random_number_generator_; /**< \brief Random number generator used to sample */
  std::vector<JointInfo>                          bounds_; /**< \brief The bounds for any joint with bounds that are more restrictive than the joint limits */

  std::vector<const robot_model::JointModel*> unbounded_; /**< \brief The joints that are not bounded except by joint limits */
  std::vector<unsigned int>                       uindex_; /**< \brief The index of the unbounded joints in the joint state vector */
  std::vector<double>                             values_; /**< \brief Values associated with this group to avoid continuously reallocating */

  std::string sampler_name_; // used for debugging

  // Leg IK solvers
  //const robot_model::JointModelGroup* left_leg_;
  //const robot_model::JointModelGroup* right_leg_;

  // Store desired feet positions
  //Eigen::Affine3d left_foot_position_;
  //Eigen::Affine3d right_foot_position_;

  // Store foot to torso translation
  //Eigen::Affine3d left_foot_to_torso_;

  // Allocate memory for storing transforms of feet
  //Eigen::Affine3d left_foot_position_new_;
  //Eigen::Affine3d right_foot_position_new_;

  // Verbose mode
  geometry_msgs::Pose text_pose_;

  // For visualizing things in rviz
  moveit_visual_tools::VisualToolsPtr visual_tools_;

  // Tool for checking balance
  moveit_humanoid_stability::HumanoidStabilityPtr humanoid_stability_;

  // Properties of robot's standing stance
  const robot_model::JointModelGroup *whole_body_minus_fixed_leg_;
  const robot_model::JointModelGroup *fixed_leg_;
  const robot_model::LinkModel* left_foot_;
  const robot_model::LinkModel* right_foot_;
  // Robot model properties
  const robot_model::JointModelGroup *whole_body_minus_left_leg_;
  const robot_model::JointModelGroup *whole_body_minus_right_leg_;
  const robot_model::JointModelGroup *left_leg_;
  const robot_model::JointModelGroup *right_leg_;

};

// define the sampler allocator plugin interface
class HumanoidConstraintSamplerAllocator : public constraint_samplers::ConstraintSamplerAllocator
{
public:

  virtual constraint_samplers::ConstraintSamplerPtr alloc(const planning_scene::PlanningSceneConstPtr &scene,
                                                          const std::string &group_name, const moveit_msgs::Constraints &constr)
  {
    constraint_samplers::ConstraintSamplerPtr cs(new HumanoidConstraintSampler(scene, group_name));
    cs->configure(constr);
    return cs;
  }

  virtual bool canService(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name,
                          const moveit_msgs::Constraints &constr) const
  {
    // override: always use
    return true;

    // do not use this sampler if there are any joint constraints, because then we are in the goal sampling stage
    if (
        constr.joint_constraints.size() == 0 &&
        group_name == "whole_body")
    {
      logInform("humanoid_constraint_sampler: Using custom constraint sampler");
      return true;
    }

    logInform("humanoid_constraint_sampler: NOT using custom constraint sampler");
    return false;
  }

};

} // namespace

PLUGINLIB_EXPORT_CLASS(moveit_humanoid_stability::HumanoidConstraintSamplerAllocator,
                       constraint_samplers::ConstraintSamplerAllocator);


#endif
