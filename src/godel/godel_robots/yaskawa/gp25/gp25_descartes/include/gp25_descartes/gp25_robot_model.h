/*
  Copyright Feb, 2015 Southwest Research Institute

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

          http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#ifndef GP25_ROBOT_MODEL_H
#define GP25_ROBOT_MODEL_H

#include <descartes_moveit/moveit_state_adapter.h>
#include <gp25_workspace_ikfast_manipulator_plugin/gp25_manipulator_ikfast_moveit_plugin.hpp>

namespace gp25_descartes
{
class GP25RobotModel : public descartes_moveit::MoveitStateAdapter,
                             public ikfast_kinematics_plugin::IKFastKinematicsPlugin
{
public:
  GP25RobotModel();

  virtual bool initialize(const std::string& robot_description, const std::string& group_name,
                          const std::string& world_frame, const std::string& tcp_frame);

  virtual bool getAllIK(const Eigen::Affine3d& pose,
                        std::vector<std::vector<double> >& joint_poses) const;

  virtual descartes_core::RobotModelPtr clone() const
  {
    descartes_core::RobotModelPtr ptr(new GP25RobotModel());
    ptr->initialize("robot_description", descartes_moveit::MoveitStateAdapter::group_name_,
                    world_frame_, tool_frame_);
    return ptr;
  }

protected:
  descartes_core::Frame world_to_base_; // world to arm base
  descartes_core::Frame tool_to_tip_;   // from urdf tool to arm tool
};
}

#endif // MOTOMAN_SIA20D_ROBOT_MODEL
