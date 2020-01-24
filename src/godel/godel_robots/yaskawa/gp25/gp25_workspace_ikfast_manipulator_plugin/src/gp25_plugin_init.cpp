
// register IKFastKinematicsPlugin as a KinematicsBase implementation
#include <gp25_workspace_ikfast_manipulator_plugin/gp25_manipulator_ikfast_moveit_plugin.hpp>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ikfast_kinematics_plugin::IKFastKinematicsPlugin,
                       kinematics::KinematicsBase);
