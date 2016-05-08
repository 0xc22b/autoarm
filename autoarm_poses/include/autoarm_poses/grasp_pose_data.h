#ifndef AUTOARM_POSES__GRASP_POSE_DATA_H_
#define AUTOARM_POSES__GRASP_POSE_DATA_H_

#include <ros/node_handle.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace autoarm_poses
{

  class GraspPoseData
  {
  public:

    geometry_msgs::Pose grasp_pose_to_eef_pose_;

    trajectory_msgs::JointTrajectory pre_grasp_posture_;
    trajectory_msgs::JointTrajectory grasp_posture_;

    std::string base_link_;
    std::string ee_parent_link_;
    std::string ee_group_;
    double grasp_depth_;
    int angle_resolution_;
    double approach_retreat_desired_dist_;
    double approach_retreat_min_dist_;
    double object_size_;

  public:

    GraspPoseData();

    bool loadData(const ros::NodeHandle& nh);

  };
}

#endif
