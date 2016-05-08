#ifndef AUTOARM_POSES__GRASP_POSES_GENERATOR_H_
#define AUTOARM_POSES__GRASP_POSES_GENERATOR_H_

#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/PoseArray.h>

#include <moveit_msgs/Grasp.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>

#include <math.h>
#define _USE_MATH_DEFINES

#include <autoarm_poses/grasp_pose_data.h>

namespace autoarm_poses
{

  static const double RAD2DEG = 57.2957795;

  enum grasp_axis_t {X_AXIS, Y_AXIS, Z_AXIS};
  enum grasp_direction_t {UP, DOWN};
  enum grasp_rotation_t {FULL, HALF};

  class GraspPoseGenerator
  {

  private:

    Eigen::Affine3d object_global_transform_;
    bool verbose_;

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GraspPoseGenerator(bool verbose = false);
    ~GraspPoseGenerator();

    bool generateBlockGraspPoses(const geometry_msgs::Pose& object_pose,
                                 const GraspPoseData& grasp_data,
                                 std::vector<moveit_msgs::Grasp>& possible_grasps);

    bool generateAxisGraspPoses(const geometry_msgs::Pose& object_pose,
                                grasp_axis_t axis,
                                grasp_direction_t direction,
                                grasp_rotation_t rotation,
                                double hand_roll,
                                const GraspPoseData& grasp_data,
                                std::vector<moveit_msgs::Grasp>& possible_grasps);
  };

  typedef boost::shared_ptr<GraspPoseGenerator> GraspPoseGeneratorPtr;
  typedef boost::shared_ptr<const GraspPoseGenerator> GraspPoseGeneratorConstPtr;
}

#endif
