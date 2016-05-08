#include <autoarm_poses/grasp_pose_generator.h>

namespace autoarm_poses
{

  GraspPoseGenerator::GraspPoseGenerator(bool verbose)
    :verbose_(verbose)
  {
  }

  GraspPoseGenerator::~GraspPoseGenerator()
  {
  }

  bool GraspPoseGenerator::generateBlockGraspPoses(const geometry_msgs::Pose& object_pose,
                                                   const GraspPoseData& grasp_data,
                                                   std::vector<moveit_msgs::Grasp>& possible_grasps)
  {

    generateAxisGraspPoses(object_pose,
                           X_AXIS,
                           DOWN,
                           HALF,
                           0,
                           grasp_data,
                           possible_grasps);

    generateAxisGraspPoses(object_pose,
                           X_AXIS,
                           UP,
                           HALF,
                           0,
                           grasp_data,
                           possible_grasps);

    generateAxisGraspPoses(object_pose,
                           Y_AXIS,
                           DOWN,
                           HALF,
                           0,
                           grasp_data,
                           possible_grasps);

    generateAxisGraspPoses(object_pose,
                           Y_AXIS,
                           UP,
                           HALF,
                           0,
                           grasp_data,
                           possible_grasps);

    return true;
  }

  bool GraspPoseGenerator::generateAxisGraspPoses(const geometry_msgs::Pose& object_pose,
                                                  grasp_axis_t axis,
                                                  grasp_direction_t direction,
                                                  grasp_rotation_t rotation,
                                                  double hand_roll,
                                                  const GraspPoseData& grasp_data,
                                                  std::vector<moveit_msgs::Grasp>& possible_grasps)
  {

    tf::poseMsgToEigen(object_pose, object_global_transform_);

    moveit_msgs::GripperTranslation pre_grasp_approach;
    pre_grasp_approach.direction.header.stamp = ros::Time::now();
    pre_grasp_approach.desired_distance = grasp_data.approach_retreat_desired_dist_;
    pre_grasp_approach.min_distance = grasp_data.approach_retreat_min_dist_;

    moveit_msgs::GripperTranslation post_grasp_retreat;
    post_grasp_retreat.direction.header.stamp = ros::Time::now();
    post_grasp_retreat.desired_distance = grasp_data.approach_retreat_desired_dist_;
    post_grasp_retreat.min_distance = grasp_data.approach_retreat_min_dist_;

    geometry_msgs::PoseStamped grasp_pose_msg;
    grasp_pose_msg.header.stamp = ros::Time::now();
    grasp_pose_msg.header.frame_id = grasp_data.base_link_;

    double radius = grasp_data.grasp_depth_;
    double xb;
    double yb = 0.0;
    double zb;
    double theta1 = 0.0;
    double theta2 = 0.0;

    if( direction == DOWN ) {
      theta2 = M_PI;
    }

    for(int i = 0; i <= grasp_data.angle_resolution_; ++i) {

      moveit_msgs::Grasp new_grasp;

      xb = radius*cos(theta1);
      zb = radius*sin(theta1);

      Eigen::Affine3d grasp_pose;

      switch(axis) {
      case X_AXIS:
        grasp_pose = Eigen::AngleAxisd(theta1, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX());
        grasp_pose.translation() = Eigen::Vector3d( yb, xb ,zb);
        break;

      case Y_AXIS:
        grasp_pose = Eigen::AngleAxisd(M_PI - theta1, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX());
        grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);
        break;

      case Z_AXIS:
        grasp_pose = Eigen::AngleAxisd(M_PI - theta1, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX());
        grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);
        break;
      }

      double score = sin(theta1);
      new_grasp.grasp_quality = std::max(score,0.1);

      if (rotation == HALF) {
        theta1 += M_PI / grasp_data.angle_resolution_;
      } else {
        theta1 += 2*M_PI / grasp_data.angle_resolution_;
      }

      static int grasp_id = 0;
      new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
      ++grasp_id;

      new_grasp.pre_grasp_posture = grasp_data.pre_grasp_posture_;
      new_grasp.grasp_posture = grasp_data.grasp_posture_;

      if (verbose_) {
        tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);
      }

      Eigen::Affine3d roll_gripper;
      roll_gripper = Eigen::AngleAxisd(hand_roll, Eigen::Vector3d::UnitX());
      grasp_pose = grasp_pose * roll_gripper;

      Eigen::Affine3d eef_conversion_pose;
      tf::poseMsgToEigen(grasp_data.grasp_pose_to_eef_pose_, eef_conversion_pose);

      grasp_pose = grasp_pose * eef_conversion_pose;

      tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);

      new_grasp.grasp_pose = grasp_pose_msg;

      new_grasp.max_contact_force = 0;

      pre_grasp_approach.direction.header.frame_id = grasp_data.base_link_;
      pre_grasp_approach.direction.vector.x = 0;
      pre_grasp_approach.direction.vector.y = 0;
      pre_grasp_approach.direction.vector.z = -1;
      new_grasp.pre_grasp_approach = pre_grasp_approach;

      post_grasp_retreat.direction.header.frame_id = grasp_data.base_link_;
      post_grasp_retreat.direction.vector.x = 0;
      post_grasp_retreat.direction.vector.y = 0;
      post_grasp_retreat.direction.vector.z = 1;
      new_grasp.post_grasp_retreat = post_grasp_retreat;

      possible_grasps.push_back(new_grasp);

      pre_grasp_approach.direction.header.frame_id = grasp_data.ee_parent_link_;
      pre_grasp_approach.direction.vector.x = 0;
      pre_grasp_approach.direction.vector.y = 0;
      pre_grasp_approach.direction.vector.z = 1;
      new_grasp.pre_grasp_approach = pre_grasp_approach;

      post_grasp_retreat.direction.header.frame_id = grasp_data.ee_parent_link_;
      post_grasp_retreat.direction.vector.x = 0;
      post_grasp_retreat.direction.vector.y = 0;
      post_grasp_retreat.direction.vector.z = -1;
      new_grasp.post_grasp_retreat = post_grasp_retreat;

      possible_grasps.push_back(new_grasp);
    }

    ROS_INFO_STREAM_NAMED("grasp", "Generated " << possible_grasps.size() << " grasps." );
    return true;
  }
}
