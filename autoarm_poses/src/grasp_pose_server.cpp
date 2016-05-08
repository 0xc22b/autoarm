#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseArray.h>

#include <autoarm_poses/GenerateGraspPosesAction.h>
#include <autoarm_poses/GraspPoseOptions.h>

#include <autoarm_poses/grasp_pose_generator.h>
#include <autoarm_poses/grasp_pose_data.h>

namespace autoarm_poses
{

  bool extractGraspPoseOptions(const autoarm_poses::GraspPoseOptions &options,
                               grasp_axis_t &axis,
                               grasp_direction_t &direction,
                               grasp_rotation_t &rotation)
  {
    switch (options.grasp_axis) {
    case GraspPoseOptions::GRASP_AXIS_X:
      axis = X_AXIS;
      break;
    case GraspPoseOptions::GRASP_AXIS_Y:
      axis = Y_AXIS;
      break;
    case GraspPoseOptions::GRASP_AXIS_Z:
      axis = Z_AXIS;
      break;
    default:
      assert(false);
      break;
    }

    switch (options.grasp_direction) {
    case GraspPoseOptions::GRASP_DIRECTION_UP:
      direction = UP;
      break;
    case GraspPoseOptions::GRASP_DIRECTION_DOWN:
      direction = DOWN;
      break;
    default:
      assert(false);
      break;
    }

    switch (options.grasp_rotation) {
    case GraspPoseOptions::GRASP_ROTATION_FULL:
      rotation = FULL;
      break;
    case GraspPoseOptions::GRASP_ROTATION_HALF:
      rotation = HALF;
      break;
    default:
      assert(false);
      break;
    }

    return true;
  }

  class GraspPoseServer
  {
  private:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<autoarm_poses::GenerateGraspPosesAction> server_;

    autoarm_poses::GraspPoseGeneratorPtr generator_;
    autoarm_poses::GraspPoseData data_;
    autoarm_poses::GenerateGraspPosesResult result_;

  public:

    GraspPoseServer()
      : nh_("~")
      , server_(nh_,
                "generate_grasp_poses",
                boost::bind(&autoarm_poses::GraspPoseServer::execute, this, _1),
                false)
    {

      if (!data_.loadData(nh_)) {
        ros::shutdown();
      }

      generator_.reset(new autoarm_poses::GraspPoseGenerator);
      server_.start();
    }

    void execute(const autoarm_poses::GenerateGraspPosesGoalConstPtr& goal)
    {

      result_.grasps.clear();

      data_.object_size_ = goal->width;

      grasp_axis_t axis;
      grasp_direction_t direction;
      grasp_rotation_t rotation;

      if (goal->options.empty()) {
        generator_->generateBlockGraspPoses(goal->pose,
                                            data_,
                                            result_.grasps);
      } else {

        for (size_t i = 0; i < goal->options.size(); ++i) {

          extractGraspPoseOptions(goal->options[i],
                                  axis,
                                  direction,
                                  rotation);

          generator_->generateAxisGraspPoses(goal->pose,
                                             axis,
                                             direction,
                                             rotation,
                                             0,
                                             data_,
                                             result_.grasps);
        }
      }

      server_.setSucceeded(result_);
    }
  };
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "grasp_pose_server");

  autoarm_poses::GraspPoseServer grasp_pose_server;

  ros::spin();
  return 0;
}
