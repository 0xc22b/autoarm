#include <autoarm_poses/grasp_pose_data.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>

#include <math.h>
#define _USE_MATH_DEFINES

namespace autoarm_poses
{

  GraspPoseData::GraspPoseData()
    : base_link_("/base_link")
    , grasp_depth_(0.12)
    , angle_resolution_(16)
    , approach_retreat_desired_dist_(0.6)
    , approach_retreat_min_dist_(0.4)
    , object_size_(0.04)
  {
  } 

  bool GraspPoseData::loadData(const ros::NodeHandle& nh)
  {

    std::vector<std::string> joint_names;
    std::vector<double> pre_grasp_posture;
    std::vector<double> grasp_posture;
    std::vector<double> grasp_pose_to_eef;
    std::vector<double> grasp_pose_to_eef_rotation;
    double pregrasp_time_from_start;
    double grasp_time_from_start;
    std::string end_effector_name;
    std::string end_effector_parent_link;

    ROS_ASSERT(nh.hasParam("base_link"));
    nh.getParam("base_link", base_link_);

    ros::NodeHandle child_nh(nh, "gripper");

    ROS_ASSERT(child_nh.hasParam("pregrasp_time_from_start"));
    child_nh.getParam("pregrasp_time_from_start", pregrasp_time_from_start);

    ROS_ASSERT(child_nh.hasParam("grasp_time_from_start"));
    child_nh.getParam("grasp_time_from_start", grasp_time_from_start);

    ROS_ASSERT(child_nh.hasParam("end_effector_name"));
    child_nh.getParam("end_effector_name", end_effector_name);

    ROS_ASSERT(child_nh.hasParam("end_effector_parent_link"));
    child_nh.getParam("end_effector_parent_link", end_effector_parent_link);

    ROS_ASSERT(child_nh.hasParam("joints"));
    XmlRpc::XmlRpcValue joint_list;
    child_nh.getParam("joints", joint_list);
    if (joint_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int32_t i = 0; i < joint_list.size(); ++i) {
        ROS_ASSERT(joint_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        joint_names.push_back(static_cast<std::string>(joint_list[i]));
      }
    } else {
      ROS_ERROR_STREAM_NAMED("temp", "joint list type is not type array???");
    }

    if (child_nh.hasParam("pregrasp_posture")) {
      XmlRpc::XmlRpcValue preg_posture_list;
      child_nh.getParam("pregrasp_posture", preg_posture_list);
      ROS_ASSERT(preg_posture_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (int32_t i = 0; i < preg_posture_list.size(); ++i) {
        ROS_ASSERT(preg_posture_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        pre_grasp_posture.push_back(static_cast<double>(preg_posture_list[i]));
      }
    }

    ROS_ASSERT(child_nh.hasParam("grasp_posture"));
    XmlRpc::XmlRpcValue grasp_posture_list;
    child_nh.getParam("grasp_posture", grasp_posture_list);
    ROS_ASSERT(grasp_posture_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < grasp_posture_list.size(); ++i) {
      ROS_ASSERT(grasp_posture_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      grasp_posture.push_back(static_cast<double>(grasp_posture_list[i]));
    }

    ROS_ASSERT(child_nh.hasParam("grasp_pose_to_eef"));
    XmlRpc::XmlRpcValue g_to_eef_list;
    child_nh.getParam("grasp_pose_to_eef", g_to_eef_list);
    ROS_ASSERT(g_to_eef_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < g_to_eef_list.size(); ++i) {

      if (g_to_eef_list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
        if (g_to_eef_list[i].getType() != XmlRpc::XmlRpcValue::TypeInt ) {
          ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `grasp_pose_to_eef` wrong data type - int or double required.");
          return false;
        } else {
          grasp_pose_to_eef.push_back(static_cast<int>(g_to_eef_list[i]));
        }
      } else {
        grasp_pose_to_eef.push_back(static_cast<double>(g_to_eef_list[i]));
      }
    }

    ROS_ASSERT(child_nh.hasParam("grasp_pose_to_eef_rotation"));
    XmlRpc::XmlRpcValue g_to_eef_rotation_list;
    child_nh.getParam("grasp_pose_to_eef_rotation", g_to_eef_rotation_list);
    ROS_ASSERT(g_to_eef_rotation_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < g_to_eef_rotation_list.size(); ++i) {
      if (g_to_eef_rotation_list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
        if (g_to_eef_rotation_list[i].getType() != XmlRpc::XmlRpcValue::TypeInt ) {
          ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `grasp_pose_to_eef_rotation` wrong data type - int or double required.");
          return false;
        } else {
          grasp_pose_to_eef_rotation.push_back(static_cast<int>(g_to_eef_rotation_list[i]));
        }
      } else {
        grasp_pose_to_eef_rotation.push_back(static_cast<double>(g_to_eef_rotation_list[i]));
      }
    }

    ROS_ASSERT(grasp_pose_to_eef_rotation.size() == 3);
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(grasp_pose_to_eef_rotation[1]), Eigen::Vector3d::UnitY()));
    grasp_pose_to_eef_pose_.orientation.x = quat.x();
    grasp_pose_to_eef_pose_.orientation.y = quat.y();
    grasp_pose_to_eef_pose_.orientation.z = quat.z();
    grasp_pose_to_eef_pose_.orientation.w = quat.w();

    ROS_ASSERT(grasp_pose_to_eef.size() == 3);
    grasp_pose_to_eef_pose_.position.x = grasp_pose_to_eef[0];
    grasp_pose_to_eef_pose_.position.y = grasp_pose_to_eef[1];
    grasp_pose_to_eef_pose_.position.z = grasp_pose_to_eef[2];

    if (!pre_grasp_posture.empty()) {
      pre_grasp_posture_.header.frame_id = base_link_;
      pre_grasp_posture_.header.stamp = ros::Time::now();

      pre_grasp_posture_.joint_names = joint_names;

      pre_grasp_posture_.points.resize(1);
      pre_grasp_posture_.points[0].positions = pre_grasp_posture;
      pre_grasp_posture_.points[0].time_from_start = ros::Duration(pregrasp_time_from_start);
    }

    grasp_posture_.header.frame_id = base_link_;
    grasp_posture_.header.stamp = ros::Time::now();

    grasp_posture_.joint_names = joint_names;

    grasp_posture_.points.resize(1);
    grasp_posture_.points[0].positions = grasp_posture;
    grasp_posture_.points[0].time_from_start = ros::Duration(grasp_time_from_start);

    ee_parent_link_ = end_effector_parent_link;
    ee_group_ = end_effector_name;

    approach_retreat_desired_dist_ = 0.2;
    approach_retreat_min_dist_ = 0.06;

    grasp_depth_ = 0.06;

    angle_resolution_ = 16;

    return true;
  }
}
