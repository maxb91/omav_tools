/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2017 Karen Bodie, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef OMAV_LOCAL_PLANNER_VOLIRO_TRAJECTORIES_NODE_H
#define OMAV_LOCAL_PLANNER_VOLIRO_TRAJECTORIES_NODE_H

#include <fstream>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <boost/bind.hpp>
#include <stdio.h>

// ros
#include <eigen_conversions/eigen_msg.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
// #include <tf_conversions/tf_eigen.h>

// msgs
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseArray.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

// dynamic reconfiguration
#include <dynamic_reconfigure/server.h>
#include <omav_local_planner/VoliroTrajectoriesConfig.h>

// Trajectory generation
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

namespace omav_local_planner {

struct VoliroTrajectoriesParameters {
  VoliroTrajectoriesParameters() :
  a_max_ang(4.0),
  a_max_trans(1.0),
  v_max_ang(4.0),
  v_max_trans(1.0),
  takeoff_z(1.0)
      {
  }
  double a_max_ang;
  double a_max_trans;
  double v_max_ang;
  double v_max_trans;
  double takeoff_z;
};



class VoliroTrajectoriesNode {
public:
  VoliroTrajectoriesNode(const ros::NodeHandle &nh,
                       const ros::NodeHandle &private_nh);
  ~VoliroTrajectoriesNode();

private:
  void initializeParams();
  void initializeSubscribers();
  void initializePublishers();

  bool parseTextFile(const std::string &filename);

  void dynConfigCallback(omav_local_planner::VoliroTrajectoriesConfig &config,
                         uint32_t level);

  void odometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg);

  void poseArrayFromEigen(const mav_msgs::EigenTrajectoryPoint::Vector &states,
                          geometry_msgs::PoseArray *pose_array) const;
  void publishRvizPoseArray(const geometry_msgs::PoseArray &pose_array) const;

  bool takeOff();

  bool homing();

  bool planTrajectory();

  bool stopTrajectory();

  void rotateRPYToAxisAngle();

  bool takeOffSrv(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response);

  bool planTrajectorySrv(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response);

  bool publishPathSrv(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response);

  bool loadFileSrv(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response);

  bool stopTrajectorySrv(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response);

  bool homingSrv(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response);

  void publishTrajectory(const bool &rotate_trajectory);

  bool planToWaypoint(const mav_msgs::EigenTrajectoryPoint &start,
                                      const mav_msgs::EigenTrajectoryPoint& goal);

  void createTrajectory(const std::vector<Eigen::Vector3d> &positions,
                                              const std::vector<Eigen::Vector3d> &velocities,
                                              const double &a_max, const double &v_max,
                                              mav_trajectory_generation::Trajectory* trajectory);
  void addConstantPosition(const Eigen::Vector3d &pos,
                            const mav_trajectory_generation::Trajectory &traj_rot);
  void addConstantRotation(const Eigen::Vector3d &rot,
                            const mav_trajectory_generation::Trajectory &traj_lin);

  void multiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);

  ros::NodeHandle nh_;

  std::string waypoints_filename_;

  std::vector<Eigen::Vector3d> trajectory_positions_;
  std::vector<Eigen::Vector3d> trajectory_rotations_;
  int n_points_;

  // subscribers
  ros::Subscriber odometry_sub_;
  ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;

  // publishers
  ros::Publisher cmd_multi_dof_joint_trajectory_pub_;
  ros::Publisher trajectory_reference_pub_;
  ros::Publisher wrench_msg_pub_;
  ros::Publisher tiltrotor_actuator_pub_;
  ros::Publisher rviz_traj_pub_;

  // Services:
  ros::ServiceServer take_off_srv_, load_file_srv_;
  ros::ServiceServer plan_trajectory_srv_, publish_path_srv_;
  ros::ServiceServer stop_trajectory_srv_, homing_srv_;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<omav_local_planner::VoliroTrajectoriesConfig>
      dyn_config_server_;

  // Odometry:
  std::string odometry_topic_;
  std::string world_frame_id_;

  ros::Time odometry_timestamp_;
  mav_msgs::EigenOdometry current_odometry_;

  // Params:
  VoliroTrajectoriesParameters params_;

  bool trajectory_running_;
  bool path_planned_;

  // Trajectory info:
  mav_trajectory_generation::Trajectory trajectory_;
  ros::Time trajectory_end_time_;

  // Current setpoint info:
  mav_msgs::EigenTrajectoryPoint current_setpoint_;
  trajectory_msgs::MultiDOFJointTrajectory current_trajectory_msg_;
  mav_msgs::EigenTrajectoryPoint::Vector states_;
  mav_msgs::EigenTrajectoryPoint home_;
};
} // namespace omav_local_planner

#endif // OMAV_LOCAL_PLANNER_VOLIRO_TRAJECTORIES_NODE_H
