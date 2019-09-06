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

#include "voliro_trajectories_node.h"

namespace omav_local_planner {

VoliroTrajectoriesNode::VoliroTrajectoriesNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh),
      dyn_config_server_(private_nh),
      n_points_(0) {
  // Set up dynamic reconfigure parameter server.
  dynamic_reconfigure::Server<
      omav_local_planner::VoliroTrajectoriesConfig>::CallbackType f;
  f = boost::bind(&VoliroTrajectoriesNode::dynConfigCallback, this, _1, _2);
  dyn_config_server_.setCallback(f);
  path_planned_ = false;
  waypoints_filename_ = "";
  order_rpy_ = 0;

  initializeParams();
  initializeSubscribers();
  initializePublishers();

  home_.position_W << 0, 0, 1;

  trajectory_running_ = false;
  trajectory_end_time_ = ros::Time::now();

}

VoliroTrajectoriesNode::~VoliroTrajectoriesNode() {}

// Load dynamic reconfigure parameters.
void VoliroTrajectoriesNode::dynConfigCallback(
    omav_local_planner::VoliroTrajectoriesConfig& config,
    uint32_t level) {

  params_.a_max_ang = config.a_max_ang;
  params_.a_max_trans = config.a_max_trans;
  params_.v_max_ang = config.v_max_ang;
  params_.v_max_trans = config.v_max_trans;

  params_.takeoff_z = config.takeoff_z;
  return;
}

void VoliroTrajectoriesNode::initializeParams() {
  ros::NodeHandle pnh("~");
  pnh.param("waypoints_filename", waypoints_filename_, waypoints_filename_);
  // Rotation order:
  // 0 = rotate yaw - pitch - roll from world to body frame
  // 1 = rotate roll - pitch - yaw from world to body frame
  pnh.param("order_rpy", order_rpy_, order_rpy_);
  parseTextFile(waypoints_filename_);
}

void VoliroTrajectoriesNode::initializeSubscribers() {
  cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &VoliroTrajectoriesNode::multiDofJointTrajectoryCallback, this,
      ros::TransportHints().tcpNoDelay());

  odometry_sub_ = nh_.subscribe(
      mav_msgs::default_topics::ODOMETRY, 1,
      &VoliroTrajectoriesNode::odometryCallback, this,
      ros::TransportHints().tcpNoDelay());
}

bool VoliroTrajectoriesNode::parseTextFile(const std::string &filename) {
  std::ifstream file(filename, std::ios::in);
  if (!file.is_open()) {
    ROS_ERROR("Problem opening file. Tried opening file %s", filename);
    return false;
  }
  trajectory_positions_.clear();
  trajectory_rotations_.clear();
  std::vector<double> vals;
  double val = 0.0;
  while (file >> val) {
    vals.push_back(val);
  }
  n_points_ = vals.size() / 6;
  if (n_points_ > 0) {
    for (int i=0;i<n_points_;i++) {
      trajectory_positions_.push_back(Eigen::Vector3d(vals[6*i], vals[6*i+1], vals[6*i+2]));
      trajectory_rotations_.push_back(Eigen::Vector3d(vals[6*i+3], vals[6*i+4], vals[6*i+5]));
    }
    ROS_INFO("Finished loading %i waypoints.", n_points_);
    rotateRPYToAxisAngle();
    planTrajectory();
  }
  else {
    return false;
  }

  return true;
}

void unwrapAxisAngle(const Eigen::Vector3d &axis1, const Eigen::AngleAxisd &rot2, Eigen::Vector3d *rot2_3d) {
  // Simple geometric method to find closest axis angle representation given a previous one
  double min_dist = 1000000.0;
  Eigen::Vector3d best_angle;
  best_angle = rot2.angle() * rot2.axis();
  for (int i=-10;i<10;i++) {
    Eigen::Vector3d new_angle;
    new_angle = rot2.axis() * (rot2.angle() + i*2.0*M_PI);
    if ( (new_angle-axis1).norm() < min_dist) {
      min_dist = (new_angle-axis1).norm();
      best_angle = new_angle;
    }
  }
  *rot2_3d = best_angle;
}

void quaternionFromRPY(const Eigen::Vector3d &rot, Eigen::Quaterniond *q, const int &order=0) {
    Eigen::Matrix3d m = (Eigen::AngleAxisd(-rot(2), Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(-rot(1),  Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(-rot(0), Eigen::Vector3d::UnitZ())).toRotationMatrix();
    if (order == 0){
      *q = Eigen::Quaterniond(m.transpose());
    }
    else {
      *q = Eigen::Quaterniond(m);
    }
}

void VoliroTrajectoriesNode::rotateRPYToAxisAngle() {
  for (int i=0;i<n_points_;i++) {
    Eigen::Vector3d rot = trajectory_rotations_[i];
    Eigen::AngleAxisd this_rot;
    Eigen::Matrix3d m = (Eigen::AngleAxisd(-rot(2), Eigen::Vector3d::UnitX())
                                * Eigen::AngleAxisd(-rot(1),  Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(-rot(0), Eigen::Vector3d::UnitZ())).toRotationMatrix();
    if (order_rpy_ == 0){
      this_rot.fromRotationMatrix(m.transpose());
    }
    else {
      this_rot.fromRotationMatrix(m);      
    }

    if (i==0) {
      trajectory_rotations_[0] = this_rot.axis() * this_rot.angle();
    }
    else {
      unwrapAxisAngle(trajectory_rotations_[i-1], this_rot, &trajectory_rotations_[i]);
    }
  }
}

void VoliroTrajectoriesNode::initializePublishers() {
  trajectory_reference_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>("current_traj", 1);
  rviz_traj_pub_ =
      nh_.advertise<geometry_msgs::PoseArray>("current_traj_poses", 1);

  cmd_multi_dof_joint_trajectory_pub_ =
  nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);

  plan_trajectory_srv_ = nh_.advertiseService(
      "plan_trajectory", &VoliroTrajectoriesNode::planTrajectorySrv, this);

  homing_srv_ = nh_.advertiseService(
      "homing", &VoliroTrajectoriesNode::homingSrv, this);

  publish_path_srv_ = nh_.advertiseService(
      "publish_path", &VoliroTrajectoriesNode::publishPathSrv, this);

  stop_trajectory_srv_ = nh_.advertiseService(
      "stop_trajectory", &VoliroTrajectoriesNode::stopTrajectorySrv, this);

  take_off_srv_ = nh_.advertiseService(
      "take_off", &VoliroTrajectoriesNode::takeOffSrv, this);

  start_srv_ = nh_.advertiseService(
      "start", &VoliroTrajectoriesNode::startSrv, this);

  land_srv_ = nh_.advertiseService(
      "land", &VoliroTrajectoriesNode::landSrv, this);

  load_file_srv_ = nh_.advertiseService(
      "load_file", &VoliroTrajectoriesNode::loadFileSrv, this);

  go_to_trajectory_srv_ = nh_.advertiseService(
      "go_to_trajectory", &VoliroTrajectoriesNode::goToTrajectorySrv, this);
}

void VoliroTrajectoriesNode::odometryCallback(
    const nav_msgs::OdometryConstPtr& odometry_msg) {
  ROS_INFO_ONCE("VoliroTrajectories got first odometry message.");

  mav_msgs::eigenOdometryFromMsg(*odometry_msg, &current_odometry_);

  odometry_timestamp_ = odometry_msg->header.stamp;
}

void VoliroTrajectoriesNode::multiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  const size_t n_commands = msg->points.size();

  if (n_commands < 1) {
    return;
  }
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.back(), &current_setpoint_);
}


bool VoliroTrajectoriesNode::startSrv(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response) {
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_point.position_W = current_odometry_.position_W;
  trajectory_point.position_W(2) = 0.0;
  trajectory_point.orientation_W_B = current_odometry_.orientation_W_B;
  mav_msgs::msgMultiDofJointTrajectoryFromEigen (trajectory_point, &current_trajectory_msg_);
  cmd_multi_dof_joint_trajectory_pub_.publish(current_trajectory_msg_);
  return true;
}

bool VoliroTrajectoriesNode::takeOffSrv(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response) {
  return takeOff();
}

bool VoliroTrajectoriesNode::planTrajectorySrv(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response) {
  return parseTextFile(waypoints_filename_);
}

bool VoliroTrajectoriesNode::goToTrajectorySrv(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response) {
  return goToTrajectory();
}

bool VoliroTrajectoriesNode::loadFileSrv(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response) {
  return parseTextFile(waypoints_filename_);
}

bool VoliroTrajectoriesNode::homingSrv(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response) {
  return homing();
}

bool VoliroTrajectoriesNode::landSrv(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response) {
  return land();
}

bool VoliroTrajectoriesNode::goToTrajectory() {
  parseTextFile(waypoints_filename_);
  mav_msgs::EigenTrajectoryPoint start, goal;
  start.position_W = current_odometry_.position_W;
  start.orientation_W_B = current_odometry_.orientation_W_B;
  goal.position_W = trajectory_positions_.front();
  // Need hack since the parseFile inverts rotations
  Eigen::Vector3d start_rpy(trajectory_rotations_.front()(2),
                            trajectory_rotations_.front()(1),
                            trajectory_rotations_.front()(0));
  quaternionFromRPY(start_rpy, &goal.orientation_W_B, order_rpy_);
  if(planToWaypoint(start, goal)) {
    publishTrajectory(false);
  }
  parseTextFile(waypoints_filename_);
  return true;
}

bool VoliroTrajectoriesNode::land() {
  mav_msgs::EigenTrajectoryPoint start;
  start.position_W = current_odometry_.position_W;
  start.orientation_W_B = current_odometry_.orientation_W_B;
  mav_msgs::EigenTrajectoryPoint goal = start;
  goal.setFromYaw(start.getYaw());
  goal.position_W(2) = 0.0;
  if (planToWaypoint(start, goal)) {
    publishTrajectory(false);
  }
  return true;
}

bool VoliroTrajectoriesNode::publishPathSrv(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response) {
  parseTextFile(waypoints_filename_);
  if (path_planned_) {
    publishTrajectory(false);
    return true;
  }
  return false;
}

bool VoliroTrajectoriesNode::homing() {
  mav_msgs::EigenTrajectoryPoint start;
  start.position_W = current_odometry_.position_W;
  start.orientation_W_B = current_odometry_.orientation_W_B;
  mav_msgs::EigenTrajectoryPoint goal = home_;
  if(planToWaypoint(start, goal)) {
    publishTrajectory(false);
  }
  return true;
}

bool VoliroTrajectoriesNode::stopTrajectorySrv(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response) {
  return stopTrajectory();
}

bool VoliroTrajectoriesNode::stopTrajectory() {
  ros::Time t_now = ros::Time::now();
  ros::Duration dt = t_now - current_trajectory_msg_.header.stamp;
  // Iterate through points to find current setpoint
  int n = states_.size();
  int i=0;
  int64_t dt_nsec = dt.toNSec();
  while (states_[i].time_from_start_ns <= dt_nsec && i < n-1) {
    i++;
  }
  mav_msgs::msgMultiDofJointTrajectoryFromEigen (states_[i], &current_trajectory_msg_);
  current_trajectory_msg_.header.stamp = t_now;
  cmd_multi_dof_joint_trajectory_pub_.publish(current_trajectory_msg_);
  return true;
}

bool VoliroTrajectoriesNode::planTrajectory() {
  mav_trajectory_generation::Trajectory trajectory_rot;
  mav_trajectory_generation::Trajectory trajectory_lin;

  std::vector<Eigen::Vector3d> velocities;
  for (int i=0;i<n_points_;i++) {
    velocities.push_back(Eigen::Vector3d::Zero());
  }

  bool only_rot = false;
  // Catch zero translation:
  for (int i=0;i<n_points_-1;i++){
    if((trajectory_positions_[i]-trajectory_positions_[i+1]).norm() < 0.01) {
      ROS_WARN("Detected zero movement between points. Taking first positoin waypoint and ignoring translation.");
      only_rot = true;
    }
  }
  if (only_rot) {
    createTrajectory(trajectory_rotations_, velocities,
                      params_.a_max_ang, params_.v_max_ang,
                      &trajectory_rot);
    addConstantPosition(trajectory_positions_[0], trajectory_rot);
  }
  else {
    createTrajectory(trajectory_positions_, velocities,
                      params_.a_max_trans, params_.v_max_trans,
                      &trajectory_lin);
    createTrajectory(trajectory_rotations_, velocities,
                      params_.a_max_ang, params_.v_max_ang,
                      &trajectory_rot);
    trajectory_lin.getTrajectoryWithAppendedDimension(trajectory_rot, &trajectory_);
  }
  path_planned_ = true;
  ROS_INFO("Finished planning trajectory.");
  return true;
}

bool VoliroTrajectoriesNode::planToWaypoint(const mav_msgs::EigenTrajectoryPoint &start,
                                      const mav_msgs::EigenTrajectoryPoint& goal) {

  if ( (start.position_W - goal.position_W).norm() < 0.05 ) {
    ROS_WARN("[trajectories] No position change requested. Aborting.");
    return false;
  }
  // Create trajectory for position change:
  // Set up trajectory:
  std::vector<Eigen::Vector3d> pos, vel;
  pos.push_back(start.position_W);
  pos.push_back(goal.position_W);
  vel.push_back(Eigen::Vector3d::Zero());
  vel.push_back(Eigen::Vector3d::Zero());

  mav_trajectory_generation::Trajectory trajectory_lin;
  createTrajectory(pos, vel, params_.a_max_trans, params_.v_max_trans, &trajectory_lin);

  // Compute angle axis representation of start orientation:
  Eigen::AngleAxisd att0(start.orientation_W_B);
  if (start.orientation_W_B.angularDistance(goal.orientation_W_B) < 0.01) {
    ROS_INFO("[trajectories] No orientation change, planning pure translation.");
    addConstantRotation(att0.axis()*att0.angle(), trajectory_lin);
  }
  else {
    // Create trajectory for orientation change:
    // Set up trajectory:
    std::vector<Eigen::Vector3d> att, ang_vel;
    Eigen::AngleAxisd att1(goal.orientation_W_B);
    att.push_back(att0.axis() * att0.angle());
    att.push_back(att1.axis() * att1.angle());
    ang_vel.push_back(Eigen::Vector3d::Zero());
    ang_vel.push_back(Eigen::Vector3d::Zero());

    mav_trajectory_generation::Trajectory trajectory_rot;
    createTrajectory(att, ang_vel, params_.a_max_ang, params_.v_max_ang, &trajectory_rot);
    trajectory_lin.getTrajectoryWithAppendedDimension(trajectory_rot, &trajectory_);
  }
  path_planned_ = true;
  return true;

}

bool VoliroTrajectoriesNode::takeOff() {
  Eigen::Vector3d takeoff_pos;
  takeoff_pos << 0, 0, params_.takeoff_z;
  std::vector<Eigen::Vector3d> pos, vel;
  Eigen::Vector3d pos0 = current_odometry_.position_W;
  pos0(2) = 0.0;
  pos.push_back(pos0);
  pos.push_back(current_odometry_.position_W + takeoff_pos);
  vel.push_back(Eigen::Vector3d::Zero());
  vel.push_back(Eigen::Vector3d::Zero());
  mav_trajectory_generation::Trajectory trajectory_lin;
  createTrajectory(pos, vel, params_.a_max_trans, params_.v_max_trans, &trajectory_lin);
  Eigen::AngleAxisd att0(current_odometry_.orientation_W_B);
  addConstantRotation(att0.axis()*att0.angle(), trajectory_lin);
  publishTrajectory(false);
  home_.position_W = current_odometry_.position_W + takeoff_pos;
  return true;
}

void VoliroTrajectoriesNode::addConstantPosition(const Eigen::Vector3d &pos,
                            const mav_trajectory_generation::Trajectory &traj_rot) {
  // This function adds a constant position to a rotation trajectory
  // (without running an optimizer since this would crash)
  int N = traj_rot.N();   // order of polynomials
  int K = traj_rot.K();   // number of segments
  mav_trajectory_generation::Trajectory traj_trans;
  mav_trajectory_generation::Segment::Vector segments;
  traj_rot.getSegments(&segments);
  for (int i=0;i<K;i++) {
    for (int j=0;j<3;j++) {
      Eigen::VectorXd coeffs(N);
      coeffs.setZero();
      coeffs(0) = pos(j);
      segments[i][j].setCoefficients(coeffs);
    }
  }
  traj_trans.setSegments(segments);
  traj_trans.getTrajectoryWithAppendedDimension(
          traj_rot, &trajectory_);
}

void VoliroTrajectoriesNode::addConstantRotation(const Eigen::Vector3d &rot,
                            const mav_trajectory_generation::Trajectory &traj_trans) {
  // This function adds a constant orientation to a translation trajectory
  // (without running an optimizer since this would crash)
  int N = traj_trans.N();   // order of polynomials
  int K = traj_trans.K();   // number of segments
  mav_trajectory_generation::Trajectory traj_rot;
  mav_trajectory_generation::Segment::Vector segments;
  traj_trans.getSegments(&segments);
  for (int i=0;i<K;i++) {
    for (int j=0;j<3;j++) {
      Eigen::VectorXd coeffs(N);
      coeffs.setZero();
      coeffs(0) = rot(j);
      segments[i][j].setCoefficients(coeffs);
    }
  }
  traj_rot.setSegments(segments);
  traj_trans.getTrajectoryWithAppendedDimension(
          traj_rot, &trajectory_);
}

void VoliroTrajectoriesNode::publishTrajectory(const bool &rotate_trajectory) {
  const double sampling_interval = 0.01;
  mav_trajectory_generation::sampleWholeTrajectory(trajectory_,
                           sampling_interval, &states_);

  // Rotate states to initial rotation:
  if (rotate_trajectory){
    for (auto & s: states_) {
      s.orientation_W_B = current_setpoint_.orientation_W_B * s.orientation_W_B;
    }
  }
  mav_msgs::msgMultiDofJointTrajectoryFromEigen (states_, &current_trajectory_msg_);
  current_trajectory_msg_.header.stamp = ros::Time::now();
  cmd_multi_dof_joint_trajectory_pub_.publish(current_trajectory_msg_);

  geometry_msgs::PoseArray pose_array;
  poseArrayFromEigen(states_, &pose_array);
  publishRvizPoseArray(pose_array);

  double trajectory_duration = trajectory_.getMaxTime();
  ROS_INFO("Starting trajectory, duration = %.02f s", trajectory_duration);
  trajectory_end_time_ = ros::Time::now() + ros::Duration(trajectory_duration);
}

void VoliroTrajectoriesNode::poseArrayFromEigen(
                    const mav_msgs::EigenTrajectoryPoint::Vector &states,
                          geometry_msgs::PoseArray *pose_array) const {
  pose_array->header.stamp = ros::Time::now();
  pose_array->header.frame_id = "world";
  for(auto const& state : states) {
  // for (int i=0;i<states.size();i++) {
    geometry_msgs::Pose pose;
    tf::pointEigenToMsg(state.position_W, pose.position);
    tf::quaternionEigenToMsg(state.orientation_W_B, pose.orientation);
    pose_array->poses.push_back(pose);
  }
}

void VoliroTrajectoriesNode::createTrajectory(const std::vector<Eigen::Vector3d> &positions,
                                              const std::vector<Eigen::Vector3d> &velocities,
                                              const double &a_max, const double &v_max,
                                              mav_trajectory_generation::Trajectory* trajectory) {

  const int dimension = 3;
  int n_points = positions.size();
  if (n_points != velocities.size()) {
    ROS_WARN("Positions and velocities have different number of points.");
    return;
  }
  if (n_points < 2) {
    ROS_WARN("Not enough waypoints.");
    return;
  }

  // Array for all waypoints and their constraints
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  // start = desired start vector
  // end = desired end vector
  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  /******* Configure start point *******/
  start.makeStartOrEnd(positions.front(), derivative_to_optimize);
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      velocities.front());
  vertices.push_back(start);

  for (int i=1;i<n_points-1;i++) {
    mav_trajectory_generation::Vertex v(dimension);
    // Only add position constraint (velocities should be optimized automatically)
    v.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                    positions[i]);
    vertices.push_back(v);
  }

  /******* Configure end point *******/
  // set end point constraints to desired position and set all derivatives to zero
  end.makeStartOrEnd(positions.back(), derivative_to_optimize);
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                    velocities.back());
  vertices.push_back(end);

  // setimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, v_max, a_max);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

  // solve trajectory
  // opt.optimize();
  opt.solveLinear();

  // get trajectory as polynomial parameters
  opt.getTrajectory(trajectory);
  trajectory->scaleSegmentTimesToMeetConstraints(v_max, a_max);
}

void VoliroTrajectoriesNode::publishRvizPoseArray(const geometry_msgs::PoseArray &pose_array) const {
  rviz_traj_pub_.publish(pose_array);
}


}  // namespace omav_local_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "voliro_trajectories_node");

  ros::NodeHandle nh, private_nh("~");

  std::shared_ptr<omav_local_planner::VoliroTrajectoriesNode>
      voliro_controller_node(
          new omav_local_planner::VoliroTrajectoriesNode(nh, private_nh));

  ros::spin();

  return 0;
}
