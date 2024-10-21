#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "regulated_pure_pursuit/regulated_pure_pursuit.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Poses
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double euclidean distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::Pose & pos1,
  const geometry_msgs::msg::Pose & pos2,
  const bool is_3d = false)
{
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;

  if (is_3d) {
    double dz = pos1.position.z - pos2.position.z;
    return std::hypot(dx, dy, dz);
  }

  return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::PoseStamped
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::msg::PoseStamped & pos1,
  const geometry_msgs::msg::PoseStamped & pos2,
  const bool is_3d = false)
{
  return euclidean_distance(pos1.pose, pos2.pose, is_3d);
}

/**
 * Find element in iterator with the minimum calculated value
 */
template<typename Iter, typename Getter>
inline Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp <= lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

/**
 * Find first element in iterator that is greater integrated distance than comparevalue
 */
template<typename Iter, typename Getter>
inline Iter first_after_integrated_distance(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  Getter dist = 0.0;
  for (Iter it = begin; it != end - 1; it++) {
    dist += euclidean_distance(*it, *(it + 1));
    if (dist > getCompareVal) {
      return it + 1;
    }
  }
  return end;
}

/**
 * @brief Calculate the length of the provided path, starting at the provided index
 * @param path Path containing the poses that are planned
 * @param start_index Optional argument specifying the starting index for
 * the calculation of path length. Provide this if you want to calculate length of a
 * subset of the path.
 * @return double Path length
 */
inline double calculate_path_length(const nav_msgs::msg::Path & path, size_t start_index = 0)
{
  if (start_index + 1 >= path.poses.size()) {
    return 0.0;
  }
  double path_length = 0.0;
  for (size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
    path_length += euclidean_distance(path.poses[idx].pose, path.poses[idx + 1].pose);
  }
  return path_length;
}

RegulatedPurePursuitController::RegulatedPurePursuitController() : Node("RPP_Controller"){
  configure();
}

RegulatedPurePursuitController::~RegulatedPurePursuitController()
  {
    delete goal_checker_;
  }

// 2D 변환 함수 (회전 및 평행 이동)
void transformPose2D(
  const geometry_msgs::msg::PoseStamped &in_pose,
  geometry_msgs::msg::PoseStamped &out_pose,
  const geometry_msgs::msg::TransformStamped &transform)
{
  // 변환 행렬에서 평행 이동 및 회전 각도를 추출
  double tx = transform.transform.translation.x;
  double ty = transform.transform.translation.y;
  double theta = tf2::getYaw(transform.transform.rotation); // 2D 회전 각도 (yaw)

  // 입력 포즈의 좌표를 변환 (회전 + 평행 이동)
  double x = in_pose.pose.position.x;
   double y = in_pose.pose.position.y;

  // 회전 및 평행 이동 적용
  out_pose.pose.position.x = tx + (x * std::cos(theta) - y * std::sin(theta));
  out_pose.pose.position.y = ty + (x * std::sin(theta) + y * std::cos(theta));

  // 회전 변환도 적용 (쿼터니언 사용)
  double in_yaw = tf2::getYaw(in_pose.pose.orientation);
  double out_yaw = in_yaw + theta;

  tf2::Quaternion q;
  q.setRPY(0, 0, out_yaw); // Z축 회전 (yaw)만 적용
  out_pose.pose.orientation = tf2::toMsg(q);

  // 헤더 정보 유지
  out_pose.header.stamp = transform.header.stamp;
  out_pose.header.frame_id = transform.header.frame_id;
}

void RegulatedPurePursuitController::configure()
{
  readPathFromYAML();
  tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_->setUsingDedicatedThread(true);
  
  clock_ = get_clock();

  double transform_tolerance = 0.1;
  goal_dist_tol_ = 0.25;  // reasonable default before first update

  declare_parameter("desired_linear_vel", 0.2);
  declare_parameter("min_lookahead_dist", 0.3);
  declare_parameter("max_lookahead_dist", 0.9);
  declare_parameter("lookahead_time", 4.0);
  declare_parameter("rotate_to_heading_angular_vel", 0.5);
  declare_parameter("transform_tolerance", 0.1);
  declare_parameter("use_velocity_scaled_lookahead_dist", true);
  declare_parameter("approach_velocity_scaling_dist", 0.6);
  declare_parameter("use_regulated_linear_velocity_scaling", true);
  declare_parameter("regulated_linear_scaling_min_radius", 0.9);
  // applyConstraints func => min speed
  declare_parameter("regulated_linear_scaling_min_speed", 0.25);
  declare_parameter("use_rotate_to_heading", true);
  declare_parameter("rotate_to_heading_min_angle", 0.785);
  declare_parameter("max_angular_accel", 3.2);
  declare_parameter("max_robot_pose_search_dist", -1.0);
  declare_parameter("use_interpolation", true);

  get_parameter("desired_linear_vel", desired_linear_vel_);
  base_desired_linear_vel_ = desired_linear_vel_;
  get_parameter("min_lookahead_dist", min_lookahead_dist_);
  get_parameter("max_lookahead_dist", max_lookahead_dist_);
  get_parameter("lookahead_time", lookahead_time_);
  get_parameter("rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);
  get_parameter("transform_tolerance", transform_tolerance);
  get_parameter("use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
  get_parameter("approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  get_parameter("use_regulated_linear_velocity_scaling", use_regulated_linear_velocity_scaling_);
  get_parameter("regulated_linear_scaling_min_radius", regulated_linear_scaling_min_radius_);
  get_parameter("regulated_linear_scaling_min_speed", regulated_linear_scaling_min_speed_);
  get_parameter("use_rotate_to_heading", use_rotate_to_heading_);
  get_parameter("rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
  get_parameter("max_angular_accel", max_angular_accel_);
  get_parameter("max_robot_pose_search_dist", max_robot_pose_search_dist_);
  get_parameter("use_interpolation", use_interpolation_);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

  goal_checker_ = new SimpleGoalChecker();
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/Odometry", 10, std::bind(&RegulatedPurePursuitController::odom_cb, this, _1));
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_smoothed", 1, std::bind(&RegulatedPurePursuitController::twist_cb, this, _1));
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, std::bind(&RegulatedPurePursuitController::tf_cb, this, std::placeholders::_1));
  // carrot_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  // carrot_arc_pub_ = this->create_publisher<nav_msgs::msg::Path>("lookahead_collision_arc", 1);
  RCLCPP_INFO(this->get_logger(), "End config");
}

void RegulatedPurePursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  // carrot_pub_.reset();
  // carrot_arc_pub_.reset();
}

double RegulatedPurePursuitController::getLookAheadDistance(
  const geometry_msgs::msg::Twist & speed)
{
  double lookahead_dist;
  lookahead_dist = fabs(speed.linear.x) * lookahead_time_;
  lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);

  return lookahead_dist;
}

geometry_msgs::msg::TwistStamped RegulatedPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  SimpleGoalChecker * goal_checker)
{
  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    // goal_dist_tol_은 shouldRotateToGoalHeading에서 사용
    goal_dist_tol_ = pose_tolerance.position.x;
  }
  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);
  
  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);
  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  RCLCPP_INFO(this->get_logger(), "carrot pose: x: %f, y: %f", carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  double linear_vel, angular_vel;

  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

  // Find curvature of circle (k = 1 / R)
  double curvature = 0.0;
  if (carrot_dist2 > 0.001) {
    curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
  }

  // Setting the velocity direction
  double sign = 1.0;

  linear_vel = desired_linear_vel_;

  // Make sure we're in compliance with basic constraints
  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal);
  } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
    rotateToHeading(linear_vel, angular_vel, angle_to_heading);
  } else {
    applyConstraints(
      curvature, speed,
      transformed_plan,
      linear_vel, sign);

    // Apply curvature to angular velocity after constraining linear velocity
    angular_vel = linear_vel * curvature;
  }

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
}

void RegulatedPurePursuitController::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_.header = msg->header;
  current_pose_.pose = msg->pose.pose;

  geometry_msgs::msg::TwistStamped cmd_vel_stamped;
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel_stamped = computeVelocityCommands(current_pose_, current_twist_, goal_checker_);
  
  cmd_vel.linear.x = cmd_vel_stamped.twist.linear.x;
  cmd_vel.angular.z = cmd_vel_stamped.twist.angular.z;
  
  RCLCPP_INFO(this->get_logger(), "cmd_vel: x: %.2f, y: %.2f", cmd_vel.linear.x, cmd_vel.angular.z);
  vel_pub_->publish(std::move(cmd_vel));
}

void RegulatedPurePursuitController::twist_cb(const geometry_msgs::msg::Twist msg)
{
  current_twist_ = msg;
}

void RegulatedPurePursuitController::tf_cb(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  for (const auto & transform : msg->transforms) {
    if (transform.header.frame_id == "map" && transform.child_frame_id == "odom") {
      map2odom_tf_ = transform;
      getOdomToMapTransform(odom2map_tf_);
      break;
    }
  }
}

bool RegulatedPurePursuitController::getOdomToMapTransform(geometry_msgs::msg::TransformStamped & odom_to_map)
{
  // Check if the map2odom_tf_ has been received
  if (map2odom_tf_.header.frame_id.empty() || map2odom_tf_.child_frame_id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "map2odom_tf_ is not initialized.");
    return false;
  }

  try {
    // Convert the geometry_msgs Transform to a tf2 Transform
    tf2::Transform map_to_odom;
    tf2::fromMsg(map2odom_tf_.transform, map_to_odom);

    // Invert the transform to get odom to map
    tf2::Transform odom_to_map_tf = map_to_odom.inverse();

    // Convert the inverted tf2 Transform back to geometry_msgs Transform
    odom_to_map.transform = tf2::toMsg(odom_to_map_tf);

    // Set the header information appropriately
    odom_to_map.header.stamp = map2odom_tf_.header.stamp;
    odom_to_map.header.frame_id = map2odom_tf_.child_frame_id; // "odom"
    odom_to_map.child_frame_id = map2odom_tf_.header.frame_id; // "map"

    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to invert map2odom transform: %s", ex.what());
    return false;
  }
}

void RegulatedPurePursuitController::readPathFromYAML()
{
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("regulated_pure_pursuit");
  std::string yaml_file_path = package_share_directory + "/config/path.yaml";

  YAML::Node yaml_file = YAML::LoadFile(yaml_file_path);
  auto path = yaml_file["path"];
  if (!path) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load path from YAML file: %s", yaml_file_path.c_str());
    return;
  }

  for (const auto & entry : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = entry["point"]["x"].as<double>();
    pose.pose.position.y = entry["point"]["y"].as<double>();
    pose.pose.position.z = entry["point"]["z"].as<double>();
    pose.pose.orientation.w = 1.0;

    double linear_x = entry["velocity"]["linear_x"].as<double>();
    global_plan_.poses.push_back(pose);
    velocities_.push_back(linear_x);
  }

  global_plan_.header.frame_id = "map";
  RCLCPP_INFO(this->get_logger(), "Path loaded with %zu points from %s", global_plan_.poses.size(), yaml_file_path.c_str());
}

bool RegulatedPurePursuitController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  return use_rotate_to_heading_ && fabs(angle_to_path) > rotate_to_heading_min_angle_;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  // Whether we should rotate robot to goal heading
  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  return use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_;
}

void RegulatedPurePursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * rotate_to_heading_angular_vel_;
}

geometry_msgs::msg::Point RegulatedPurePursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      RCLCPP_INFO(this->get_logger(), "transformPose2D size : %ld", transformed_plan.poses.size());
      RCLCPP_INFO(this->get_logger(), "hypot: %f", hypot(ps.pose.position.x, ps.pose.position.y));
      RCLCPP_INFO(this->get_logger(), "lookdist: %f", lookahead_dist);
      return ps.pose.position.x > 0.0 && hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });
  
  RCLCPP_INFO(this->get_logger(), "goal_pose x: %f, y: %f", goal_pose_it->pose.position.x, goal_pose_it->pose.position.y);
  
  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position,
      goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;

    return pose;
  }

  return *goal_pose_it;
}

double RegulatedPurePursuitController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path
) const
{
  // Waiting to apply the threshold based on integrated distance ensures we don't
  // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
  double remaining_distance = calculate_path_length(transformed_path);
  if (remaining_distance < approach_velocity_scaling_dist_) {
    auto & last = transformed_path.poses.back();
    // Here we will use a regular euclidean distance from the robot frame (origin)
    // to get smooth scaling, regardless of path density.
    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    return distance_to_last_pose / approach_velocity_scaling_dist_;
  } else {
    return 1.0;
  }
}

void RegulatedPurePursuitController::applyApproachVelocityScaling(
  const nav_msgs::msg::Path & path,
  double & linear_vel
) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel *= velocity_scaling;
  }

  // Use the lowest velocity between approach and other constraints, if all overlapping
  linear_vel = std::min(linear_vel, approach_vel);
}

void RegulatedPurePursuitController::applyConstraints(
  const double & curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
  const nav_msgs::msg::Path & path, double & linear_vel, double & sign)
{
  double curvature_vel = linear_vel;

  // limit the linear velocity by curvature
  const double radius = fabs(1.0 / curvature);
  const double & min_rad = regulated_linear_scaling_min_radius_;
  if (use_regulated_linear_velocity_scaling_ && radius < min_rad) {
    curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
  }

  // Use the lowest of the 2 constraint heuristics, but above the minimum translational speed
  linear_vel = curvature_vel;
  linear_vel = std::max(linear_vel, regulated_linear_scaling_min_speed_);

  applyApproachVelocityScaling(path, linear_vel);

  // Limit linear velocities to be valid
  linear_vel = std::clamp(fabs(linear_vel), 0.0, desired_linear_vel_);
  linear_vel = sign * linear_vel;
}

nav_msgs::msg::Path RegulatedPurePursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose = pose;
  // max_robot_pose_search_dist_ == -1 => end 리턴
  auto closest_pose_upper_bound = first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin =
    min_by(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points up to max_transform_dist so we only transform them.
  // 로봇 위치 <-> path
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) {
      return euclidean_distance(pose, robot_pose) > std::numeric_limits<double>::max();
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose2D(stamped_pose, transformed_pose, odom2map_tf_);
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };
  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = current_pose_.header.frame_id;
  transformed_plan.header.stamp = robot_pose.header.stamp;
  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  RCLCPP_INFO(this->get_logger(), "transformation_begin x: %f", transformation_begin->pose.position.x);
  RCLCPP_INFO(this->get_logger(), "erased plans: %ld", global_plan_.poses.size());

  // if (transformed_plan.poses.empty()) {
  //   throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  // }

  return transformed_plan;
}

double RegulatedPurePursuitController::findVelocitySignChange(
  const nav_msgs::msg::Path & transformed_plan)
{
  // Iterating through the transformed global path to determine the position of the cusp
  for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = transformed_plan.poses[pose_id].pose.position.x -
      transformed_plan.poses[pose_id - 1].pose.position.x;
    double oa_y = transformed_plan.poses[pose_id].pose.position.y -
      transformed_plan.poses[pose_id - 1].pose.position.y;
    double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x -
      transformed_plan.poses[pose_id].pose.position.x;
    double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y -
      transformed_plan.poses[pose_id].pose.position.y;

    /* Checking for the existance of cusp, in the path, using the dot product
    and determine it's distance from the robot. If there is no cusp in the path,
    then just determine the distance to the goal location. */
    if ( (oa_x * ab_x) + (oa_y * ab_y) < 0.0) {
      // returning the distance if there is a cusp
      // The transformed path is in the robots frame, so robot is at the origin
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }
  }

  return std::numeric_limits<double>::max();
}

bool RegulatedPurePursuitController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }
  try {
    if (!tf_) {
    RCLCPP_ERROR(logger_, "TF listener is not initialized!");
    return false;
    }
    
    RCLCPP_INFO(logger_, "Transforming from frame: %s to frame: %s", in_pose.header.frame_id.c_str(), frame.c_str());
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    
    RCLCPP_INFO(this->get_logger(), "end transformPose using tf_");
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}
