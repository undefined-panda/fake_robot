#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <random>

struct Landmark {
  int id;
  double x;
  double y;
};

class LandmarkPublisher : public rclcpp::Node {
public:
  LandmarkPublisher() : Node("landmark_publisher") {
    // Create publishers
    landmarks_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("landmarks_gt", 10);
    robot_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("robot_gt", 10);
    robot_noisy_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("robot_noisy", 10);
    observed_landmarks_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("landmarks_observed", 10);
    
      // Declare parameters for noisy robot pose
      this->declare_parameter("robot_noise_variance_x", 0.05);
      this->declare_parameter("robot_noise_variance_y", 0.05);
      this->declare_parameter("robot_noise_variance_theta", 0.01);
    
    // Create transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
    // Declare parameters for circle trajectory
    this->declare_parameter("circle_radius", 5.0);
    this->declare_parameter("circle_speed", 1.0);  // m/s
    this->declare_parameter("circle_center_x", 0.0);
    this->declare_parameter("circle_center_y", 0.0);
    
    // Declare parameters for landmark observation
    this->declare_parameter("observation_radius", 10.0);  // Radius around robot to observe landmarks
    this->declare_parameter("measurement_noise_variance", 0.1);  // Gaussian noise variance
    
    circle_radius_ = this->get_parameter("circle_radius").as_double();
    circle_speed_ = this->get_parameter("circle_speed").as_double();
    circle_center_x_ = this->get_parameter("circle_center_x").as_double();
    circle_center_y_ = this->get_parameter("circle_center_y").as_double();
    observation_radius_ = this->get_parameter("observation_radius").as_double();
    measurement_noise_variance_ = this->get_parameter("measurement_noise_variance").as_double();
      robot_noise_variance_x_ = this->get_parameter("robot_noise_variance_x").as_double();
      robot_noise_variance_y_ = this->get_parameter("robot_noise_variance_y").as_double();
      robot_noise_variance_theta_ = this->get_parameter("robot_noise_variance_theta").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Circle parameters - radius: %.2f m, speed: %.2f m/s, center: (%.2f, %.2f)",
                circle_radius_, circle_speed_, circle_center_x_, circle_center_y_);
    RCLCPP_INFO(this->get_logger(), "Observation parameters - radius: %.2f m, noise variance: %.4f",
                observation_radius_, measurement_noise_variance_);
      RCLCPP_INFO(this->get_logger(), "Robot noise parameters - x_var: %.4f, y_var: %.4f, theta_var: %.4f",
            robot_noise_variance_x_, robot_noise_variance_y_, robot_noise_variance_theta_);
    
    // Initialize random number generator
    rng_.seed(std::random_device{}());
    
    // Declare and get the landmarks file path parameter
    this->declare_parameter("landmarks_file", "landmarks.txt");
    std::string landmarks_file = this->get_parameter("landmarks_file").as_string();
    
    RCLCPP_DEBUG(this->get_logger(), "Landmarks file: %s", landmarks_file.c_str());
    
    // Load landmarks from file
    if (!load_landmarks(landmarks_file)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load landmarks from file: %s", landmarks_file.c_str());
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Loaded %zu landmarks", landmarks_.size());

    // Create .csv files to store values
    robot_gt_csv_.open("robot_gt.csv", std::ios::out | std::ios::trunc);
    if (robot_gt_csv_.is_open()) {
      robot_gt_csv_ << "timestamp,x,y,theta,vx,vy,vtheta\n";
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: robot_gt.csv");
    }
    
    // Initialize both GT and noisy robot pose to same starting location on circle
    // At t=0: theta=0, position on circle at theta=0
    double initial_theta = 0.0;
    robot_x_ = circle_center_x_ + circle_radius_ * std::cos(initial_theta);
    robot_y_ = circle_center_y_ + circle_radius_ * std::sin(initial_theta);
    robot_theta_gt_ = initial_theta + M_PI / 2.0;  // Tangent direction
    
    robot_noisy_x_ = robot_x_;
    robot_noisy_y_ = robot_y_;
    robot_noisy_theta_ = robot_theta_gt_;
    
    // Create a timer to publish landmarks periodically (1 Hz)
    landmarks_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&LandmarkPublisher::publish_landmarks, this));
    
    // Create a timer for robot trajectory (10 Hz for smooth motion)
    robot_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LandmarkPublisher::publish_robot_trajectory, this));
    
    // Create a timer for observed landmarks (10 Hz)
    observed_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LandmarkPublisher::publish_observed_landmarks, this));
    
      // Create a timer for noisy robot pose (10 Hz)
      robot_noisy_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&LandmarkPublisher::publish_robot_noisy_pose, this));
  }

private:
  std::vector<Landmark> landmarks_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_noisy_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr observed_landmarks_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr landmarks_timer_;
  rclcpp::TimerBase::SharedPtr robot_timer_;
  rclcpp::TimerBase::SharedPtr observed_timer_;
    rclcpp::TimerBase::SharedPtr robot_noisy_timer_;

  // ADDED BY ME: CSV files to store values
  std::ofstream robot_gt_csv_;
  
  // Circle trajectory parameters
  double circle_radius_;
  double circle_speed_;
  double circle_center_x_;
  double circle_center_y_;
  double elapsed_time_ = 0.0;  // Track elapsed time for circle calculation
  double dt_ = 0.1;  // Timer interval in seconds
  
  // Landmark observation parameters
  double observation_radius_;
  double measurement_noise_variance_;
    double robot_noisy_x_ = 0.0;
    double robot_noisy_y_ = 0.0;
    double robot_noisy_theta_ = 0.0;
  
    // Robot noise parameters
    double robot_noise_variance_x_;
    double robot_noise_variance_y_;
    double robot_noise_variance_theta_;
  double robot_x_ = 0.0;  // Current robot x position
  double robot_y_ = 0.0;  // Current robot y position
  double robot_theta_gt_ = 0.0;  // Current GT robot orientation
  
  // Random number generator for noise
  std::mt19937 rng_;
  std::normal_distribution<double> noise_dist_{0.0, 1.0};  // Will be scaled by std dev
  
  bool load_landmarks(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", filename.c_str());
      return false;
    }
    
    std::string line;
    while (std::getline(file, line)) {
      // Skip empty lines and comments
      if (line.empty() || line[0] == '#') {
        continue;
      }
      
      // Parse CSV format: id,x,y
      std::string id_str, x_str, y_str;
      size_t pos1 = line.find(',');
      size_t pos2 = line.find(',', pos1 + 1);
      
      if (pos1 == std::string::npos || pos2 == std::string::npos) {
        RCLCPP_WARN(this->get_logger(), "Invalid line format (expected id,x,y): %s", line.c_str());
        continue;
      }
      
      try {
        id_str = line.substr(0, pos1);
        x_str = line.substr(pos1 + 1, pos2 - pos1 - 1);
        y_str = line.substr(pos2 + 1);
        
        int id = std::stoi(id_str);
        double x = std::stod(x_str);
        double y = std::stod(y_str);
        
        landmarks_.push_back({id, x, y});
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse line: %s (error: %s)", line.c_str(), e.what());
        continue;
      }
    }
    
    file.close();
    return !landmarks_.empty();
  }
  
  void publish_landmarks() {
    if (landmarks_.empty()) {
      return;
    }
    
    auto now = this->get_clock()->now();
    auto pc = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pc->header.stamp = now;
    pc->header.frame_id = "map";
    pc->height = 1;
    pc->width = landmarks_.size();
    
    // Define fields: x, y, z (fixed), id
    pc->fields.resize(4);
    pc->fields[0].name = "x";
    pc->fields[0].offset = 0;
    pc->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc->fields[0].count = 1;
    
    pc->fields[1].name = "y";
    pc->fields[1].offset = 4;
    pc->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc->fields[1].count = 1;
    
    pc->fields[2].name = "z";
    pc->fields[2].offset = 8;
    pc->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc->fields[2].count = 1;
    
    pc->fields[3].name = "id";
    pc->fields[3].offset = 12;
    pc->fields[3].datatype = sensor_msgs::msg::PointField::INT32;
    pc->fields[3].count = 1;
    
    pc->point_step = 16;  // 4 bytes (x) + 4 bytes (y) + 4 bytes (z) + 4 bytes (id)
    pc->row_step = pc->point_step * pc->width;
    pc->data.resize(pc->row_step);
    pc->is_dense = true;
    
    // Fill point cloud data
    for (size_t i = 0; i < landmarks_.size(); ++i) {
      float* ptr = (float*)&pc->data[i * pc->point_step];
      ptr[0] = landmarks_[i].x;  // x
      ptr[1] = landmarks_[i].y;  // y
      ptr[2] = 0.0;               // z (fixed)
      
      int32_t* id_ptr = (int32_t*)&pc->data[i * pc->point_step + 12];
      *id_ptr = landmarks_[i].id;  // id
    }
    
    landmarks_publisher_->publish(std::move(pc));
  }
  
  void publish_robot_trajectory() {
    // Calculate position on circle using parametric equations
    // x = center_x + radius * cos(theta)
    // y = center_y + radius * sin(theta)
    // theta = (speed / radius) * time
    
    double angular_velocity = circle_speed_ / circle_radius_;  // rad/s
    double theta = angular_velocity * elapsed_time_;
    
    auto now = this->get_clock()->now();

    // Compute previous GT pose for velocity (if first step, previous == current)
    double theta_prev = angular_velocity * (elapsed_time_ - dt_);
    double gt_x_curr = circle_center_x_ + circle_radius_ * std::cos(theta);
    double gt_y_curr = circle_center_y_ + circle_radius_ * std::sin(theta);
    double gt_x_prev = circle_center_x_ + circle_radius_ * std::cos(theta_prev);
    double gt_y_prev = circle_center_y_ + circle_radius_ * std::sin(theta_prev);

    double motion_x = gt_x_curr - gt_x_prev;
    double motion_y = gt_y_curr - gt_y_prev;
    double motion_theta = theta - theta_prev;

    // Calculate traveled distance and motion direction
    double traveled_distance = std::sqrt(motion_x * motion_x + motion_y * motion_y);
    double motion_direction = std::atan2(motion_y, motion_x);

    // Compute position along motion direction (not arbitrary x,y)
    robot_x_ = robot_x_ + traveled_distance * std::cos(motion_direction);
    robot_y_ = robot_y_ + traveled_distance * std::sin(motion_direction);
    robot_theta_gt_ = robot_theta_gt_ + motion_theta;

    // Create and publish odometry message for ground truth
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "map";
    odom.child_frame_id = "robot";
    odom.pose.pose.position.x = robot_x_;
    odom.pose.pose.position.y = robot_y_;
    odom.pose.pose.position.z = 0.0;
    // Robot orientation is tangent to circle
    double half_theta_gt = robot_theta_gt_ / 2.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(half_theta_gt);
    odom.pose.pose.orientation.w = std::cos(half_theta_gt);

    // Fill twist from motion increment
    odom.twist.twist.linear.x = traveled_distance * std::cos(motion_direction) / dt_;
    odom.twist.twist.linear.y = traveled_distance * std::sin(motion_direction) / dt_;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = motion_theta / dt_;

    // Minimal covariance for ground truth (near-zero)
    for (size_t i = 0; i < 36; ++i) odom.pose.covariance[i] = 0.0;
    for (size_t i = 0; i < 36; ++i) odom.twist.covariance[i] = 0.0;

    robot_publisher_->publish(odom);

    // ADDED BY ME
    if (robot_gt_csv_.is_open()) {
      int64_t timestamp_ns = now.nanoseconds();
      robot_gt_csv_ << timestamp_ns << ","
                    << robot_x_ << ","
                    << robot_y_ << ","
                    << robot_theta_gt_ << ","
                    << odom.twist.twist.linear.x << ","
                    << odom.twist.twist.linear.y << ","
                    << odom.twist.twist.angular.z << "\n";
      robot_gt_csv_.flush();
    }

    // Publish robot transform (same as before)
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = now;
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "robot";
    transform_stamped.transform.translation.x = robot_x_;
    transform_stamped.transform.translation.y = robot_y_;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform_stamped);

    // Update elapsed time
    elapsed_time_ += dt_;
  }
  
  void publish_observed_landmarks() {
    if (landmarks_.empty()) {
      return;
    }
    
    auto now = this->get_clock()->now();
    double std_dev = std::sqrt(measurement_noise_variance_);
    int observed_count = 0;
    
    // Collect observed landmarks
    std::vector<std::pair<Landmark, double>> observed;  // landmark + noise
    for (const auto& landmark : landmarks_) {
      double dx = landmark.x - robot_x_;
      double dy = landmark.y - robot_y_;
      double distance = std::sqrt(dx * dx + dy * dy);
      
      // If landmark is within observation radius
      if (distance <= observation_radius_) {
        observed.push_back({landmark, distance});
        observed_count++;
      }
    }
    
    // Create PointCloud2 message
    auto pc = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pc->header.stamp = now;
    pc->header.frame_id = "robot";
    pc->height = 1;
    pc->width = observed.size();
    
    // Define fields: x, y, z (fixed), id
    pc->fields.resize(4);
    pc->fields[0].name = "x";
    pc->fields[0].offset = 0;
    pc->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc->fields[0].count = 1;
    
    pc->fields[1].name = "y";
    pc->fields[1].offset = 4;
    pc->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc->fields[1].count = 1;
    
    pc->fields[2].name = "z";
    pc->fields[2].offset = 8;
    pc->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc->fields[2].count = 1;
    
    pc->fields[3].name = "id";
    pc->fields[3].offset = 12;
    pc->fields[3].datatype = sensor_msgs::msg::PointField::INT32;
    pc->fields[3].count = 1;
    
    pc->point_step = 16;  // 4 bytes (x) + 4 bytes (y) + 4 bytes (z) + 4 bytes (id)
    pc->row_step = pc->point_step * pc->width;
    pc->data.resize(pc->row_step);
    pc->is_dense = true;
    
    // Fill point cloud data with noise
    for (size_t i = 0; i < observed.size(); ++i) {
      double noise_x = noise_dist_(rng_) * std_dev;
      double noise_y = noise_dist_(rng_) * std_dev;

      double dx = observed[i].first.x - robot_x_;
      double dy = observed[i].first.y - robot_y_;

      double c = std::cos(robot_theta_gt_);
      double s = std::sin(robot_theta_gt_);

      double xr =  c * dx + s * dy;
      double yr = -s * dx + c * dy;

      xr += noise_x;
      yr += noise_y;

      float *ptr = (float *)&pc->data[i * pc->point_step];
      ptr[0] = xr;
      ptr[1] = yr;
      ptr[2] = 0.0f;
      
      int32_t* id_ptr = (int32_t*)&pc->data[i * pc->point_step + 12];
      *id_ptr = observed[i].first.id;  // id
    }
    
    observed_landmarks_publisher_->publish(std::move(pc));
  }

  void publish_robot_noisy_pose() {
    // Compute noisy pose by adding incremental noise to previous noisy pose
    auto now = this->get_clock()->now();

    // Calculate ground truth motion between steps
    double angular_velocity = circle_speed_ / circle_radius_;
    double theta = angular_velocity * elapsed_time_;
    double theta_prev = angular_velocity * (elapsed_time_ - dt_);

    double gt_x = circle_center_x_ + circle_radius_ * std::cos(theta);
    double gt_y = circle_center_y_ + circle_radius_ * std::sin(theta);
    double gt_x_prev = circle_center_x_ + circle_radius_ * std::cos(theta_prev);
    double gt_y_prev = circle_center_y_ + circle_radius_ * std::sin(theta_prev);

    double motion_x = gt_x - gt_x_prev;
    double motion_y = gt_y - gt_y_prev;
    double motion_theta = theta - theta_prev;

    // Calculate traveled distance and motion direction
    double traveled_distance = std::sqrt(motion_x * motion_x + motion_y * motion_y);
    double motion_direction = std::atan2(motion_y, motion_x);

    // Add noise proportional to traveled distance
    // Scale noise std dev by distance ratio: distance / circle_speed / dt
    double distance_scale = traveled_distance / (circle_speed_ * dt_);
    double distance_noise = noise_dist_(rng_) * std::sqrt(robot_noise_variance_x_) * distance_scale;
    double theta_noise = noise_dist_(rng_) * std::sqrt(robot_noise_variance_theta_) * distance_scale;

    // Apply noise along direction of motion (not arbitrarily in x,y)
    double noisy_distance = traveled_distance + distance_noise;
    double noisy_motion_x = noisy_distance * std::cos(motion_direction);
    double noisy_motion_y = noisy_distance * std::sin(motion_direction);
    double noisy_motion_theta = motion_theta + theta_noise;

    // Update noisy pose (noise accumulates along motion direction and orientation)
    robot_noisy_x_ += noisy_motion_x;
    robot_noisy_y_ += noisy_motion_y;
    robot_noisy_theta_ += noisy_motion_theta;

    // Publish odometry for noisy pose
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "map";
    odom.child_frame_id = "robot_noisy";
    odom.pose.pose.position.x = robot_noisy_x_;
    odom.pose.pose.position.y = robot_noisy_y_;
    odom.pose.pose.position.z = 0.0;
    double half_theta_noisy = robot_noisy_theta_ / 2.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(half_theta_noisy);
    odom.pose.pose.orientation.w = std::cos(half_theta_noisy);

    // Set twist based on noisy increment
    odom.twist.twist.linear.x = noisy_motion_x / dt_;
    odom.twist.twist.linear.y = noisy_motion_y / dt_;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.z = noisy_motion_theta / dt_;

    // Fill pose covariance from noise parameters
    for (size_t i = 0; i < 36; ++i) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0] = robot_noise_variance_x_;
    odom.pose.covariance[7] = robot_noise_variance_y_;
    odom.pose.covariance[35] = robot_noise_variance_theta_;

    // Twist covariance (variance over dt)
    for (size_t i = 0; i < 36; ++i) odom.twist.covariance[i] = 0.0;
    double inv_dt2 = 1.0 / (dt_ * dt_);
    odom.twist.covariance[0] = robot_noise_variance_x_ * inv_dt2;
    odom.twist.covariance[7] = robot_noise_variance_y_ * inv_dt2;
    odom.twist.covariance[35] = robot_noise_variance_theta_ * inv_dt2;

    robot_noisy_publisher_->publish(odom);

    // Also broadcast TF for noisy robot
    geometry_msgs::msg::TransformStamped noisy_tf;
    noisy_tf.header.stamp = now;
    noisy_tf.header.frame_id = "map";
    noisy_tf.child_frame_id = "robot_noisy";
    noisy_tf.transform.translation.x = robot_noisy_x_;
    noisy_tf.transform.translation.y = robot_noisy_y_;
    noisy_tf.transform.translation.z = 0.0;
    noisy_tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(noisy_tf);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LandmarkPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

