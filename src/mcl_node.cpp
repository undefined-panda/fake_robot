/**
 * This ROS2 node subsribes to topics from fake_robot and uses the MCL class from mcl.cpp to apply the MCL algorithm.
*/

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "mcl/mcl.hpp"
#include <fstream>

/**
 * @brief Convert quaternion to yaw angle
 * @param q Quaternion from orientation
 * @return Yaw angle in radians [-pi, pi]
 */
double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    return yaw;
}

class MCLNode : public rclcpp::Node {
public:
    MCLNode() : Node("mcl_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Monte Carlo Localization");

        this->declare_parameter<double>("measurement_noise_variance", 0.1);
        this->declare_parameter<int>("num_particles", 250);
        this->declare_parameter<double>("alpha1", 0.05);
        this->declare_parameter<double>("alpha2", 0.05);
        this->declare_parameter<double>("alpha3", 0.1);
        this->declare_parameter<double>("alpha4", 0.05);

        double measurement_noise_variance = this->get_parameter("measurement_noise_variance").as_double();
        num_particles = this->get_parameter("num_particles").as_int();
        alpha1 = this->get_parameter("alpha1").as_double();
        alpha2 = this->get_parameter("alpha2").as_double();
        alpha3 = this->get_parameter("alpha3").as_double();
        alpha4 = this->get_parameter("alpha4").as_double();

        RCLCPP_INFO(this->get_logger(), "MCL started with %d particles", num_particles);

        // initialize MCL
        mcl_ = std::make_unique<MCL>();
        mcl_->measurement_noise_variance = measurement_noise_variance;
        mcl_->num_particles = num_particles;

        // Create subscriber
        landmarks_gt_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/landmarks_gt",
            rclcpp::QoS(10),
            std::bind(&MCLNode::landmarksGtCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /landmarks_gt");

        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot_noisy",
            rclcpp::QoS(10),
            std::bind(&MCLNode::odometryCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /robot_noisy");

        landmarks_obs_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/landmarks_observed",
            rclcpp::QoS(10),
            std::bind(&MCLNode::landmarksObservedCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /landmarks_observed");

        // Create publisher
        estimated_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mcl_pose", rclcpp::QoS(10)
        );
        RCLCPP_INFO(this->get_logger(), "Publishing to /mcl_pose");

        particles_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/particles", rclcpp::QoS(10)
        );
        RCLCPP_INFO(this->get_logger(), "Publishing to /particles");
        
        RCLCPP_INFO(this->get_logger(), "MCL Node initialized successfully");


        estimated_pos_.open("estimated_pos.csv", std::ios::out | std::ios::trunc);
        if (estimated_pos_.is_open()) {
            estimated_pos_ << "timestamp,x,y,theta\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: estimated_pos.csv");
        }
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_gt_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_obs_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_;
    std::unique_ptr<MCL> mcl_;
    std::ofstream estimated_pos_;

    bool initialized_ = false; // to check if map and particles are initialized
    Pose prev_odom_;
    bool first_odom_ = true; // to check if received odometry is first one

    int num_particles;

    double alpha1;
    double alpha2;
    double alpha3;
    double alpha4;

    void landmarksGtCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "calling landmarksGtCallback");
        try {
            if (initialized_) return; // only initialize map and particles once

            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_id(*msg, "id");

            std::vector<Landmark> landmarks;

            int count = 0;
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_id) {
                int landmark_id = static_cast<int>(*iter_id);
                float obs_x = static_cast<float>(*iter_x);
                float obs_y = static_cast<float>(*iter_y);
                
                RCLCPP_DEBUG(this->get_logger(), "Landmark %d observed at (%.3f, %.3f)", landmark_id, obs_x, obs_y);

                Landmark landmark {landmark_id, obs_x, obs_y};
                landmarks.push_back(landmark);
                
                count++;
            }

            mcl_->setMap(landmarks);
            RCLCPP_INFO(this->get_logger(), "Map initialized");

            mcl_->initializeParticles();
            RCLCPP_INFO(this->get_logger(), "Particles initialized");

            RCLCPP_DEBUG(this->get_logger(), "Processed %d landmark observations", count);

            initialized_ = true;

        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse landmark observations: %s", e.what());
        }
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Odometry received: x=%.3f, y=%.3f", msg->pose.pose.position.x, msg->pose.pose.position.y);
        // RCLCPP_INFO(this->get_logger(), "calling odometryCallback");

        if (!initialized_) return;

        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = quaternionToYaw(msg->pose.pose.orientation);

        Pose curr_odom {x, y, theta};

        if (first_odom_) {
            prev_odom_ = curr_odom;
            first_odom_ = false;
            return;
        }

        // set delta values
        MotionDelta delta;

        double x_diff = curr_odom.x - prev_odom_.x;
        double y_diff = curr_odom.y - prev_odom_.y;
        double theta_diff = mcl_->normalizeAngle(curr_odom.theta - prev_odom_.theta);

        delta.trans = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));
        delta.rot1 = mcl_->normalizeAngle(std::atan2(y_diff, x_diff) - prev_odom_.theta);
        delta.rot2 = mcl_->normalizeAngle(theta_diff - delta.rot1);

        // set variance values
        MotionNoise variance;
        variance.var_trans = alpha3 * std::pow(delta.trans, 2) + alpha4 * std::pow(delta.rot1, 2) + alpha4 * std::pow(delta.rot2, 2);
        variance.var_rot1 = alpha1 * std::pow(delta.rot1, 2) + alpha2 * std::pow(delta.trans, 2);
        variance.var_rot2 = alpha1 * std::pow(delta.rot2, 2) + alpha2 * std::pow(delta.trans, 2);

        mcl_->motionUpdate(delta, variance);
        
        prev_odom_ = curr_odom;
    }

    void landmarksObservedCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), 
            "Landmark observation received with %u points", msg->width);
        // RCLCPP_INFO(this->get_logger(), "calling landmarksObservedCallback");
        
        try {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_id(*msg, "id");

            // vector with landmark observations
            std::vector<Landmark> landmark_observations;
            
            int count = 0;
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_id) {
                int landmark_id = static_cast<int>(*iter_id);
                float obs_x = static_cast<float>(*iter_x);
                float obs_y = static_cast<float>(*iter_y);
                
                RCLCPP_DEBUG(this->get_logger(),
                    "Landmark %d observed at (%.3f, %.3f)",
                    landmark_id, obs_x, obs_y);

                Landmark landmark_observation = {landmark_id, obs_x, obs_y};
                landmark_observations.push_back(landmark_observation);
                
                count++;
            }

            mcl_->measurementUpdate(landmark_observations);

            mcl_->resampling();

            publishParticles();

            publishEstimatedPose(msg);
            
            RCLCPP_DEBUG(this->get_logger(), 
                "Processed %d landmark observations", count);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), 
                "Failed to parse landmark observations: %s", e.what());
        }
    }

    void publishEstimatedPose(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {        
        auto pose_opt = mcl_->poseEstimation();
        if (!pose_opt) return;

        Pose pos = *pose_opt;
        const rclcpp::Time timestamp = msg->header.stamp;

        geometry_msgs::msg::PoseStamped estimated_pose;
        estimated_pose.header.stamp = timestamp;
        estimated_pose.header.frame_id = "map";

        estimated_pose.pose.position.x = pos.x;
        estimated_pose.pose.position.y = pos.y;

        tf2::Quaternion q;
        q.setRPY(0, 0, pos.theta);
        q.normalize();
        estimated_pose.pose.orientation = tf2::toMsg(q);

        estimated_pose_pub_->publish(estimated_pose);

        // Store values in CSV file
        if (estimated_pos_.is_open()) {
            int64_t timestamp_ns = timestamp.nanoseconds();
            estimated_pos_ << timestamp_ns << ","
                           << pos.x << ","
                           << pos.y << ","
                           << pos.theta << "\n";
            estimated_pos_.flush();
        }
    }

    void publishParticles()
    {
        const auto& particles = mcl_->getParticles();

        geometry_msgs::msg::PoseArray pa;
        pa.header.stamp = this->now();
        pa.header.frame_id = "map";

        pa.poses.reserve(particles.size());

        for (const auto& p : particles) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p.x;
            pose.position.y = p.y;
            pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, p.theta);
            q.normalize();
            pose.orientation = tf2::toMsg(q);

            pa.poses.push_back(pose);
        }

        particles_->publish(pa);
    }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MCLNode>());
  rclcpp::shutdown();
  return 0;
}
