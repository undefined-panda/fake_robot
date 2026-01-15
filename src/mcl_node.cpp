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
        this->declare_parameter<double>("robot_noise_variance_theta", 0.01);
        this->declare_parameter<int>("num_particles", 100);
        this->declare_parameter<int>("M", 100);

        
        double meas_var = this->get_parameter("measurement_noise_variance").as_double();
        // double rot_var  = this->get_parameter("robot_noise_variance_theta").as_double();
        num_particles = this->get_parameter("num_particles").as_int();
        M = this->get_parameter("M").as_int();

        // initialize MCL
        mcl_ = std::make_unique<MCL>();
        mcl_->measurement_noise_variance = meas_var;

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
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_gt_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_obs_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_;
    std::unique_ptr<MCL> mcl_;

    bool initialized_ = false; // to check if map and particles are initialized
    Pose prev_odom_;
    bool first_odom_ = true; // to check if received odometry is first one

    int num_particles;
    int M;

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

            mcl_->initializeParticles(num_particles);
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

        delta.rot1 = mcl_->normalizeAngle(std::atan2(y_diff, x_diff) - prev_odom_.theta);
        delta.rot2 = mcl_->normalizeAngle(theta_diff - delta.rot1);
        delta.trans = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));

        // set variance values
        MotionNoise variance;

        double var_x = msg->pose.covariance[0];
        double var_y = msg->pose.covariance[7];
        double var_yaw = msg->pose.covariance[35];

        variance.var_trans = std::max(1e-6, 0.5*(var_x + var_y));
        variance.var_rot   = std::max(1e-6, var_yaw);

        // RCLCPP_INFO(this->get_logger(), "enter motionUpdate");
        mcl_->motionUpdate(delta, variance);
        // RCLCPP_INFO(this->get_logger(), "left motionUpdate");
        
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

            RCLCPP_INFO(this->get_logger(), "obs=%zu", landmark_observations.size());
            mcl_->measurementUpdate(landmark_observations);

            mcl_->resampling(M);

            publishParticles();

            publishEstimatedPose();
            
            RCLCPP_DEBUG(this->get_logger(), 
                "Processed %d landmark observations", count);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), 
                "Failed to parse landmark observations: %s", e.what());
        }
    }

    void publishEstimatedPose()
    {
        // RCLCPP_INFO(this->get_logger(), "calling publishEstimatedPose");
        
        auto pose_opt = mcl_->poseEstimation();
        if (!pose_opt) return;

        Pose pos = *pose_opt;

        geometry_msgs::msg::PoseStamped estimated_pose;
        estimated_pose.header.stamp = this->now();
        estimated_pose.header.frame_id = "map";

        estimated_pose.pose.position.x = pos.x;
        estimated_pose.pose.position.y = pos.y;

        tf2::Quaternion q;
        q.setRPY(0, 0, pos.theta);
        q.normalize();
        estimated_pose.pose.orientation = tf2::toMsg(q);

        estimated_pose_pub_->publish(estimated_pose);
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
