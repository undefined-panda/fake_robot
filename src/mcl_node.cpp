/**
 * This ROS2 node subsribes to topics from fake_robot and uses the MCL class from mcl.cpp to
 * apply the MCL algorithm.
*/

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "mcl/mcl.hpp"

class MCLNode : public rclcpp::Node {
public:
    MCLNode() : Node("mcl_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Monte Carlo Localization");

        this->declare_parameter<double>("num_particles", 100);
        this->declare_parameter<double>("M", 50);
        double num_particles = this->get_parameter("num_particles").as_double();
        int M = this->get_parameter("M").as_double();

        // initialize MCL
        mcl_ = std::make_unique<MCL>();

        // Create subscriber
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot_noisy",
            rclcpp::QoS(10),
            std::bind(&MCLNode::odometryCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /robot_noisy");

        landmarks_gt_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/landmarks_gt",
            rclcpp::QoS(10),
            std::bind(&MCLNode::landmarksGtCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /landmarks_gt");

        landmarks_obs_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/landmarks_observed",
            rclcpp::QoS(10),
            std::bind(&MCLNode::landmarksObservedCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /landmarks_observed");

        // Create publisher
        estimated_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mcl_estimated_pose", rclcpp::QoS(10)
        );
        RCLCPP_INFO(this->get_logger(), "Publishing to /mcl_estimated_pose");
        

        RCLCPP_INFO(this->get_logger(), "MCL Node initialized successfully");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_gt_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_obs_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_pub_;
    std::unique_ptr<MCL> mcl_;

    double num_particles;
    int M;

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Odometry received: x=%.3f, y=%.3f", msg->pose.pose.position.x, msg->pose.pose.position.y);

        MotionDelta delta;
        MotionNoise variance;

        mcl_->motionUpdate(delta, variance);
    }

    void landmarksGtCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        try {
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

            mcl_->initializeParticles(num_particles);

            RCLCPP_DEBUG(this->get_logger(), "Processed %d landmark observations", count);

        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse landmark observations: %s", e.what());
        }
    }

    void landmarksObservedCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), 
            "Landmark observation received with %u points", msg->width);
        
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

            mcl_->resampling(M);

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
        Pose pos = mcl_->poseEstimation();

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
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MCLNode>());
  rclcpp::shutdown();
  return 0;
}
