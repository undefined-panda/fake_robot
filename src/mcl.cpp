/**
 * This class implements a Monte Carlo Localization algorithm.
*/

#include "mcl/mcl.hpp"

#include <cmath>
#include <algorithm>
#include <stdexcept>

MCL::MCL()
{
  std::random_device rd;
  rng_ = std::mt19937(rd());
}

MCL::~MCL() = default;

/**
 * @brief Initialize map and set map boundaries.
 * @param landmarks Vector of {id, x, y}
 */
void MCL::setMap(const std::vector<Landmark>& landmarks)
{
    if (landmarks.empty()) {
        throw std::runtime_error("MCL::setMap(): landmarks are empty");
    }
    
    map_.clear();
    map_.reserve(landmarks.size());

    for (const Landmark& lm : landmarks) {
        auto [it, inserted] = map_.emplace(lm.id, lm);
        if (!inserted) {
            throw std::runtime_error("Duplicate landmark id: " + std::to_string(lm.id));
        }
    }

    map_initialized_ = true;

    // calculate map bounds
    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(landmarks.size());
    ys.reserve(landmarks.size());
    for (const auto& landmark : landmarks){
        xs.push_back(landmark.x);
        ys.push_back(landmark.y);
    }

    auto [min_xIt, max_xIt] = std::minmax_element(xs.begin(), xs.end());
    auto [min_yIt, max_yIt] = std::minmax_element(ys.begin(), ys.end());
    double min_x = *min_xIt;
    double max_x = *max_xIt;
    double min_y = *min_yIt;
    double max_y = *max_yIt;

    bounds_ = {min_x, max_x, min_y, max_y};
}

/**
 * @brief Initialie num_particles particles uniformly distributed across the whole map.
 */
void MCL::initializeParticles()
{
    if (!map_initialized_) throw std::runtime_error("MCL::initializeParticles(): landmarks not initialized");

    particles_.clear();

    for (size_t i = 0; i < MCL::num_particles; i++){
        Particle particle;

        particle.x = uniform_dist(bounds_[0], bounds_[1]);
        particle.y = uniform_dist(bounds_[2], bounds_[3]);
        particle.theta = uniform_dist(-M_PI, M_PI);
        particle.weight = 1.0 / MCL::num_particles;

        particles_.push_back(particle);
  };

  particles_initialized_ = true;
}

/**
 * @brief Return particles.
 * @return Vector of {x, y, theta}
 */
const std::vector<Particle>& MCL::getParticles() const
{
  return particles_;
}

/**
 * @brief Apply odometry-based motion model to each particle
 * 
 * Calculation is taken from the lecture "Autonomous Mobile Robots" lecture 6 slide 40.
 * A Gaussian noise is added to translation and rotation. Orientations are normalized.
 * 
 * @param delta Structure with value difference of current and previous odometry
 * @param variance Translation and rotation variance
 */
void MCL::motionUpdate(MotionDelta delta, MotionNoise variance)
{
    if (particles_.empty()) throw std::runtime_error("MCL::motionUpdate(): no particles initialized");

    for (auto& particle : particles_){
        // gaussian noise
        double noise_rot1 = normal_dist(0.0, std::sqrt(variance.var_rot1));
        double noise_rot2 = normal_dist(0.0, std::sqrt(variance.var_rot2));
        double noise_trans = normal_dist(0.0, std::sqrt(variance.var_trans));

        // delta hat values
        double delta_trans_hat = std::max(0.0, delta.trans + noise_trans);
        double delta_rot1_hat = normalizeAngle(delta.rot1 + noise_rot1);
        double delta_rot2_hat = normalizeAngle(delta.rot2 + noise_rot2);

        // odometry-based motion model
        particle.x += delta_trans_hat * std::cos(particle.theta + delta_rot1_hat);
        particle.y += delta_trans_hat * std::sin(particle.theta + delta_rot1_hat);
        particle.theta = normalizeAngle(particle.theta + delta_rot1_hat + delta_rot2_hat);
    }
}

/**
 * @brief Compute likelihoof of particle using observed landmarks
 * 
 * 
 * Likelihood of particle, i.e. possibility of observations if robot pose is current particle.
 * Using log likelihood to avoid numerical underflow and using addition instead of multiplikation. Weights are normalized
 * 
 * @param observations Vector of {id, x, y}
 */
void MCL::measurementUpdate(const std::vector<Landmark>& observations)
{
    // particle weights stay the same if no observations are made
    if (observations.empty()) return;

    // or particles not initialized yet
    if (!particles_initialized_) return;

    double weight_sum = 0.0;
    std::vector<double> likelihoods;
    likelihoods.reserve(particles_.size());
    double maxlog = -std::numeric_limits<double>::infinity();

    for (Particle& particle : particles_){
        double log_likelihood = 0.0;

        // calculate distance of observed landmark to true landmark
        for (const Landmark& observation : observations){
            auto it = map_.find(observation.id);
            if (it == map_.end()) continue;
            const Landmark& lm_map = it->second;

            // transform landmark position from world frame (landmarks) to robot frame (observations)

            // first: translation of world frame to robot frame           
            double lm_x = lm_map.x - particle.x;
            double lm_y = lm_map.y - particle.y;

            // second: inverse rotation of world frame vector by particle.theta to robot frame
            // x_r = R(-theta) * x_w
            // R(theta) = (cos(theta) -sin(theta))
            //            (sin(theta)  cos(theta))
            double lm_x_r = std::cos(particle.theta) * lm_x + std::sin(particle.theta) * lm_y;
            double lm_y_r = -std::sin(particle.theta) * lm_x + std::cos(particle.theta) * lm_y;

            // using distance directly instead of std::exp(...) and then std::log(...)
            double lm_distance = -std::pow(distance(lm_x_r, lm_y_r, observation.x, observation.y), 2) / (2 * MCL::measurement_noise_variance);

            log_likelihood += lm_distance;
        }

        maxlog = std::max(maxlog, log_likelihood);
        likelihoods.push_back(log_likelihood);
    }

    // subtract max log for stabilization
    for (size_t i = 0; i < particles_.size(); i++){
        particles_[i].weight *= std::exp(likelihoods[i] - maxlog);
        weight_sum += particles_[i].weight;
    }

    // normalize weights
    if (weight_sum > 0){
        for (Particle& particle : particles_) particle.weight /= weight_sum;
    }
}

/**
 * @brief Resample particles using low-variance resampling
 * 
 * Low-variance calculation is taken from the lecture "Autonomous Mobile Robots" lecture 6 slide 60.
 */
void MCL::resampling()
{
    // no resampling if particles are not initialized yet
    if (!particles_initialized_) return;

    std::vector<Particle> resampled_particles;
    resampled_particles.clear();
    resampled_particles.reserve(MCL::num_particles);

    // only resample if samples have significantly different weights
    const double Neff = MCL::num_effective_particles();
    if (Neff < 0.5 * particles_.size()) {

        // lay particle weights in order and create M uniform distributed marker that point at some weights
        // sample particles where the current marker points at

        double r = uniform_dist(0.0, 1.0 / static_cast<double>(MCL::num_particles)); // random start for marker
        double cumulative_weight = particles_[0].weight; // keep track of weights at current marker
        int j = 0;

        for (int i = 0; i < MCL::num_particles; i++) {
            double marker_pos = r + static_cast<double>(i) / static_cast<double>(MCL::num_particles); // increase marker position by 1/M
            
            // increase cumulative_weight until current marker position is reached to get to next particle weight
            while (marker_pos > cumulative_weight && j < particles_.size() - 1){
                j++;
                cumulative_weight += particles_[j].weight;
            }

            Particle p = particles_[j];
            p.weight = 1.0 / static_cast<double>(MCL::num_particles);;
            resampled_particles.push_back(p);
        }

        particles_ = resampled_particles;
    }
}

/**
 * @brief Estimate pose by computing weighted mean of particles and publish to ROS2 topic.
 * @return Estimated pose - computed weighted mean of particles
 */
std::optional<Pose> MCL::poseEstimation()
{
    if (!particles_initialized_) return std::nullopt;

    double x_mean = 0.0;
    double y_mean = 0.0;
    double sin_mean = 0.0;
    double cos_mean = 0.0;

    for (const Particle& particle : particles_){
        x_mean += particle.x * particle.weight;
        y_mean += particle.y * particle.weight;

        // for theta_mean
        sin_mean += particle.weight * std::sin(particle.theta);
        cos_mean += particle.weight * std::cos(particle.theta);
    }

    double theta_mean = std::atan2(sin_mean, cos_mean);
    theta_mean = normalizeAngle(theta_mean);

    return Pose {x_mean, y_mean, theta_mean};
}

// ----------------------------
// Helper Functions
// ----------------------------

double MCL::normalizeAngle(double angle) 
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double MCL::uniform_dist(double min, double max)
{
    std::uniform_real_distribution<double> dist(min, max);
    return dist(rng_);
}

double MCL::normal_dist(double mean, double cov)
{
    std::normal_distribution<double> dist(mean, cov);
    return dist(rng_);
}

double MCL::distance(double x1, double y1, double x2, double y2) 
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double MCL::num_effective_particles()
{
    std::vector<double> particle_weights;
    particle_weights.clear();
    particle_weights.reserve(MCL::num_particles);

    for (const Particle& particle : particles_){
        particle_weights.push_back(particle.weight);
    }

    double sum = 0.0;
    for (auto weight : particle_weights){
        sum += std::pow(weight, 2);
    }

    if (sum <= 0) return 0.0;
    return 1/sum;
}
