/**
 * This class implements a Monte Carlo Localization algorithm.
 * Author: Jad Dayoub
 * Student ID: 2848370
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
 * @brief Set values for the map and map boundaries.
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
 * @brief Initialie num_particles particles.
 */
void MCL::initializeParticles(std::size_t num_particles)
{
    if (!map_initialized_) throw std::runtime_error("MCL::initializeParticles(): landmarks not initialized");

    particles_.clear();

    for (size_t i = 0; i < num_particles; i++){
        Particle particle;

        particle.x = uniform_dist(bounds_[0], bounds_[1]);
        particle.y = uniform_dist(bounds_[2], bounds_[3]);
        particle.theta = uniform_dist(-M_PI, M_PI);
        particle.weight = 1.0 / num_particles;

        particles_.push_back(particle);
  };
}

/**
 * @brief Return particles.
 */
const std::vector<Particle>& MCL::getParticles() const
{
  return particles_;
}

// x - particle, x with _ on top - odometry pose, x' with _ - next odometry pose, x' - next particle
// lecture 6 slide 40
void MCL::motionUpdate(MotionDelta delta, MotionNoise variance)
{
    if (particles_.empty()) throw std::runtime_error("MCL::motionUpdate(): no particles initialized");

    for (auto& particle : particles_){
        double noise_rot1 = normal_dist(0.0, std::sqrt(variance.var_rot));
        double noise_rot2 = normal_dist(0.0, std::sqrt(variance.var_rot));
        double noise_trans = normal_dist(0.0, std::sqrt(variance.var_trans));

        double delta_rot1 = delta.rot1 + noise_rot1;
        double delta_rot2 = delta.rot2 + noise_rot2;
        double delta_trans = delta.trans + noise_trans;

        particle.x += delta_trans * std::cos(particle.theta + delta_rot1);
        particle.y += delta_trans * std::sin(particle.theta + delta_rot1);
        particle.theta += delta_rot1 + delta_rot2;

        normalizeAngle(&particle.theta);
    }
}

/**
 * @brief Compute likelihood of particle, i.e. possibility of observations if robot pose is current particle
 *        Using log likelihood to avoid numerical underflow and using addition instead of multiplikation
 */
void MCL::measurementUpdate(const std::vector<Landmark>& observations)
{
    // particle weights stay the same if no observations are made
    if (observations.empty()){
        return;
    }

    double weight_sum = 0.0;
    for (Particle& particle : particles_){
        double log_likelihood = 0.0;

        // calculate distance of observed landmark to true landmark
        for (const Landmark& observation : observations){
            Landmark lm_map = map_[observation.id];

            // transform landmark position from world frame (landmarks) to robot frame (observations)

            // translation of world frame to robot frame           
            double lm_x = lm_map.x - particle.x;
            double lm_y = lm_map.y - particle.y;

            // inverse rotation of world frame vector by particle.theta to robot frame
            // x_r = R(-theta) * x_w
            // R(theta) = (cos(theta) -sin(theta))
            //            (sin(theta)  cos(theta))
            double lm_x_r = std::cos(particle.theta) * lm_x + std::sin(particle.theta) * lm_y;
            double lm_y_r = -std::sin(particle.theta) * lm_x + std::cos(particle.theta) * lm_y;

            // using distance directly instead of std::exp(...) and then std::log(...)
            double lm_distance = -std::pow(distance(lm_x_r, lm_y_r, observation.x, observation.y), 2) / (2 * measurement_noise_variance);

            log_likelihood += lm_distance;
        }

        particle.weight = std::max(std::exp(log_likelihood), 1e-300);
        weight_sum += particle.weight;
    }

    // normalize weights
    if (weight_sum > 0){
        for (Particle& particle : particles_) {
            particle.weight /= weight_sum;
        }
    }    
}

void MCL::resampling(const int M)
{
    std::vector<double> particle_weights;
    particle_weights.clear();
    particle_weights.reserve(particles_.size());

    std::vector<Particle> resampled_particles;
    resampled_particles.clear();
    resampled_particles.reserve(M);

    for (const Particle& particle : particles_){
        particle_weights.push_back(particle.weight);
    }

    // lecture 6 slide 60
    // lay particle weights in order and create M uniform distributed marker that point at some weights
    // sample particles where the current marker points at

    double r = uniform_dist(0.0, 1.0 / M); // random start for marker
    double cumulative_weight = particle_weights[0]; // keep track of weights at current marker
    int j = 0;

    for (int i = 0; i < M; i++) {
        double marker_pos = r + static_cast<double>(i) / M; // increase marker position by 1/M
        
        // increase cumulative_weight until current marker position is reached to get to next particle weight
        while (marker_pos > cumulative_weight && j < particles_.size() - 1){
            j++;
            cumulative_weight += particle_weights[j];
        }

        Particle p = particles_[j];
        p.weight = 1 / M;
        resampled_particles.push_back(p);
    }

    particles_ = resampled_particles;
}

Pose MCL::poseEstimation()
{
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
    normalizeAngle(&theta_mean);

    return Pose {x_mean, y_mean, theta_mean};
}

// ----------------------------
// Helper Functions
// ----------------------------

void MCL::normalizeAngle(double* angle) {
    while (*angle > M_PI) *angle -= 2.0 * M_PI;
    while (*angle < -M_PI) *angle += 2.0 * M_PI;
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

double MCL::distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}
