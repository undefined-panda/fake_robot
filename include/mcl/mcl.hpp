#ifndef MONTE_CARLO_LOCALIZATION_HPP
#define MONTE_CARLO_LOCALIZATION_HPP

#include <vector>
#include <random>
#include <array>
#include <unordered_map>
#include <optional>

/**
 * @brief Particle struct with state[x, y, theta] and weight
 */
struct Particle
{
  double x;
  double y;
  double theta;
  double weight;
};

/**
 * @brief Landmark struct with landmark id and coordinates.
 */
struct Landmark {
  int id;
  double x;
  double y;
};

struct MotionDelta {
  double rot1;
  double trans;
  double rot2;
};

struct MotionNoise {
  double var_rot;
  double var_trans;
};

struct Pose {
  double x;
  double y;
  double theta;
};

/**
 * @brief Monte Carlo Localization algorithm
 */
class MCL
{
public:
    MCL();
    ~MCL();

    /**
     * @brief Set map information
     * @param landmarks Map of landmark_id -> {x, y}
     */
    void setMap(const std::vector<Landmark>& landmarks);

    /**
     * @brief Initialie num_particles particles.
     */
    void initializeParticles();

    /**
     * @brief Return particles.
     */
    const std::vector<Particle>& getParticles() const;

    /**
     * @param delta difference to previous measurement, [delta_rot1, delta_trans, delta_rot2]
     * @param variance variance for particle, [var_rot, var_trans]
     */
    void motionUpdate(MotionDelta delta, MotionNoise variance);

    void measurementUpdate(const std::vector<Landmark>& observations);

    void resampling();

    std::optional<Pose> poseEstimation();

    double normalizeAngle(double angle);

    // ----------------------------
    // Attributes 
    // ----------------------------

    double measurement_noise_variance;
    int num_particles;

private:
    // ----------------------------
    // Helper Functions 
    // ----------------------------

    /**
     * @brief Uniform distribution sample with boundaries.
     * @param min lower boundary
     * @param max upper boundary
     */
    double uniform_dist(double min, double max);

    double normal_dist(double mean, double cov);

    double distance(double x1, double y1, double x2, double y2);

    double num_effective_particles();

    // ----------------------------
    // Attributes 
    // ----------------------------

    std::unordered_map<int, Landmark> map_; // store it as unordered map to access landmarks using their ID as index
    bool map_initialized_ = false;
    bool particles_initialized_ = false;
    std::array<double, 4> bounds_;

    std::vector<Particle> particles_;

    std::mt19937 rng_;
};

#endif  // MONTE_CARLO_LOCALIZATION_HPP
