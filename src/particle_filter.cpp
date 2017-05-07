#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

/**
 * Pseudo code --> 
 *
 * ParticleFilter pf;
 *
 * foreach(time_step) {
 *   if (time_step == 0)
 *     pf.init();
 *   pf.prediction();
 *   pf.updateWeights();
 *   pf.resample();
 *   pf.calcError();
 * }
 */

/**
 * init Initializes particle filter by initializing particles to Gaussian
 *   distribution around first position and all the weights to 1.
 * @param x Initial x position [m] (simulated estimate from GPS)
 * @param y Initial y position [m]
 * @param theta Initial orientation [rad]
 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
 *   standard deviation of yaw [rad]]
 */

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  std::cout << "Init particle filter (Initial belief) ..." << std::endl;
  
  // Set the number of particles.
  num_particles = 1000;
  
  // Set all weights to 1.
  double p_init_weight = 1.0;
  
  // generate a gaussian distribution of x,y and theta values (http://en.cppreference.com/w/cpp/numeric/random/normal_distribution)
  std::normal_distribution<double> x_norm_d     (/* mean */ x,     /* stddev */ std[0]);
  std::normal_distribution<double> y_norm_d     (/* mean */ y,     /* stddev */ std[1]);
  std::normal_distribution<double> theta_norm_d (/* mean */ theta, /* stddev */ std[2]);
  
  // pseudo random number generator
  std::random_device rd;
  std::mt19937 gen(rd());     // http://en.cppreference.com/w/cpp/numeric/random/mersenne_twister_engine
                              // or alternatively http://www.cplusplus.com/reference/random/default_random_engine/
  
  for(int p_id = 0; p_id < num_particles; p_id++) {
    // Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1.
    // and add random Gaussian noise to each particle. i.e: Particle position values x, y, theta are picked randomly from a
    // normal (gaussian) distribution centered (mean) around their initial position value and the given std deviations
    
    Particle p = {
      p_id,                     // particle unique id
      p_init_weight,            // initial weight
      x_norm_d(gen),            // particles x value (vehicles init x position estimate based on GPS + noise)
      y_norm_d(gen),            // particles y value (vehicles init y position estimate based on GPS + noise)
      theta_norm_d(gen)         // particles theta value (vehicles init theta estimate + noise)
    };
    
    weights.push_back(p_init_weight);
    particles.push_back(p);
  }
  
  is_initialized = true;
}

/**
 * prediction Predicts the state for the next time step
 *   using the process model.
 * @param delta_t Time between time step t and t+1 in measurements [s]
 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
 *   standard deviation of yaw [rad]]
 * @param velocity Velocity of car from t to t+1 [m/s]
 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
 */

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO
}

/**
 * dataAssociation Finds which observations correspond to which landmarks (likely by using
 *   a nearest-neighbors data association).
 * @param predicted Vector of predicted landmark observations
 * @param observations Vector of landmark observations
 */

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // TODO
}


/**
 * updateWeights Updates the weights for each particle based on the likelihood of the
 *   observed measurements.
 * @param sensor_range Range [m] of sensor
 * @param std_landmark[] Array of dimension 2 [standard deviation of range [m],
 *   standard deviation of bearing [rad]]
 * @param observations Vector of landmark observations
 * @param map Map class containing map landmarks
 */

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations, Map map_landmarks) {
  // TODO
}

/**
 * resample Resamples from the updated set of particles to form
 *   the new set of particles.
 */

void ParticleFilter::resample() {
  // TODO
}

/*
 * write Writes particle positions to a file.
 * @param filename File to write particle positions to.
 */

void ParticleFilter::write(std::string filename) {
  std::ofstream dataFile;
  dataFile.open(filename, std::ios::app);
  for (int i = 0; i < num_particles; ++i) {
    dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
  }
  dataFile.close();
}
