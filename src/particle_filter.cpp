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
  num_particles = 100;
  
  // Set all weights to 1.
  double p_init_weight = 1.0;     // max probability/max confusion
  
  // generate a gaussian distribution of x,y and theta values (http://en.cppreference.com/w/cpp/numeric/random/normal_distribution)
  std::normal_distribution<double> x_norm_d     (/* mean */ x,     /* stddev */ std[0]);
  std::normal_distribution<double> y_norm_d     (/* mean */ y,     /* stddev */ std[1]);
  std::normal_distribution<double> theta_norm_d (/* mean */ theta, /* stddev */ std[2]);
  
  // pseudo random number generator
  std::random_device rd;
  rand_generator = std::mt19937(rd());     // http://en.cppreference.com/w/cpp/numeric/random/mersenne_twister_engine
                                           // or alternatively http://www.cplusplus.com/reference/random/default_random_engine/
  
  for(int p_id = 0; p_id < num_particles; p_id++) {
    // Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1.
    // and add random Gaussian noise to each particle. i.e: Particle position values x, y, theta are picked randomly from a
    // normal (gaussian) distribution centered (mean) around their initial position value and the given std deviations
    
    Particle p = {
      p_id,                           // particle unique id
      x_norm_d(rand_generator),       // particles x value (vehicles init x position estimate based on GPS + noise)
      y_norm_d(rand_generator),       // particles y value (vehicles init y position estimate based on GPS + noise)
      theta_norm_d(rand_generator),   // particles theta value (vehicles init theta estimate + noise)
      p_init_weight                   // initial weight
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

void ParticleFilter::prediction(double delta_t, double std_pos[] /* x [m], y [m], yaw [rad] */, double velocity /* velocity [m/s] */, double yawd /* yaw_rate [rad/s] */) {
  //std::cout << "Prediction step (delta_t: " << delta_t << ")" << std::endl;
  
  // Predict the particles new position after delta_t seconds given the yaw rate (yawd) and velocity measurements.
  for(int p_id = 0; p_id < num_particles; p_id++) {
    Particle& p = particles[p_id];
    
    // extract values for better readability
    double p_x = p.x;
    double p_y = p.y;
    double yaw = p.theta;
    
    // predicted state values
    double px_p, py_p, v_p, yaw_p, yawd_p;
    
    // avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + velocity/yawd * ( sin(yaw + yawd*delta_t) - sin(yaw) );
      py_p = p_y + velocity/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
      px_p = p_x + velocity*delta_t*cos(yaw);
      py_p = p_y + velocity*delta_t*sin(yaw);
    }
    
    v_p = velocity;
    yaw_p = yaw + yawd * delta_t;
    yawd_p = yawd;
    
    // update the particle with the predicted value + noise
    p.x       = noisy(px_p, std_pos[0]);
    p.y       = noisy(py_p, std_pos[1]);
    p.theta   = noisy(yaw_p, std_pos[2]);
  }
}

double ParticleFilter::noisy(double mean, double stdev) {
  return (std::normal_distribution<double>(mean, stdev))(rand_generator);
}

/**
 * carToMapCoord Converts from car coordinate system to global map coordinate system
 * @param particle Particle (of the particle filter) relative to our observations
 * @param observations Vector of landmark observations
 * @returns Vector of landmark observations converted to map coordinate space
 */

std::vector<LandmarkObs> ParticleFilter::carToMapCoord(const Particle particle, std::vector<LandmarkObs>& observations) {
  // convert observed landmark in car's coordinate system to map coordinate
  std::vector<LandmarkObs> observations_conv = std::vector<LandmarkObs>();
  
  for( LandmarkObs& observation : observations ) {
    // x_o = x_p + x_o' * cos(θ) - y_o' * sin(θ)
    // y_o = y_p + x_o' * sin(θ) + y_o' * cos(θ)
    
    // where:
    //  x_o  = observed x coordinate (in map coordinate system)
    //  x_o' = observed x coordinate (in car coordinate system)
    //  y_o  = observed y coordinate (in map coordinate system)
    //  y_o' = observed y coordinate (in car coordinate system)
    //  x_p  = particle x position (in map coordinate system)
    //  y_p  = particle x position (in map coordinate system)
    //  θ (theta) = yaw angle
    
    // expand vars for readability ....
    double x_p = particle.x;
    double y_p = particle.y;
    double θ = particle.theta;
    
    // car to map converstion ...
    double x_o = x_p + observation.x * cos(θ) - observation.y * sin(θ);
    double y_o = y_p + observation.x * sin(θ) + observation.y * cos(θ);
    
    LandmarkObs o_conv = {
      observation.id,
      x_o,
      y_o
    };
    
    observations_conv.push_back(o_conv);
  }
  
  return observations_conv;
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

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations, Map map_landmarks)
{
  for( int p_idx = 0; p_idx < particles.size(); p_idx++ ) {
    Particle& p = particles[p_idx];
    
    // convert observed landmark in car's coordinate system to map coordinate
    std::vector<LandmarkObs> observations_conv = carToMapCoord(p, observations);
    
    // data association
    std::map<int, LandmarkDataAssoc> dataAssoc = dataAssociation(sensor_range, map_landmarks, observations_conv);
    //std::cout << "obs size: " << observations_conv.size() << " landmarks: " << map_landmarks.landmark_list.size() << " dataAssoc: " << dataAssoc.size() << std::endl;
    
    // update weight
    double weight = 1.0;
    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];
    double sigma_x_pow2 = pow(sigma_x, 2);
    double sigma_y_pow2 = pow(sigma_y, 2);
    double C = 1.0 / (2.0 * M_PI * sigma_x * sigma_y);
    
    //std::cout << " C = " << C << std::endl;
    
    for( LandmarkObs& observation : observations_conv )
    {
      if( !dataAssoc[observation.id].isInSensorRange ) {
        //std::cout << "dataAssoc[observation.id].isInSensorRange = " << dataAssoc[observation.id].isInSensorRange << std::endl;
        continue;
      }
      
      // observation coords ...
      double x_mu = observation.x;
      double y_mu = observation.y;
      
      // associated closest landmark coords ...
      Map::single_landmark_s closest_map_landmark = dataAssoc[observation.id].map_landmark;

      double x = closest_map_landmark.x_f;
      double y = closest_map_landmark.y_f;
      
      double x_diff = x - x_mu;
      double y_diff = y - y_mu;
      
      double e_raised = ((x_diff * x_diff) / (2.0 * sigma_x_pow2)) + ((y_diff * y_diff) / (2.0 * sigma_y_pow2));
      weight *= C * exp(-e_raised);
    }
    
    //std::cout << " weight before: " << p.weight << " after:  " << weight << std::endl;
    p.weight = weight;
    weights[p_idx] = weight;
  }
}

/**
 * dataAssociation Finds which observations correspond to which landmarks (likely by using
 *   a nearest-neighbors data association).
 * @param predicted Vector of predicted landmark observations
 * @param observations Vector of landmark observations
 */

std::map<int, LandmarkDataAssoc> ParticleFilter::dataAssociation(double sensor_range, Map& map_landmarks, std::vector<LandmarkObs>& observations) {
  // for each observation ...
  // for each landmark ...
  //  calc the euclidean distance vector/distance between observations and landmarks
  //  if the map landmark is within senor range *and* the map landmark is closest to the observed landmark
  //  then assoc map landmark to observed landmark
  
  std::vector<Map::single_landmark_s> landmark_list = map_landmarks.landmark_list;
  std::map<int, LandmarkDataAssoc> dataAssoc = std::map<int, LandmarkDataAssoc> ();
  
  for( LandmarkObs& obs : observations ) {
    double min_distance = -1;
    
    for( Map::single_landmark_s map_landmark : landmark_list ) {
      double distance = sqrt(pow(obs.x - map_landmark.x_f, 2) + pow(obs.y - map_landmark.y_f, 2));      
      bool isInSensorRange = sensor_range >= distance;
      
      if( min_distance == -1 || distance < min_distance ) {
        //std::cout << "map_landmark.id = " << map_landmark.id_i << " distance = " << distance << std::endl;
        LandmarkDataAssoc assoc = {
            map_landmark,
            obs,
            distance,
            isInSensorRange
        };
        
        min_distance = distance;
        obs.id = map_landmark.id_i;
        dataAssoc[map_landmark.id_i] = assoc;
      }
    }
  }
  
  return dataAssoc;
}


/**
 * resample Resamples from the updated set of particles to form
 *   the new set of particles.
 */

void ParticleFilter::resample() {
  std::discrete_distribution<> discrete_distribution(weights.begin(), weights.end());
  std::vector<Particle> particles_weighted_resample;
  
  for (int p_idx = 0; p_idx < num_particles; p_idx++) {
    int weighted_pick = discrete_distribution(rand_generator);
    particles_weighted_resample.push_back(particles[weighted_pick]);
  }
  
  // Set the internal list of particles to the re-sampled ones
  particles = particles_weighted_resample;
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
