/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Original Author: Tiffany Huang
 
 * ParticleFilter methods filled in and authored by Ben Blazado as part of the 
 * Kidnapped Vehicle project in fulfillment of the requirements 
 * of Udacity's Self-Driving Car Engineer Nanodegree Program.
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

using std::normal_distribution;
std::default_random_engine Gen;

/**
 * Indices into std dev arrays
 */
#define X     0
#define Y     1
#define THETA 2

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
   
  num_particles = 1000;  // TODO: Set the number of particles
  particles.resize(num_particles);
  weights.resize(particles.size());
  
  normal_distribution<double> dist_x(x, std[X]);
  normal_distribution<double> dist_y(y, std[Y]);
  normal_distribution<double> dist_theta(theta, std[THETA]);
   
  for (int i = 0; i < particles.size(); i++) {
    Particle& p = particles[i];
    p.id       = i;
    p.x        = dist_x(Gen);
    p.y        = dist_y(Gen);
    p.theta    = dist_theta(Gen);
    p.weight   = 1.0;
    weights[i] = p.weight;
  }

  is_initialized = true;
}

/**
 * setState sets the state of particle p given predicted state values
 *   and corresponding std devs; used in predictionYawZero and 
 *   predictionYawNonZero.
 */
void setState (double pred_x, double pred_y, double pred_theta, 
    double std_pos[], Particle& p) {
  
  normal_distribution<double> dist_x    (pred_x, std_pos[X]);
  normal_distribution<double> dist_y    (pred_y, std_pos[Y]);
  normal_distribution<double> dist_theta(pred_theta, std_pos[THETA]);
  
  p.x     = dist_x(Gen);
  p.y     = dist_y(Gen);
  p.theta = dist_theta(Gen);
}

/**
 * predictionYawZero Predicts the state for the next time step
 *   using the process model without yaw; used in 
 *   ParticleFilter::prediction.
 */
void predictionYawZero(double delta_t, double std_pos[], double velocity, 
    std::vector<Particle>& particles) {
          
  double dist = velocity * delta_t;

  for (int i = 0; i < particles.size(); i ++) {

    Particle& p = particles[i];
    
    double pred_x = p.x + dist * cos(p.theta);
    double pred_y = p.y + dist * sin(p.theta);
    
    setState(pred_x, pred_y, p.theta, std_pos, p);
  }
}

/**
 * predictionYawZero Predicts the state for the next time step
 *   using the process model with yaw; used in
 *   ParticleFilter::prediction.
 */
void predictionYawNonZero(double delta_t, double std_pos[], double velocity, 
      double yaw_rate, std::vector<Particle>& particles)  {
        
  double v_per_yaw_rate = velocity / yaw_rate;
  
  for (int i = 0; i < particles.size(); i ++) {

    Particle& p = particles[i];
    
    double pred_theta = p.theta + yaw_rate*delta_t;
    double pred_x = p.x + v_per_yaw_rate * (sin(pred_theta) - sin(p.theta));
    double pred_y = p.y + v_per_yaw_rate * (cos(p.theta) - cos(pred_theta));
    
    setState(pred_x, pred_y, pred_theta, std_pos, p);
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  double pred_theta, pred_x, pred_y;
  
  void (*pred_component) (double velocity, double theta, double pred_theta,
        double& x_comp, double& y_comp);
        
  if (yaw_rate == 0)
    predictionYawZero(delta_t, std_pos, velocity, particles);
  else
    predictionYawNonZero(delta_t, std_pos, velocity, yaw_rate, particles);
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  for (int i = 0; i < observations.size(); i ++) {
    
    LandmarkObs& o = observations[i];
    double lowest = INFINITY;
    int id_closest;
    
    for (int j = 0; j < predicted.size(); j ++) {
      
      LandmarkObs& p = predicted[j];
      double d = dist(o.x, o.y, p.x, p.y);
      if (d < lowest) {
        id_closest = j;
        lowest = d;
      }
    }
    
    // save index of closest landmark to id field of observation
    // the closest landmark to an obsevation o is now found by:
    // closest_landmark = landmark[o.id]
    o.id = id_closest;
  }
}

/**
 * selectInRange saves landmarks from map_landmarks that
 *   are in sensor_range of Particle p to predicted_landmarks;
 *   used in ParticleFilter::updateWeights.
 */
void selectInRange(const Particle& p, double sensor_range, 
                   const Map &map_landmarks, 
                   vector<LandmarkObs> &predicted_landmarks) {
  
  for (int i = 0; i < map_landmarks.landmark_list.size(); i ++) {
    
    Map::single_landmark_s l = map_landmarks.landmark_list[i];
    double d = dist(p.x, p.y, l.x_f, l.y_f);
    
    if (d <= sensor_range) {
      // landmark l is in sensor range of particle p
      LandmarkObs landmark_inrange;
      landmark_inrange.id = l.id_i;
      landmark_inrange.x  = l.x_f;
      landmark_inrange.y  = l.y_f;
      predicted_landmarks.push_back(landmark_inrange);
    }
  }
  
}

/**
 * transformToMap transforms observations from coordinates
 *   relative to Particle p to map coordinates in map_observations;
 *   used in ParticleFilter::updateWeights.
 */
void transformToMap(const Particle &p, 
                    const vector<LandmarkObs> &observations, 
                    vector<LandmarkObs> &map_observations) {

  for (int i = 0; i < observations.size(); i ++) {
    
    LandmarkObs o = observations[i];
    
    double cos_theta = cos(p.theta);
    double sin_theta = sin(p.theta);
    double x_map = p.x + o.x*cos_theta - o.y*sin_theta;
    double y_map = p.y + o.x*sin_theta + o.y*cos_theta;
    
    LandmarkObs map_observation;
    map_observation.x  = x_map;
    map_observation.y  = y_map;
    map_observations.push_back(map_observation);
    
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
   
  // calculate normlizer used for mult-variate gaussian calculations
  double std_x = std_landmark[0];
  double std_y = std_landmark[1];
  double normalizer = 2 * M_PI * std_x * std_y;
   
  for (int i = 0; i < particles.size(); i ++) {
    
    Particle& p = particles[i];
    
    // select landmarks in sensor range of particle p
    vector<LandmarkObs> map_predicted_landmarks;
    selectInRange(p, sensor_range, map_landmarks, map_predicted_landmarks);
  
    // transform observed landmarks from vehicle to map coordinates
    vector<LandmarkObs> map_observations;
    transformToMap(p, observations, map_observations);
    
    // associate nearest predicted landmark to observed landmark
    dataAssociation(map_predicted_landmarks, map_observations);
    
    // set up variables to record landmarks associated with particle p
    vector<int> associations;   // landmark id associated with particle p
    vector<double> sense_x;     // observed x coordinate of landmark
    vector<double> sense_y;     // observed y coordinate of landmark

    // calculate particle weight
    p.weight = 1.0;
    
    for (int j = 0; j < map_observations.size(); j ++) {
      
      LandmarkObs map_o = map_observations[j];
      // map_o.id is the id of the closest predicted_landmark
      LandmarkObs map_closest = map_predicted_landmarks[map_o.id];
      
      // calculate weight of particle using mult-variate gaussian
      double dx = (map_o.x - map_closest.x);
      double dy = (map_o.y - map_closest.y);
      double e = exp(-((dx*dx)/(2*std_x*std_x) + (dy*dy)/(2*std_y*std_y)));
      
      p.weight *= (e / normalizer);
      
      // record closest landmark as associated with particle p
      associations.push_back(map_closest.id);  // save id of landmark
      sense_x.push_back(map_o.x);              // save observed x coordinate
      sense_y.push_back(map_o.y);              // save observed y coordinate
    }
    
    SetAssociations(p, associations, sense_x, sense_y);
    
    // vector of weights will be used in 
    // discrete_distribution in ParticleFilter::resample
    weights[i] = p.weight;
    
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  // setup discrete distribution using the calculated weights
  std::discrete_distribution<int> dist(weights.begin(), weights.end());
  
  // vector of particles selected from sample
  std::vector<Particle> selected;
  
  // select particles from sample according to discrete_distribution dist
  // this is the part the filters out particles
  for (int n = 0; n < particles.size(); n ++) {
    
    int i = dist(Gen);
    selected.push_back(particles[i]);
  }
  
  // clear out existing set of particles
  particles.clear();
  
  // save selected to particles for use in next iteration
  particles.assign(selected.begin(), selected.end());
  
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}