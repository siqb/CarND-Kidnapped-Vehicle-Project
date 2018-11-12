/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    num_particles = 100;

    // Particles shall be implemented by sampling a Gaussian distribution, 
    // taking into account Gaussian sensor noise around the initial GPS position and heading estimates.
    // Use the C++ standard library normal distribution and C++ standard library 
    // random engine functions to sample positions around GPS measurements.

	//default_random_engine gen;
	double std_x, std_y, std_theta; // Standard deviations for x, y, and theta
	 
	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];
	
	// creates a normal (Gaussian) distribution for x.
	// Create normal distributions for y and theta.
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);
	
	for (int i = 0; i < num_particles; ++i) {
        struct Particle new_particle;
        new_particle.id = i;
        new_particle.x = dist_x(gen);
        new_particle.y = dist_y(gen);
        new_particle.theta = dist_theta(gen);
        new_particle.weight = 1.0;
        weights.push_back(1.0);
        particles.push_back(new_particle);
	}
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	//default_random_engine gen;
	double std_x, std_y, std_theta; // Standard deviations for x, y, and theta
	 
	std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = std_pos[2];
	
	// creates a normal (Gaussian) distribution for x.
	// Create normal distributions for y and theta.
	normal_distribution<double> dist_x(0, std_x);
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);

	for (int i = 0; i < num_particles; ++i) {
	    //normal_distribution<double> dist_x(particles[i].x,std_x);
	    //normal_distribution<double> dist_y(particles[i].y,std_y);
	    //normal_distribution<double> dist_theta(particles[i].theta,std_theta);
        // Add measurements
        if (fabs(yaw_rate) > 0.00001) {  
          particles[i].x += (velocity/yaw_rate)*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
          particles[i].y += (velocity/yaw_rate)*(cos(particles[i].theta) - cos(particles[i].theta+yaw_rate*delta_t));
          particles[i].theta += yaw_rate*delta_t;
        }
        else { 
          particles[i].x += velocity * delta_t * cos(particles[i].theta);
          particles[i].y += velocity * delta_t * sin(particles[i].theta);
        } 
        // Add noise
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
    }

}

// I have no idea why my implementation of dataAssociation causes
// max error to be exceeded...the code looks perfect!
// It is probably a C++ thing since I'm trying to use C++11 features.
// It was driving me crazy so I had to borrow this function and comment mine out.
// As far as I can tell though, it looks functionally the same as mine!

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

	for(unsigned int i=0; i<observations.size(); i++){
	  // Initialize the containers of minimum distance and ID
      double min_distance = 99999999;
      int closest_ID = -1;
	  double observe_x = observations[i].x;
	  double observe_y = observations[i].y;
      for(unsigned int j=0; j<predicted.size(); j++){
	    double predict_x = predicted[j].x;
	    double predict_y = predicted[j].y;
	    double distance = dist(observe_x, observe_y, predict_x, predict_y);
        if(distance < min_distance){
          min_distance = distance;
          closest_ID = j;
        }
      }
    observations[i].id = closest_ID;
  }
}
//void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
//	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
//	//   observed measurement to this particular landmark.
//	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
//	//   implement this method and use it as a helper during the updateWeights phase.
//
//    for (auto& obs:observations){
//      int idx = 0;
//      int min_id = -1;
//      double min_dist = numeric_limits<double>::max();
//      for (auto const & pred:predicted){
//        double curr_dist = dist(obs.x,obs.y,pred.x,pred.y);
//        if (curr_dist < min_dist){
//            min_dist = curr_dist;
//            min_id = idx++;
//        }
//      }
//      obs.id = min_id;
//    }
//}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
