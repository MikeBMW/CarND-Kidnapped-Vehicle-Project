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

	Particle P;
	num_particles = 100;
	default_random_engine gen;
	double std_x, std_y, std_theta; // Standard deviations for x, y, and theta
	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];	
	
	// This line creates a normal (Gaussian) distribution 
	normal_distribution<double> dist_x(x, std_x);	
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);
	
	particles.clear();
	for (int i = 0; i < num_particles; ++i) {		
		P.x = dist_x(gen);
		P.y = dist_y(gen);
		P.theta = dist_theta(gen);
		P.weight = 1.0;
		particles.push_back(P);		
	}
	is_initialized = 1;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/	
        
	default_random_engine gen;
	
	for (int i = 0; i < num_particles; i++)
	{
		
		double new_x;
		double new_y;
		double new_theta;
		
		if (yaw_rate == 0)
		{
			new_x = particles[i].x + velocity*delta_t*cos(particles[i].theta);
			new_y = particles[i].y + velocity*delta_t*sin(particles[i].theta);
			new_theta = particles[i].theta;
		}
		else
		{
			new_x = particles[i].x + velocity/yaw_rate*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta));
			new_y = particles[i].y + velocity/yaw_rate*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t));
			new_theta = particles[i].theta + yaw_rate*delta_t;
		}

		
		normal_distribution<double> N_x(new_x, std_pos[0]);
		normal_distribution<double> N_y(new_y, std_pos[1]);
		normal_distribution<double> N_theta(new_theta, std_pos[2]);
		
		particles[i].x = N_x(gen);
		particles[i].y = N_y(gen);
		particles[i].theta = N_theta(gen);
		
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	for(int i = 0; i < predicted.size(); i++)
        {       		
		//cout << "dataAssociation predicted" << i + 1 << " " << predicted[i].id << " " << predicted[i].x << " "<< predicted[i].y<<endl;
       	}
}

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
	
	vector<LandmarkObs> obs_map_pred;
	for (int k = 0; k < num_particles; ++k) {
	
		obs_map_pred.clear();
		particles[k].associations.clear();
		particles[k].sense_x.clear();
		particles[k].sense_y.clear();
		//cout<<"----------start once time observe----------------"<<endl;
		particles[k].weight = 1.0;
		for(int i = 0; i < observations.size(); i++)
		{
			//cout<<"start one line observe"<<endl;       		
			LandmarkObs obs;
			LandmarkObs obs_map;
			obs.id = i;
	       		obs.x = observations[i].x;
			obs.y = observations[i].y;
			//cout << "obs before transform" << "  " << obs.id << " " << obs.x << " "<< obs.y<<endl;
			//cout << "particles[k].y" << "  " << particles[k].y <<endl;
		
			//transform to map coordinate
			//xm​=xp​+(cosθ×xc​)−(sinθ×yc​)
			//ym=yp+(sinθ×xc)+(cosθ×yc)
			obs_map.x = particles[k].x + (obs.x) * cos(particles[k].theta) - (obs.y) * sin(particles[k].theta);
			obs_map.y = particles[k].y + (obs.x) * sin(particles[k].theta) + (obs.y) * cos(particles[k].theta); 
			//cout << "obs_map after transform" << "  " << obs_map.id << " " << obs_map.x << " "<< obs_map.y<<endl;			
		
			//calculate distance
			double closet_dis = sensor_range;
			double closet_dis_id ;
			for(int j = 0; j < map_landmarks.landmark_list.size(); j++)
			{
				double calc_dist;
				double distance_map;
				double distance_obs;
				double mu;
				double sigma;
				double prob;				

				calc_dist = sqrt( (map_landmarks.landmark_list[j].x_f - obs_map.x) * (map_landmarks.landmark_list[j].x_f - obs_map.x) + (map_landmarks.landmark_list[j].y_f - obs_map.y) * (map_landmarks.landmark_list[j].y_f - obs_map.y) );			
				
				if(calc_dist < closet_dis)
				{
					closet_dis = calc_dist;
					closet_dis_id = j;	
				} 
			}					
			obs_map.id = map_landmarks.landmark_list[closet_dis_id].id_i;
			obs_map_pred.push_back(obs_map);
			particles[k].associations.push_back(obs_map.id);			
			particles[k].sense_x.push_back(obs_map.x);
			particles[k].sense_y.push_back(obs_map.y);			 

			double meas_x = obs_map.x;
			double meas_y = obs_map.y;
			double mu_x = map_landmarks.landmark_list[closet_dis_id].x_f;
			double mu_y = map_landmarks.landmark_list[closet_dis_id].y_f;
		
			long double multipler = 1/(2*M_PI*std_landmark[0]*std_landmark[1])*exp(-( pow(meas_x-mu_x,2.0) / (2*pow(std_landmark[0],2.0))+pow(meas_y-mu_y,2.0)/ (2*pow(std_landmark[1],2.0)) ) );
			if(multipler > 0)
			{
				particles[k].weight*=multipler;
			}
	       	}		
		//cout<<"-------------end once time observe,one particle-----------------"<<endl;

	}
	
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution	
	
	vector<double> weights;
        weights.clear();
	for (int i = 0; i < num_particles; i++)
	{	
		
		weights.push_back(particles[i].weight);
	}
	//for (int k = 0; k < num_particles; ++k){
    	//	cout<<"old particle weight    "<<k<<"   "<<particles[k].weight<<endl;
	//}
	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(), weights.end());
	//discrete_distribution<int> distribution(weights[0], weights[num_particles]);
	
	vector<Particle> resample_particles;

	
	for (int i = 0; i < num_particles; i++)
	{
		
		resample_particles.push_back(particles[distribution(gen)]);
	}
	
	particles = resample_particles;
	//for (int k = 0; k < num_particles; ++k){
    	//	cout<<"new particle weight    "<<k<<"   "<<particles[k].weight<<endl;
	//}

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
