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

	// TODO: Set standard deviations for x, y, and theta
	 std_x = std[0];
	 std_y = std[1];
	 std_theta = std[2];
	 

	// This line creates a normal (Gaussian) distribution for x
	normal_distribution<double> dist_x(x, std_x);
	
	// TODO: Create normal distributions for y and theta
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	
	particles.clear();
	for (int i = 0; i < num_particles; ++i) {
		//double sample_x, sample_y, sample_theta;
		
		// TODO: Sample  and from these normal distrubtions like this: 
		//	 sample_x = dist_x(gen);
		//	 where "gen" is the random engine initialized earlier.
		
		P.x = dist_x(gen);
		P.y = dist_y(gen);
		P.theta = dist_theta(gen);
		P.weight = 1.0;
		particles.push_back(P);		
			
	// Print your samples to the terminal.
	//cout << "particles " << i + 1 << " " << particles[i].x << " " << particles[i].y << " " << particles[i].theta << endl;
	}
	is_initialized = 1;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/	
        
	default_random_engine gen;
	double std_x, std_y, std_theta; // Standard deviations for x, y, and theta

	// TODO: Set standard deviations for x, y, and theta
	 std_x = std_pos[0];
	 std_y = std_pos[1];
	 std_theta = std_pos[2];
	 
	for (int k = 0; k < (num_particles); ++k) {
		// This line creates a normal (Gaussian) distribution 
		normal_distribution<double> dist_x(particles[k].x, std_x);		
		normal_distribution<double> dist_y(particles[k].y, std_y);
		normal_distribution<double> dist_theta(particles[k].theta, std_theta);
				
		//particles[k].x = dist_x(gen);
		//particles[k].y = dist_y(gen);
		//particles[k].theta = dist_theta(gen);
		
		//particles[k].weight = 1.0;
			
	
	}
	 
	if(yaw_rate!=0){
		for (int i = 0; i < num_particles; ++i) {			
			particles[i].x = particles[i].x + (velocity/yaw_rate)*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta)); 
			particles[i].y = particles[i].y + (velocity/yaw_rate)*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t));
			particles[i].theta = particles[i].theta + yaw_rate*delta_t;
			if (particles[i].theta > M_PI) {
      				particles[i].theta = particles[i].theta - 2 * M_PI;
  			} else if (particles[i].theta < -M_PI) {
      				particles[i].theta = particles[i].theta + 2 * M_PI;
			}
			//if (big_weight > 1.0) {
			//	is_initialized = 0;
			//}
			 
			 // Print your samples to the terminal.
			 //cout << "Particle prediction " << i + 1 << " " << particles[i].x << " " << particles[i].y << " "<< particles[i].theta<<endl;
		}	
	}else{ 
		for (int i = 0; i < num_particles; ++i) {
			particles[i].x = velocity*delta_t*cos(particles[i].theta);
			particles[i].y = velocity*delta_t*sin(particles[i].theta);
			particles[i].theta = particles[i].theta;
			if (particles[i].theta > M_PI) {
      				particles[i].theta = particles[i].theta - 2 * M_PI;
  			} else if (particles[i].theta < -M_PI) {
      				particles[i].theta = particles[i].theta + 2 * M_PI;
			}
			//cout << "Particle predition yaw_rate is 0" << i + 1 << " " << particles[i].x << " " << particles[i].y << " "<< particles[i].theta<<endl;
		}			
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
	
	/*
	vector<LandmarkObs> predicted;
	
	for(int i = 0; i < 42; i++)
	{
		cout << "map" << i + 1 << " " << map_landmarks.landmark_list[i].id_i << " " << map_landmarks.landmark_list[i].x_f << " "<< map_landmarks.landmark_list[i].y_f<<endl;
	}


	predicted.clear();
	for(int i = 0; i < observations.size(); i++)
        {
       		LandmarkObs pred;
		pred.id = 0;
       		pred.x = observations[i].x;
		pred.y = observations[i].y;
		predicted.push_back(pred);		
		cout << "updateWeights observations" << i + 1 << " " << observations[i].id << " " << observations[i].x << " "<< observations[i].y<<endl;
       	}
	dataAssociation(predicted,predicted);
	*/
	vector<LandmarkObs> obs_map_pred;
	for (int k = 0; k < num_particles; ++k) {
	
		obs_map_pred.clear();
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
			for(int j = 0; j < 42; j++)
			{
				double distance;
				double distance_map;
				double distance_obs;
				double mu;
				double sigma;
				double prob;
				distance = sqrt( (map_landmarks.landmark_list[j].x_f - obs_map.x) * (map_landmarks.landmark_list[j].x_f - obs_map.x) + (map_landmarks.landmark_list[j].y_f - obs_map.y) * (map_landmarks.landmark_list[j].y_f - obs_map.y) );
			
				if(distance < 2.0){
					obs_map.id = map_landmarks.landmark_list[j].id_i;
					obs_map_pred.push_back(obs_map);
				
					distance_map = sqrt( (map_landmarks.landmark_list[j].x_f - particles[k].x) * (map_landmarks.landmark_list[j].x_f - particles[k].x) + (map_landmarks.landmark_list[j].y_f - particles[k].y) * (map_landmarks.landmark_list[j].y_f - particles[k].y) );

					distance_obs = sqrt( (particles[k].x - obs_map.x) * (particles[k].x - obs_map.x) + (particles[k].y - obs_map.y) * (particles[k].y - obs_map.y) );	
					//cout<<"distance_map   "<<distance_map<<endl;
					//cout<<"distance_obs   "<<distance_obs<<endl;	
				
					//prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
					//return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
					mu = distance_map;
					sigma = std_landmark[0] * std_landmark[1];
					prob = (exp(- ((mu - distance_obs) *(mu - distance_obs)) / (sigma *sigma) / 2.0))  /  sqrt(2.0 * M_PI * (sigma *sigma));
					//cout<<"prob   "<<prob<<endl;
					particles[k].weight *= prob; 

						
				//cout << "particles[k]" << " " << particles[k].x << " "<< particles[k].y<<  " "<< particles[k].theta << endl;
			
				//cout << "map" << j + 1 << " " << map_landmarks.landmark_list[j].id_i << " " << map_landmarks.landmark_list[j].x_f << " "<< map_landmarks.landmark_list[j].y_f<<endl;
				//cout << "obs_map" << "  " << obs_map.id << " " << obs_map.x << " "<< obs_map.y<<endl;
				//cout<<"distance"<<"  "<<distance<<endl;
				//cout << "obs_map_pred" << i + 1 << " " << obs_map_pred[i].id << " " << obs_map_pred[i].x << " "<< obs_map_pred[i].y<<endl;
				//cout<<"next observe"<<endl;
				} else {
					obs_map.id = 99;
				}	
			}
			//cout << "obs_map_pred" << " " << obs_map_pred[i].id << " " << obs_map_pred[i].x << " "<< obs_map_pred[i].y<<endl;
		
			//cout<<"end one line observe"<<endl;	
			//cout << "updateWeights observations" << i + 1 << " " << observations[i].id << " " << observations[i].x << " "<< observations[i].y<<endl;
	       	}
		//cout<<"particles[k].weight    "<< particles[k].weight<< endl;
		
		//cout<<"-------------end once time observe-----------------"<<endl;

	}
	
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	double total_weight = 0;
	double big_weight = 0;
	int    big_number = 0;
	for (int k = 0; k < num_particles; ++k) {
		total_weight += particles[k].weight;		
	}
	for (int k = 0; k < num_particles; ++k) {
		if(big_weight < particles[k].weight) {
			big_weight = particles[k].weight;
			big_number = k;
		}
	}		
	std::vector<Particle> particles_resample;
	particles_resample.clear();

	default_random_engine e;
        uniform_int_distribution<unsigned> u(0, 10);        
	int index;
	index = int((u(e)/10) * num_particles);
	double beta = 0.0;
	for (int k = 0; k < num_particles; ++k){
    		beta += (u(e)/10) * 2.0 * big_weight;
    		while( beta > particles[index].weight){
        		beta -= particles[index].weight;
        		index = (index + 1) % num_particles;
		}    
		particles_resample.push_back(particles[index]);
	}
	particles.clear();	
	particles.assign(particles_resample.begin(),particles_resample.end());
	for (int k = 0; k < num_particles; ++k){
		cout << "particle resample   " << k+1<<"  "<<particles[k].weight << endl;
	}    
	cout << "+++++++++++++++++++++++++++++++++++++++++++++++++   " <<  endl;
	cout << "biggest weight   " << big_weight << endl;
	//if (big_weight > 1.0) {
	//	is_initialized = 0;
	//}
        /*
	std::vector<Particle> particles_resample;
	particles_resample.clear();	        
	
	for (int k = 0; k < num_particles; ++k) {
		if( (particles[k].weight / total_weight) < 0.05) {
			particles_resample.push_back(particles[big_number]);
			cout << "add biggest" << endl;
		} else {
			particles_resample.push_back(particles[k]);
			cout << "keep it" << endl;			
		}
	}
	particles.clear();
	////v1.assign(v2.begin(), v2.end());
	particles.assign(particles_resample.begin(),particles_resample.end());
	*/
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
