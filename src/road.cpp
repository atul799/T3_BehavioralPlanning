#include <iostream>
#include "road.h"
#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>


/**
 * Initializes Road
 */
Road::Road(int speed_limit, double traffic_density, vector<int> lane_speeds) {

	this->num_lanes = lane_speeds.size();
	this->lane_speeds = lane_speeds;
	this->speed_limit = speed_limit;
	this->density = traffic_density;
	//camera_cntre --> visibility distance of camera to idenitfy vehicles
	//in adjacent lanes??
	this->camera_center = this->update_width/2;

}

Road::~Road() {}
//return Vehicle object for ego_key (vehicle/SDC)
//use of associate container and find method based on ego_key as key and
//then picking second which is Vehicle object (fist is key)
Vehicle Road::get_ego() {

	return this->vehicles.find(this->ego_key)->second;
}


//this function fills vehicles associative container
//for each lane add upto update_width number of vehicle objects
//and update vehicles_added to keep track of number of vehicles added.

void Road::populate_traffic() {
	//if camera center != 1/2 of update width, then the difference gets
	//picked as start position for adding vehicles (on s co-ordinate)
	//if 0 then start position is 0
	int start_s = max(this->camera_center - (this->update_width/2), 0);

	//for each lane
	for (int l = 0; l < this->num_lanes; l++)
	{
		//capture lane traffic speed
		int lane_speed = this->lane_speeds[l];
		//this var is not useful as it does nothing except for getting update
		//and not assigned to anything nor checked in conditionals
		bool vehicle_just_added = false;
		//add as many vehicles as distance between start_s(0 here) to update_width(70 here)
		//so for each lane there can be upto 70 vehicles added, depending on random number generated and normalized
		// has value < traffic density

		//loop for update_width number of times (start_s+this->update_width -start_s )
		for(int s = start_s; s < start_s+this->update_width; s++)
		{

			if(vehicle_just_added)
			{
				vehicle_just_added = false;
			}

			//if rand normal is < traffic density
			if(((double) rand() / (RAND_MAX)) < this->density)
			{
				//create a vehicle object in the current lane at s position s,
				//with lane speed of lane_speed and acceleration 0 (state is default "CS")
				Vehicle vehicle = Vehicle(l,s,lane_speed,0);

				//reset state at CS, it is default CS!!
				vehicle.state = "CS";
				//add to the count of vehicles being considered
				this->vehicles_added += 1;
				//insert this new vehicle object in vehicles hash
				this->vehicles.insert(std::pair<int,Vehicle>(vehicles_added,vehicle));
				vehicle_just_added = true;
			}
		}
	}

}

//generate predictions for non-ego vehicles which is a assoc cont. of int,Vehicles
//based on these predictions,find next state of go and advance the ego_vehicle
//
void Road::advance() {
	//associatetive container with key=int and Vehicles object vector
	map<int ,vector<Vehicle> > predictions;

	//iterator on int,vehicle associative container and assign to
	//vehicles assoc container's first key/vaue pair.
	map<int, Vehicle>::iterator it = this->vehicles.begin();

	//traverse vehicles assoc container
	while(it != this->vehicles.end())
	{
		//pick (key) index of iterator
		int v_id = it->first;
		//run generate_predictions method on vehicle object for this index
		//Generates predictions for non-ego vehicles to be used in
		//trajectory generation for the ego vehicle.
		//returns a vector of Vehicles object
		vector<Vehicle> preds = it->second.generate_predictions();
		//store vehicle objects in assoc container predictions at index (v_id)
		//of the vehicles assoc cont.
		predictions[v_id] = preds;
		it++;

		//AT END:for each vehicles at their key value there is going to be a vector of vehicle objects
	}

	//reset iterator to beginning of vehicles
	it = this->vehicles.begin();

	////traverse vehicles assoc container again
	while(it != this->vehicles.end())
	{
		//pick (key) index of iterator
		int v_id = it->first;
		//if key is for ego_vehicle
		if(v_id == ego_key)
		{
			//use predictions map and generate and return the best (lowest cost) trajectory
			vector<Vehicle> trajectory = it->second.choose_next_state(predictions);
			//Sets state and kinematics for ego vehicle using the last state of the trajectory.
			it->second.realize_next_state(trajectory);
		}
		else {

			//else move on, do nothing, can break the while loop when ego_vehicle is found, as there
			//is only 1 ego_vehicle
			it->second.increment(1);
		}
		it++;

		//AT END: for ego vehicle find next sate and transition to that state
	}

}

//create and add ego vehicle in vehicles map
void Road::add_ego(int lane_num, int s, vector<int> config_data) {

	//iterator on int,Vehicle associative container and assign to
	//vehicles assoc container's first key/vaue pair.
	map<int, Vehicle>::iterator it = this->vehicles.begin();

	//iterate till end of vehicles map
	while(it != this->vehicles.end())
	{
		//isolate key and value (Vehicle obj) from iterator
		int v_id = it->first;
		Vehicle v = it->second;

		//if current Vehicle's lane is same as lane_num passed
		//&& current Vehicle's s (position) is same as s passed

		//this is ego vehicle so don't want another key/value in vehicles map
		if(v.lane == lane_num && v.s == s)
		{
			//remove this key/value vehicles from map
			this->vehicles.erase(v_id);
		}
		it++;
	}

	Vehicle ego = Vehicle(lane_num, s, this->lane_speeds[lane_num], 0);

	//configure method called by simulator before simulation begins. Sets various
	//parameters which will impact the ego vehicle.
	ego.configure(config_data);


	ego.state = "KL";
	// add ego to vehicles map with ego_key=-1 (ego key set to -1 in constructor for Road class)
	this->vehicles.insert(std::pair<int,Vehicle>(ego_key,ego));

}


//display ego and other vehicles position
void Road::display(int timestep) {

	//find ego vehicle in vehicles map
	Vehicle ego = this->vehicles.find(this->ego_key)->second;

	// get position of ego
	int s = ego.s;
	//get state
	string state = ego.state;

	//update camera center then calculate s_min and max
	//smin= max of camera center-update_width/2 or 0
	//smax= s_min to update_width
	this->camera_center = max(s, this->update_width/2);
	int s_min = max(this->camera_center - this->update_width/2, 0);
	int s_max = s_min + this->update_width;


	//2d vector of strings
	vector<vector<string> > road;

	//loop all the way to update_width
	for(int i = 0; i < this->update_width; i++)
	{
		//store "  " for number of lanes in road_lane
		//and push it to roads vector
		//"   " * number of lanes for each update_width
		vector<string> road_lane;
		for(int ln = 0; ln < this->num_lanes; ln++)
		{
			road_lane.push_back("     ");
		}
		road.push_back(road_lane);

	}

	//iterator for vehicles
	map<int, Vehicle>::iterator it = this->vehicles.begin();

	//for each vehicle in vehicles
	while(it != this->vehicles.end())
	{

		int v_id = it->first;
		Vehicle v = it->second;

		//v.s is in target of smin and smax
		if(s_min <= v.s && v.s < s_max)
		{
			string marker = "";

			//if the ego vehicles
			//set marker to *** (this--> ego_rep)
			if(v_id == this->ego_key)
			{
				marker = this->ego_rep;
			}
			else
			{

				//for other vehicles
				//set marker to "(0)*xindex_in_vehicles_map"
				//number of x depends on length of key of vehicles
				//for 2-->length=1 marker 002
				//for 20 -->length=2 --> 020
				stringstream oss;
				stringstream buffer;
				buffer << " ";
				oss << v_id;
				for(int buffer_i = oss.str().length(); buffer_i < 3; buffer_i++)
				{
					buffer << "0";

				}
				buffer << oss.str() << " ";
				marker = buffer.str();
			}
			//add marker to rod 2d string array at position v.s-s_min and lane
			road[int(v.s - s_min)][int(v.lane)] = marker;
		}
		it++;
	}
	ostringstream oss;
	oss << "+Meters ======================+ step: " << timestep << endl;
	int i = s_min;
	for(int lj = 0; lj < road.size(); lj++)
	{
		if(i%20 ==0)
		{
			stringstream buffer;
			stringstream dis;
			dis << i;
			for(int buffer_i = dis.str().length(); buffer_i < 3; buffer_i++)
			{
				buffer << "0";
			}

			oss << buffer.str() << dis.str() << " - ";
		}
		else
		{
			oss << "      ";
		}
		i++;
		for(int li = 0; li < road[0].size(); li++)
		{
			oss << "|" << road[lj][li];
		}
		oss << "|";
		oss << "\n";
	}

	cout << oss.str();

}






