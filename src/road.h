#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"

using namespace std;

class Road {
public:

	//looks like it is span of s distance in which vehicles in lanes are going to be considered
	//for cost calculation
	//vehicle width (or lane width??)
	int update_width = 70;

	string ego_rep = " *** ";

	//vehicle number index
	int ego_key = -1;

	//number of lanes
	int num_lanes;

	//traffic speed in each lane
	vector<int> lane_speeds;

	//speed limit
	int speed_limit;

	//traffic density
	double density;

	//center of vehicle length (In constructor it is set at = update_width/2)
	int camera_center;


	//associative container with vehicle index and vehicle object associated to the vehicle object)
	//keep track of multiple objects (Vehicles in this case)
	//ego_key is the index used as key for this container
	map<int, Vehicle> vehicles;

	//number of vehicles
	int vehicles_added = 0;

	/**
	 * Constructor
	 */
	Road(int speed_limit, double traffic_density, vector<int> lane_speeds);

	/**
	 * Destructor
	 */
	virtual ~Road();

	//return the Vehicle object for ego_key value, to query members of vehicle object
	Vehicle get_ego();

	////this function fills vehicles associative container
	//for each lane add upto update_width number of vehicle objects
	//and update vehicles_added to keep track of number of vehicles added.

	void populate_traffic();

	//
	void advance();

	void display(int timestep);

	void add_ego(int lane_num, int s, vector<int> config_data);

	void cull();

};
