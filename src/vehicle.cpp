#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"

/**
 * Initializes Vehicle
 */
//constructor overloaded
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s, float v, float a, string state) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    max_acceleration = -1;

}

Vehicle::~Vehicle() {}

////////////My implementation based on pseudocode an more efficient one from solution is below
vector<Vehicle> Vehicle::choose_next_state_mine(map<int, vector<Vehicle>> predictions) {

	//transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights)
	/*

    ***Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept.***

    INPUT: A predictions map. This is a map using vehicle id as keys with predicted
        vehicle trajectories as values. A trajectory is a vector of Vehicle objects. The first
        item in the trajectory represents the vehicle at the current timestep. The second item in
        the trajectory represents the vehicle one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory for the ego vehicle corresponding to the next ego vehicle state.

    Functions that will be useful:
    1. successor_states() - Uses the current state to return a vector of possible successor states for the finite
       state machine.
    2. generate_trajectory(string state, map<int, vector<Vehicle>> predictions) - Returns a vector of Vehicle objects
       representing a vehicle trajectory, given a state and predictions. Note that trajectory vectors
       might have size 0 if no possible trajectory exists for the state.
    3. calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory) - Included from
       cost.cpp, computes the cost for a trajectory.
    */

    //Your solution here.
	//////////////////////////////////
	//only consider states which can be reached from current FSM state.
	vector<string> possible_successor_states = successor_states();
	//keep track of the total cost of each state.
	vector<float> costs;
	//Vehicle vehicle=predictions;
	//Vehicle vehicle={Vehicle(this->lane, this->s, this->v, this->a, this->state)};
	for(int i=0;i < possible_successor_states.size();i++){


		//generate a rough idea of what trajectory we would follow IF we chose this state.
		vector<Vehicle> trajectory_for_state = generate_trajectory(possible_successor_states[i], predictions);

		//calculate the "cost" associated with that trajectory.
		float cost_for_state = 0.0;

		//for i in range(len(cost_functions)) :
		//        # apply each cost function to the generated trajectory
		//float cost_for_state = calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory)
		if(trajectory_for_state.size() > 0) {
			cost_for_state = calculate_cost(*this, predictions, trajectory_for_state);
		}
		//else {
		//	cost_for_state=1.0;
		//}
		costs.push_back(cost_for_state);

		cout << "Poss STATE :" << possible_successor_states[i]<<endl;
		cout << "cost for state: "<< cost_for_state<<endl;

	}

	//# Find the minimum cost state.
	vector<Vehicle> best_traj;
	float min_cost = 9999999.0;
	for (int i=0;i < possible_successor_states.size();i++){
		string state=possible_successor_states[i];
		float cost=costs[i];
		cout << "successor state: "<<state << " cost: "<<cost<<endl;
		if (cost < min_cost){
			min_cost = cost;
			//best_next_state = state;
			best_traj=generate_trajectory(possible_successor_states[i], predictions);
			cout <<"In cost min loop: "<<" min_cost: "<<min_cost<< " best_traj[0]: "<<best_traj[0].state<<" best_traj[1]: "<<best_traj[1].state<<endl;
		}

	}
	cout <<"Cost picked :"<<min_cost<<endl;
	//cout <<"traj0: "<<best_traj[0].state<<endl;
	//cout <<"traj1: "<<best_traj[1].state<<endl;
	//////////////////////////////

    //TODO: Change return value here:
    //return generate_trajectory("KL", predictions);
	return best_traj;
}

//////////////////////////
vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<string> final_states;
    vector<vector<Vehicle>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0) {
            cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    return final_trajectories[best_idx];
}


//find next possible states based on current state
vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
	//new vector for states
    vector<string> states;
    //add "KL" state at head of vector
    states.push_back("KL");

    //current state
    string state = this->state;
    //if current state=="KL" then push back PLCL/PLCR to states--> only 2 possible successor state to KL

    //if current state=="PLCL" and currentlane != lanes_available-1
    //lanes_available-1=right most lane, can't do PLCL from right most lane
    //then push back PLCL/LCL --> only 2 possible successor state to PLCL

    //if current state=="PLCR" and currentlane != 0
    //can't do PLCR from left most lane (0)
    //then push back PLCR/LCR --> only 2 possible successor state to PLCR


    //If state is "LCL" or "LCR", then just return "KL"

    if(state.compare("KL") == 0) {
        states.push_back("PLCL");
        states.push_back("PLCR");
    } else if (state.compare("PLCL") == 0) {
        if (lane != lanes_available - 1) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    } else if (state.compare("PLCR") == 0) {
        if (lane != 0) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Given a possible next state, and predictions generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    } else if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }
    return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
    /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
    //limit_of_vel based on comfort = max_a*t+v -->t=1
	float max_velocity_accel_limit = this->max_acceleration + this->v;

    float new_position;
    float new_velocity;
    float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    //if vehicle ahead
    // 	check if vehicle behind, match traffic speed without regard to buffer, don't want to to get hit by car behind
    //  else if no vehicle behind choose max vel (s_front_veh-sego-buffer/t)+v_ahead_veh+0.5*accel*t**2 --> t=1
    //		new vel is min of (min of max_vel above or limit_of_vel) or target_speed
    //if no vehicle ahead min(limit_of_vel, target_speed)

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
        } else {
            float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
        }
    } else {
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
    }

    //new_acceleration is (new_vel-current_vel)/t --> t=1
    new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration

    //new pos=s+new_vel*t+0.5*new_accel*t*t --> t=1
    new_position = this->s + new_velocity + new_accel / 2.0;
    return{new_position, new_velocity, new_accel};

}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    Vector of 2 vehicle objects with current loc
    and next location after time 1 unit, only s changes between vehcile objects
    */

	//returns position after 1 time steps=s+v*t+a*t*t/2
    float next_pos = position_at(1);


    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state),
                                  Vehicle(this->lane, next_pos, this->v, 0, this->state)};
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    Vector of 2 vehicle objects with current loc
    and next location after time 1 unit,
    */
    //first vehicle object in trajectory vector
	vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};

	//s/v/a returned from get_kinematic method
	//based on if there are vehicles in front and behind

    vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    //second vehicle object in trajectory vector
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    next state to be considered
    Vector of 2 vehicle objects with current loc
    and next location after time 1 unit,
    */
    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;

    //intended lane (+1/-1 based on state)
    int new_lane = this->lane + lane_direction[state];

    //first vehicle object in trajectory vector
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};

    //s/v/a returned from get_kinematic method based on current lane
    //based on if there are vehicles in front and behind

    vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

    //if car behind
    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];

    } else {
    	//no car behind, update s/v/a based on lower of velocity from current vs predicted vel in intended lane
        vector<float> best_kinematics;
        //get kinematic based on intended lane
        vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    //second vehicle object in trajectory vector, don't change the lane yet (PCLx)
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory.
    */
    //final lane
	int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {

    	//pick first vehicle from vehicle vector in predictions for each index
    	//
    	next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
            //If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    vector<float> kinematics = get_kinematics(predictions, new_lane);

    //second vehicle object in trajectory vector, change lane as new_lane
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    return trajectory;
}

void Vehicle::increment(int dt = 1) {
	this->s = position_at(dt);
}

float Vehicle::position_at(int t) {
    return this->s + this->v*t + this->a*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        //what is this check for temp_vehicle.s > max_s?? -->
        //look for closest vehicle behind, max_s gets updated based on this check
        //
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int min_s = this->goal_s;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
	vector<Vehicle> predictions;
    for(int i = 0; i < horizon; i++) {
      float next_s = position_at(i);
      float next_v = 0;
      if (i < horizon-1) {
        next_v = position_at(i+1) - s; //v=(sf-si)/t -->t=0
      }
      predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
  	}
    return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->v = next_state.v;
    this->a = next_state.a;
}

void Vehicle::configure(vector<int> road_data) {
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    goal_lane = road_data[3];
    max_acceleration = road_data[4];
}


