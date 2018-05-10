# SDC Term 3 
## Lesson 4 Behavioral Planning 

## NOTES:

[Pynotebook](./Behavioral_Planning.ipynb)


##Implement a Behavior Planner

In this exercise you will implement a behavior planner and cost functions for highway driving. The planner will use prediction data to set the state of the ego vehicle to one of 5 values and generate a corresponding vehicle trajectory:

*	<span style="color:red">"KL" </span> - Keep Lane
*	<span style="color:red">"LCL" / "LCR"</span>- Lane Change Left / Lane Change Right
*	<span style="color:red">"PLCL" / "PLCR"</span> - Prepare Lane Change Left / Prepare Lane Change Right

The objective of the quiz is to navigate through traffic to the goal in as little time as possible. Note that the **goal lane** and **s value**, as well as the **traffic speeds** for each lane, are set in <span style="color:red"> main.cpp </span> below. Since the goal is in the slowest lane, in order to get the lowest time, you'll want to choose cost functions and weights to drive in faster lanes when appropriate. We've provided two suggested cost functions in <span style="color:red">cost.cpp </span>.

## Instructions

1.**Implement the choose\_next_state method in the vehicle.cpp class.** 

You can use the Behavior Planning Pseudocode concept as a guideline for your implementation. 
 
---
	def transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights):
    # only consider states which can be reached from current FSM state.
    possible_successor_states = successor_states(current_fsm_state)

    # keep track of the total cost of each state.
    costs = []
    for state in possible_successor_states:
        # generate a rough idea of what trajectory we would
        # follow IF we chose this state.
        trajectory_for_state = generate_trajectory(state, current_pose, predictions)

        # calculate the "cost" associated with that trajectory.
        cost_for_state = 0
        for i in range(len(cost_functions)) :
            # apply each cost function to the generated trajectory
            cost_function = cost_functions[i]
            cost_for_cost_function = cost_function(trajectory_for_state, predictions)

            # multiply the cost by the associated weight
            weight = weights[i]
            cost_for_state += weight * cost_for_cost_function
         costs.append({'state' : state, 'cost' : cost_for_state})

    # Find the minimum cost state.
    best_next_state = None
    min_cost = 9999999
    for i in range(len(possible_successor_states)):
        state = possible_successor_states[i]
        cost  = costs[i]
        if cost < min_cost:
            min_cost = cost
            best_next_state = state 

    return best_next_state 
 
---

In this quiz, there are a couple of small differences from that pseudocode: you'll be returning a best trajectory corresponding to the best state instead of the state itself. Additionally, the function inputs will be slightly different in this quiz than in the classroom concept. For this part of the quiz, we have provided several useful functions:

* successor_states() - Uses the current state to return a vector of possible successor states for the finite state machine.
	
* generate_trajectory(string state, map<int, vector<Vehicle>> predictions) - Returns a vector of Vehicle objects representing a vehicle trajectory, given a state and predictions. Note that trajectory vectors might have size 0 if no possible trajectory exists for the state.

* calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory) - Included from cost.cpp, computes the cost for a trajectory.


2.**Choose appropriate weights for the cost functions in <span style="color:red;">cost.cpp</span> to induce the desired vehicle behavior.** Two suggested cost functions have been implemented based on previous quizzes, but you are free to experiment with your own cost functions. The <span style="color:red;">get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) </span>  function in <span style="color:red"> cost.cpp </span> provides some preprocessing of vehicle data that may be useful in your cost functions. See if you can get the vehicle to move into a fast lane for a portion of the track and then move back to reach the goal!
Hit Test Run and see how your car does! How fast can you get to the goal?


## Extra Practice
Provided in one of the links below is a zip file <span style="color:red;"> python\_3\_practice </span> [link](./python-3-practice.zip), which is the same problem written in Python 3 - you can optionally use this file for additional coding practice. In the <span style="color:red;">python\_3_solution </span> [link](./python-3-solution.zip), the solution is provided as well. If you get stuck on the quiz see if you can convert the python solution to C++ and pass the classroom quiz with it. You can run the python quiz with <span style="color:red;">python simulate\_behavior.py </span>.