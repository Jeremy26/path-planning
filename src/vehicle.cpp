#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"
#include "cost.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double v, double a, string state) {
    this->lane = lane;
    this->s = s;
    this->d = 4*lane+2;
    this->v = v;
    this->a = a;
    this->state = state;
}

void Vehicle::print_vehicle(string name)
{
    cout << name<<endl;
    cout << " - Lane :"<<this->lane<<endl;
    cout<< " - s :"<<this->s<<endl;
    cout<< " - d :"<<this->d<<endl;
    cout<<" - v :"<<this->v<<endl;
    cout<<" - a :"<<this->a<<endl;
    cout<<" - state : "<<this->state<<endl;
}

Vehicle::~Vehicle() {}


vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle> > predictions) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    this->print_vehicle("***************** This is the Current State of our Vehicle ***************");

    // First Step = Generate Successor States depending on Current State / Ex : Keep Lane / Prepare change left...
    vector<string> states = successor_states();
    
    // Define a cost
    float cost = 0.0;
    // Define an array of costs and later we will take the lowest
    vector<float> costs;
    // Define an array of states and later we will take the one with the lowest cost
    vector<string> final_states;
    
    // Array of array of vehicles : Each array has the vehicle and the same vehicle in the future
    vector<vector<Vehicle> > final_trajectories;
    /*
    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        cout <<"STATES POSSIBLE FROM HERE "<<*it<<endl;
    }
    */
    // Iterate through all possible states (depending on the lane we are in)
    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        // Call to Generate trajectory function that will output the vehicle & itself in the future
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        trajectory[1].print_vehicle(*it);
        trajectory[1].nearest_ahead = this->nearest_ahead;
        trajectory[1].nearest_behind = this->nearest_behind;
        // If there is a trajectory
        if (trajectory.size() != 0) {
            // Calculate its cost depending on parameters
            cost = calculate_cost(*this, predictions, trajectory);
            // Print it
            cout<<"$$$$ COST for "<<trajectory[1].state<<" trajectory : "<<cost<<endl;
            // Add to costs vector
            costs.push_back(cost);
            // Add to trajectories
            final_trajectories.push_back(trajectory);
        }
    }
    // Iterate through all costs and take the minimum
    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    cout <<"------------------> Decision to go with state : " << final_trajectories[best_idx][1].state<<endl;
    return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
    
    //Provides the possible next states given the current state for the FSM 
    //discussed in the course, with the exception that lane changes happen 
    //instantaneously, so LCL and LCR can only transition back to KL.
    
    vector<string> states;
    states.push_back("KL");
    if(state.compare("KL") == 0) {
      if (lane != 0) {
        states.push_back("PLCL");
      }
      if (lane != lanes_available-1) {
        states.push_back("PLCR");
      }
    } else if (state.compare("PLCL") == 0) {
            states.push_back("PLCL");
            states.push_back("LCL");
    } else if (state.compare("PLCR") == 0) {
            states.push_back("PLCR");
            states.push_back("LCR");
    }
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    // Look at the possible state and call its trajectory function
  if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
    // Create a vehicle with the same  conditions of our own
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};
    // Get kinematics
    vector<double> kinematics = get_kinematics(predictions, this->lane);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL")); // Create a vehicle based on calculated kinematics
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    double new_s;
    double new_v;
    double new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + this->lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};
    vector<double> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
        
    } else {
        vector<double> best_kinematics;
        vector<double> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
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

    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[0];
        if (abs(next_lane_vehicle.s - this->s )==0 && next_lane_vehicle.lane == new_lane) {
            //If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    vector<double> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], "KL"));
    return trajectory;
}


vector<double> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane_tried) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    double max_velocity_accel_limit = this->max_acceleration + this->v;
    double new_position = this->s;
    double new_velocity;
    double new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    bool ahead = false;
    bool behind = false;
    
    if (get_vehicle_behind(predictions, lane_tried, vehicle_behind)) {
        cout << "Vehicle Behind Detected : " << this->s - vehicle_behind.s <<" meters" <<endl;
        this->nearest_behind = this->s - vehicle_behind.s;
        behind = true;
    }
    if (get_vehicle_ahead(predictions, lane_tried, vehicle_ahead)){
        this->nearest_ahead = vehicle_ahead.s - this->s;
        cout << "Vehicle Ahead detected : "<< vehicle_ahead.s - this->s <<" meters "<< endl;
        ahead = true;
    }
    if (ahead && vehicle_ahead.s - this->s <50){
        new_velocity = vehicle_ahead.v;
    }
    else new_velocity = this->target_speed;
/*
    if (ahead && (vehicle_ahead.s - this-> s) < 100) {
        if (behind) {
            new_velocity = vehicle_ahead.v ; //must travel at the speed of traffic, regardless of preferred buffer
        }
        else {
            //double max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
            //new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
            new_velocity = vehicle_ahead.v - 10;
        }
    } 
    else if(behind && !ahead){
       // new_velocity = min(max_velocity_accel_limit, this->target_speed);
       new_velocity = this->target_speed;
    }
    */
    
    //new_accel = (new_velocity - this->v)*.002; //Equation: (v_1 - v_0)/t = acceleration
    if (new_velocity > this-> v) new_accel = this->max_acceleration;
    else new_accel = - this->max_acceleration;
    new_position += new_velocity*0.02 + new_accel*0.0004 / 2.0; // x = vt + 1/2 * at^2

    return {new_position, new_velocity, new_accel};
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
    // Iterate though predictions
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        // Return closest vehicle behind us if it exist
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
    // Select goal S
    int min_s = this->goal_s;
    // Initialize boolean at false
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    // Iterate through predictions
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        // Create a buffer vehicle around iterated vehicle
        temp_vehicle = it->second[0];
        // Return closest vehicle in front of us if it exist
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
      double next_s = position_at(i);
      double next_v = 0;
      if (i < horizon-1) {
        next_v = position_at(i+1) - s;
      }
      predictions.push_back(Vehicle(this->lane, next_s, next_v, 0,"KL"));
  	}
    return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    Vehicle next_state = trajectory[1];
    /*
    if(next_state.v > this->v && next_state.v < target_speed){
        this->v += next_state.a;
    }
    else if (next_state.v > target_speed)
    {
        if (this->v < target_speed){
            this->v+= next_state.a;
        }
    }
    else if(next_state.v < this->v){
        this->v -= next_state.a;
    }
    */
    if (this->v < next_state.v){
        this->v+=max_acceleration;
    }
    else
    {
        this->v -=max_acceleration;
    }

    this->state = next_state.state;
    this->lane = next_state.lane;
    this->d = next_state.d;
    this->s = next_state.s;
    this->a = next_state.a;
}