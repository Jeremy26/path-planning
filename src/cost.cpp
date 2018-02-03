#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

// Here we define the weights of the costs.
const float EFFICIENCY = pow(10,2);
const float COLLISION = pow(10,1);
const float LANE_CHANGE = pow(10,0);

const float VEHICLE_RADIUS = 1.5; // model vehicle as circle to simplify collision detection

// Changing lane is less comfortable than staying in our lane
float lane_change_cost (const Vehicle & vehicle, const vector<Vehicle> & trajectory){
    if (vehicle.lane != trajectory[1].lane) return 1.0;
    else return 0.0;
}

float collision_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions ){
    
    float delta_ahead = trajectory[1].nearest_ahead;
    cout << "DELTA AHEAD "<<delta_ahead <<endl;
    float delta_behind = trajectory[1].nearest_behind;
    cout << "DELTA BEHIND " <<delta_behind <<endl;
    if ((delta_ahead < 40*VEHICLE_RADIUS)) return 1.0;
     return 0.0;
}

float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed. 
    */

    float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
    if (proposed_speed_intended < 0) { // If no vehicle found, lane_speed returns -1
        proposed_speed_intended = vehicle.target_speed;
    }

    float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
    if (proposed_speed_final < 0) { // If no vehicle found, lane_speed returns -1
        proposed_speed_final = vehicle.target_speed;
    }
    
    float cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final)/vehicle.target_speed;

    return cost;
}

float lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) {
    /*
    All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
    we can just find one vehicle in that lane.
    */
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        int key = it->first;
        Vehicle vehicle = it->second[0];
        if (vehicle.lane == lane && key != -1) {
            return vehicle.v;
        }
    }
    //Found no vehicle in the lane
    return -1.0;
}

float calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    float cost = 0.0;

    //Add additional cost functions here.
    //vector< function<float(const Vehicle & , const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, float> &)>> cf_list = {inefficiency_cost, collision_cost};
    vector<float> weight_list = {EFFICIENCY, COLLISION, LANE_CHANGE};
    vector<float> costs;
    costs.push_back(inefficiency_cost(vehicle, trajectory, predictions, trajectory_data));
    costs.push_back(collision_cost(vehicle, trajectory, predictions));
    costs.push_back(lane_change_cost(vehicle,trajectory));
    for (int i = 0; i < costs.size(); i++) {
        //float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
        float new_cost = weight_list[i]*costs[i];
        cost += new_cost;
    }
    return cost;
}

map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    /*
    Generate helper data to use in cost functions:
    indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
    final_lane: the lane of the vehicle at the end of the trajectory.
    distance_to_goal: the distance of the vehicle to the goal.

    Note that indended_lane and final_lane are both included to help differentiate between planning and executing
    a lane change in the cost functions.
    */
    map<string, float> trajectory_data;
    Vehicle trajectory_last = trajectory[1]; // Take the future trajectory
    float intended_lane; // Intended lane will be this future trajectory +/- 1 lane depending on state

    if (trajectory_last.state.compare("PLCR") == 0) {
        intended_lane = trajectory_last.lane + 1;
    } else if (trajectory_last.state.compare("PLCL") == 0) {
        intended_lane = trajectory_last.lane - 1;
    } else {
        intended_lane = trajectory_last.lane;
    }

    float distance_to_goal = vehicle.goal_s - trajectory_last.s;
    float final_lane = trajectory_last.lane;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;
    return trajectory_data;
}

