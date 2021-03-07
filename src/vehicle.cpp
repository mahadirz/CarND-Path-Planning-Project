#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "cost.h"
#include "waypoint.h"
#include <iostream>

using std::cout;
using std::endl;
using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle() {}

Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double speed, int lane, string state, double a)
{
    this->lane = lane;
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
    this->speed = speed;
    this->state = state;
    this->a = a;
}

Vehicle::~Vehicle() {}

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> &predictions)
{
    /**
   * Here you can implement the transition_function code from the Behavior 
   *   Planning Pseudocode classroom concept.
   *
   * @param A predictions map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle 
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego 
   *   vehicle state.
   *
   * Functions that will be useful:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing 
   *    a vehicle trajectory, given a state and predictions. Note that 
   *    trajectory vectors might have size 0 if no possible trajectory exists 
   *    for the state. 
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
   *
   */
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<vector<Vehicle>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it)
    {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0)
        {
            cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);
            cout << "[vehicle.cpp] state:" << *it << " cost:" << cost << endl;
            cout << "[vehicle.cpp] speed traj 0:" << trajectory[0].speed << endl;
            cout << "[vehicle.cpp] speed traj 1:" << trajectory[1].speed << endl;
            final_trajectories.push_back(trajectory);
        }
    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);

    Vehicle best_trajectory = final_trajectories[best_idx][1];
    if (this->lane != best_trajectory.lane){
        cout << "[vehicle.cpp] Lowest cost in lane changing to " << best_trajectory.state << endl;
    }
    return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states()
{
    // Provides the possible next states given the current state for the FSM
    //   discussed in the course, with the exception that lane changes happen
    //   instantaneously, so LCL and LCR can only transition back to KL.
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if (state.compare("KL") == 0)
    {
        if (lane != 0)
        {
            states.push_back("LCL");
        }
        if (lane != lanes_available - 1)
        {
            states.push_back("LCR");
        }
    }

    // If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state,
                                             map<int, vector<Vehicle>> &predictions)
{
    // Given a possible next state, generate the appropriate trajectory to realize
    //   the next state.
    vector<Vehicle> trajectory;
    if (state.compare("KL") == 0)
    {
        cout << "[vehicle.cpp] " << state << endl;
        trajectory = keep_lane_trajectory(predictions);
    }
    else if (state.compare("LCL") == 0 || state.compare("LCR") == 0)
    {
        cout << "[vehicle.cpp] " << state << endl;
        trajectory = lane_change_trajectory(state, predictions);
    }
    return trajectory;
}

vector<double> Vehicle::get_kinematics(map<int, vector<Vehicle>> &predictions,
                                       int lane)
{
    // Gets next timestep kinematics (position, velocity, acceleration)
    //   for a given lane. Tries to choose the maximum velocity and acceleration,
    //   given other vehicle positions and accel/velocity constraints.
    //double max_velocity_accel_limit = (this->max_acceleration * 0.02) + this->speed;

    double new_position;
    double new_velocity;
    double new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    bool is_vehicle_ahead = get_vehicle_ahead(predictions, lane, vehicle_ahead);
    bool is_vehicle_behind = get_vehicle_behind(predictions, lane, vehicle_behind);

    cout << "[vehicle.cpp] Odometer speed:" << this->speed << " Last Projection Speed:" << last_intended_speed << endl;

    if(this->last_intended_speed == -1){
        this->last_intended_speed = (this->speed/1.0);
    }

    if (is_vehicle_ahead)
    {
        cout << "[vehicle.cpp] vehicle_ahead.lane:" << vehicle_ahead.lane << " this->lane: " << this->lane << " distance:" << (vehicle_ahead.s - this->s) << endl;
    }

    if (is_vehicle_behind)
    {
        cout << "[vehicle.cpp] vehicle_behind.lane" << vehicle_behind.lane << " this->lane" << this->lane << " distance:" << (vehicle_behind.s - this->s) << endl;
    }

    double distance_to_car = (vehicle_ahead.s - this->s);
    // if (is_vehicle_ahead && (vehicle_ahead.s - this->s) < 0.5 && vehicle_ahead.lane == this->lane)
    // {
    //     // emergency
    //     cout << "emergency distance:" << (vehicle_ahead.s - this->s) << endl;
    //     new_velocity = vehicle_ahead.speed;
    // }
    if (is_vehicle_ahead && distance_to_car <= this->preferred_buffer && vehicle_ahead.lane == this->lane)
    {
        cout << "[vehicle.cpp] slow down" << endl;
        cout << "[vehicle.cpp] Distance to impact :" << (vehicle_ahead.s - this->s) << "m" << endl;
        cout << "[vehicle.cpp] vehiche ahead speed :" << vehicle_ahead.speed << " car speed:" << this->last_intended_speed << endl;
        cout << "[vehicle.cpp] vehicle_ahead.s " << vehicle_ahead.s << " this->s:" << this->s << endl;
        // v = v0 + at
        double time_to_match_v = (this->last_intended_speed - vehicle_ahead.speed) / this->max_acceleration;
        double a_required = (vehicle_ahead.speed - this->last_intended_speed) / 0.02;
        cout << "[vehicle.cpp] time_to_match_v:" << time_to_match_v << " screen update required:" << time_to_match_v/0.02  << endl;
        cout << "[vehicle.cpp] Acceleration required:" << a_required << endl;
        // slow down
        new_velocity = (this->last_intended_speed - (this->max_acceleration * 0.02));

        //double max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.speed - 0.5 * (this->max_acceleration);
        cout << "[vehicle.cpp] Existing velocity:" << this->last_intended_speed << " New velocity:" << new_velocity << endl;
    }
    else
    {
        new_velocity = (this->max_acceleration * 0.02) + this->last_intended_speed;
        if (new_velocity > this->target_speed)
        {
            new_velocity = this->target_speed;
        }

        if( is_vehicle_ahead && distance_to_car <= this->preferred_buffer && new_velocity > vehicle_ahead.speed){
            // cap to the front vehicle velocity
            new_velocity = vehicle_ahead.speed;
        }

        cout << "this->speed:" << this->last_intended_speed << " new_velocity:" << new_velocity << endl;
    }

    new_accel = (new_velocity - this->last_intended_speed)/0.02; // Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s + new_velocity + new_accel / 2.0;
    this->last_intended_speed = (new_velocity/1.0);

    return {new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> &predictions)
{
    // Generate a keep lane trajectory.
    cout << "[vehicle.cpp] keep_lane_trajectory speed:" << this->speed << endl;
    vector<Vehicle> trajectory = {Vehicle(this->x, this->y, this->s, this->d, this->yaw, this->speed, this->lane)};
    vector<double> kinematics = get_kinematics(predictions, this->lane);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    cout << "[vehicle.cpp] keep_lane_trajectory after kinematics speed:" << new_v << endl;
    trajectory.push_back(Vehicle(this->x, this->y, new_s, this->d, this->yaw, new_v, this->lane, "KL", new_a));

    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state,
                                                map<int, vector<Vehicle>> &predictions)
{
    double new_s;
    double new_v;
    double new_a;
    // Generate a lane change trajectory.
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    Vehicle nearest_vehicle;
    double nearest_s = 1e7;
    if (((this->s - last_lane_change_s) < 100) || (this->s < 300))
    {
        // min x meter since last change lane
        cout << "[vehicle.cpp] last_lane_change_s less than x meter " << (this->s - last_lane_change_s) << " s:" << this->s << " last_lane_change_s:" << last_lane_change_s << endl;
        return trajectory;
    }
    // Check if a lane change is possible (check if another vehicle occupies
    //   that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
         it != predictions.end(); ++it)
    {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.lane == new_lane)
        {
            if (fabs(next_lane_vehicle.s - this->s) < this->preferred_buffer)
            {
                // If lane change is not possible, return empty trajectory.
                cout << "[vehicle.cpp] change to lane " << state << " imposible ";
                cout << "new lane:" << new_lane << " found vehicle in this lane:" << next_lane_vehicle.lane;
                cout << "next_lane_vehicle.s:" << next_lane_vehicle.s << " s:" << s << " distance:";
                cout << fabs(next_lane_vehicle.s - this->s) << endl;
                return trajectory;
            }
            else
            {
                if(next_lane_vehicle.s < nearest_s){
                    nearest_vehicle = next_lane_vehicle;
                }
                cout << "[vehicle.cpp] changing lane distance:" << fabs(next_lane_vehicle.s - this->s) << endl;
            }
        }
    }
    if( nearest_vehicle.speed - this->speed > 5){
        cout << "[vehicle.cpp] vehicle at the back approaching fast!" << endl;
        return trajectory;
    }
    cout << "[vehicle.cpp] last_lane_change_s:" << last_lane_change_s << " changing lane OK " << endl;
    trajectory.push_back(Vehicle(this->x, this->y, this->s, this->d, this->yaw, this->speed, this->lane, this->state));
    // vector<double> kinematics = get_kinematics(predictions, new_lane);
    // new_s = kinematics[0];
    // new_v = kinematics[1];
    // new_a = kinematics[2];
    // found the reasons the car stall because new_v not init
    new_v = (this->last_intended_speed/1.0);
    if (this->speed < target_speed)
    {
        new_v = this->speed + (this->max_acceleration * 0.02);
    }
    if (new_v > this->target_speed){
        new_v = this->target_speed;
    }
    cout << "[vehicle.cpp] s:" << this->s << " speed:" << this->speed << " new_speed:" << new_v << endl;
    trajectory.push_back(Vehicle(this->x, this->y, this->s, this->d, this->yaw, new_v, new_lane, state, this->a));
    return trajectory;
}

double Vehicle::position_at(double t)
{
    // x = x0 + v0t + 1/2at^2
    return this->s + this->speed * t + this->a * t * t / 2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> &predictions,
                                 int lane, Vehicle &rVehicle)
{
    // Returns a true if a vehicle is found behind the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
         it != predictions.end(); ++it)
    {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s)
        {
            max_s = temp_vehicle.s;
            // only the nearest vehicle
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }

    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> &predictions,
                                int lane, Vehicle &rVehicle)
{
    // Returns a true if a vehicle is found ahead of the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    double min_distance = 1e7;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
         it != predictions.end(); ++it)
    {
        temp_vehicle = it->second[0];
        // cout << "temp_vehicle.lane:" << temp_vehicle.lane << " this->lane:" << this->lane <<  " temp_vehicle.s:" << temp_vehicle.s <<  " this->s:" << this->s << endl;
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s)
        {
            if ((temp_vehicle.s - this->s) < min_distance)
            {
                min_distance = temp_vehicle.s - this->s;
                rVehicle = temp_vehicle;
            }
            found_vehicle = true;
        }
    }

    return found_vehicle;
}

void Vehicle::frenetDToLane()
{
    if (d >= 0 && d <= 4)
    {
        lane = 0;
    }
    else if (d > 4 && d <= 8)
    {
        lane = 1;
    }
    else if (d > 8 && d <= 12)
    {
        lane = 2;
    }
}

vector<Vehicle> Vehicle::generate_predictions(int horizon)
{
    // Generates predictions for non-ego vehicles to be used in trajectory
    //   generation for the ego vehicle.
    vector<Vehicle> predictions;
    for (int i = 0; i < horizon; ++i)
    {
        // s + prev_size * v * t = s + prev_size(vt)
        // let prev_size = 3 then s + vt + vt + vt
        double next_s = position_at(0.02);
        predictions.push_back(Vehicle(this->x, this->y, next_s, this->d, 0.0, this->speed, this->lane));
    }

    return predictions;
}