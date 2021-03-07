#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle
{
public:
    // Constructors
    Vehicle();
    Vehicle(double x, double y, double s, double d, double yaw, double speed, int lane = 0, string state = "CS", double a = 0);

    // Destructor
    virtual ~Vehicle();

    void frenetDToLane();

    // Vehicle functions
    vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> &predictions);

    vector<string> successor_states();

    vector<Vehicle> generate_trajectory(string state,
                                        map<int, vector<Vehicle>> &predictions);

    vector<double> get_kinematics(map<int, vector<Vehicle>> &predictions, int lane);

    vector<Vehicle> constant_speed_trajectory();

    vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> &predictions);

    vector<Vehicle> lane_change_trajectory(string state,
                                           map<int, vector<Vehicle>> &predictions);

    vector<Vehicle> prep_lane_change_trajectory(string state,
                                                map<int, vector<Vehicle>> &predictions);

    void increment(int dt);

    double position_at(double t);

    bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane,
                            Vehicle &rVehicle);

    bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane,
                           Vehicle &rVehicle);

    vector<Vehicle> generate_predictions(int horizon = 1);

    map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

    int L = 1;

    // car telemetry
    double x, y, s, d, yaw, speed, a = 0, last_lane_change_s = 0;

    // the car odometer is usually different that our intended speed
    // due to delay in projection
    double last_intended_speed = -1;

    int preferred_buffer = 30; // impacts "keep lane" behavior.

    int lane, goal_lane , lanes_available = 3;

    // in meter
    double target_speed = 22.13, max_acceleration = 5, goal_s=6945.554;

    string state;
};

#endif // VEHICLE_H