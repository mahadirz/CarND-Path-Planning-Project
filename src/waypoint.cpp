#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "waypoint.h"
#include "helpers.h"
#include "spline.h"
#include <iostream>
#include "json.hpp"

using nlohmann::json;
using std::cout;
using std::endl;
using std::string;
using std::vector;

// Initializes Vehicle
Waypoint::Waypoint() {}

Waypoint::Waypoint(const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                   const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y)
{
    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;
    this->map_waypoints_s = map_waypoints_s;
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
}

XY Waypoint::generate_waypoint(vector<Vehicle> trajectory)
{
    Vehicle original = trajectory[0];
    Vehicle final = trajectory[1];
    double x = original.x;
    double y = original.y;
    double s = original.s;
    double d = original.d;
    double yaw = original.yaw;
    double lane = final.lane;
    double speed = final.speed;
    cout << "[waypoint.cpp] generate_waypoint x:" << x << " y:" << y << " s:" << s << " d:" << d << " yaw:" << yaw << " lane:" << lane << " speed:" << speed << endl;
    // create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
    // later we will interpolate these waypoints with a spline and fill it in with more points that control speed
    vector<double> ptsx;
    vector<double> ptsy;
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // reference x,y yaw states
    // either we will reference the starting point as where the car is or at the previous paths end point
    // vector<double> tempxy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    double ref_x = x;
    double ref_y = y;
    double ref_yaw = deg2rad(yaw);

    int prev_size = previous_path_x.size();

    //if previos size is almost empty, use the car as starting reference
    if (prev_size < 2)
    {
        // use two points that make the path tangent to the car
        double prev_car_x = x - cos(ref_yaw);
        double prev_car_y = y - sin(ref_yaw);

        cout << "[waypoint.cpp] prev_size:" << prev_size << " prev_car_x:" << prev_car_x << " x:" << x << " ref_yaw" << ref_yaw << endl;

        ptsx.push_back(prev_car_x);
        ptsx.push_back(x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(y);

        //use the previous path's end point as starting reference
    }
    else
    {
        // redefine reference state as previous path and point
        double ref_x_prev;
        double ref_y_prev;
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        ref_x_prev = previous_path_x[prev_size - 2];
        ref_y_prev = previous_path_y[prev_size - 2];

        // TOA = Tan theta = y/x
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // use two points that make the path tangent to the previous path's end point
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);

        cout << "[waypoint.cpp] prev_size:" << prev_size << "  ref_x:" << ref_x << " ref_x_prev:" << ref_x_prev << " ref_yaw:" << ref_yaw << endl;
    }
    //in frenet add evenly 30m spaced points ahead of the starting reference
    vector<double> next_wp0 = getXY(s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    json x_debug;
    for (int i = 0; i < ptsx.size(); i++)
    {
        //shift car reference angle to 0 degree
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

        x_debug[i] = ptsx[i];
    }

    for (int i = 0; i < ptsx.size() - 1; i++)
    {
        if (ptsx[i + 1] <= ptsx[i])
        {
            cout << "[waypoint.cpp] assert error:" << i << endl;
            cout << x_debug.dump() << endl;
        }
    }

    // create spline
    tk::spline sp;

    // set (x,y) points to the spline
    sp.set_points(ptsx, ptsy);

    // // define the actual (x,y) points we will use for the planner
    // vector<double> next_x_vals;
    // vector<double> next_y_vals;

    for (int i = 0; i < previous_path_x.size(); i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // calculate how to break up the spline points so that we travel at our desired reference velocity
    double target_x = 30.0;
    double target_y = sp(target_x);
    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

    double x_add_on = 0;

    // fill up the rest of our path planner after filling it with previous points, here we wil always output 50 points
    json val_x_debug;
    json val_y_debug;
    
    for (int i = 0; i <previous_path_x.size(); i++){
        val_x_debug[i] = previous_path_x[i];
        val_y_debug[i] = previous_path_y[i];
    }
    int next_i = previous_path_x.size();

    for (int i = 1; i <= 50 - previous_path_x.size(); i++)
    {
        //double N = (target_dist / (0.02 * speed / 2.24)); // mph to mps
        // .224 mph = 0.10013696 ms = 5ms^-2 * 0.02
        double N = (target_dist / (0.02 * speed));
        double x_point = x_add_on + (target_x / N);
        double y_point = sp(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back to normal after rotating it earlier
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);

        val_x_debug[next_i] = x_point;
        val_y_debug[next_i] = y_point;
        next_i++;
    }

    //cout << "[waypoint.cpp] val_x_debug:" << val_x_debug.dump() << endl;
    //cout << "[waypoint.cpp] val_y_debug:" << val_y_debug.dump() << endl;

    return XY{next_x_vals, next_y_vals};
}

Waypoint::~Waypoint() {}