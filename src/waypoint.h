#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <map>
#include <string>
#include <vector>
#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

struct XY
{
    vector<double> x; 
    vector<double> y;  
};

class Waypoint
{
public:
    // Constructors
    Waypoint();
    Waypoint(const vector<double> &previous_path_x, const vector<double> &previous_path_y,
               const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y);

    XY generate_waypoint(vector<Vehicle> trajectory);

    // Destructor
    virtual ~Waypoint();

protected:

    vector<double> previous_path_x;
    vector<double> previous_path_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
};

#endif //WAYPOINT_H
