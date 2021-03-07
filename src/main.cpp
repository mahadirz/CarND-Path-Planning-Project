#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "waypoint.h"

// for convenience
using nlohmann::json;
using std::cout;
using std::endl;
using std::string;
using std::vector;

/**
 * The simulator runs a cycle every 20 ms (50 frames per second)
 * path planning program will provide a new path at least one 20 ms cycle behind
 * The jerk and the total acceleration should not exceed 10 m/s^2
**/

string getCurrentPath()
{
  size_t size;
  char *path = NULL;
  path = getcwd(path, size);
  return path;
}

bool is_file_exist(const char *fileName)
{
  std::ifstream infile(fileName);
  return infile.good();
}

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  Vehicle ego = Vehicle(0, 0, 0, 0, 0, 0, 1, "KL");

  // Waypoint map to read from
  string map_file_ = "data/highway_map.csv";


  // std::cout << "Current path is " << getCurrentPath() << '\n';
  // std::cout << "File exist " << is_file_exist(map_file_.c_str()) << '\n';

  if(!is_file_exist(map_file_.c_str())){
    cout << "Cannot find " << map_file_ << endl;
    exit(1);
  }

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &ego]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {


      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"]; // mph
          cout << "[main.cpp] car_speed in mph:"<<  car_speed << endl;
          car_speed = car_speed / 2.237;  // convert from mph to meter per second

          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          int prev_size = previous_path_x.size();
          

          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
          ego.x = car_x;
          ego.y = car_y;
          ego.s = car_s;
          ego.d = car_d;
          ego.yaw = car_yaw;
          ego.speed = car_speed;
          //Vehicle ego = Vehicle(car_x, car_y, car_s, car_d, car_yaw, car_speed, lane, "KL");
          map<int, vector<Vehicle>> predictions;
          int horizon = prev_size > 0 ? prev_size: 50;
          // prediction
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            //  id, x, y, vx, vy, s, d
            double sensor_id = sensor_fusion[i][0];
            double sensor_x = sensor_fusion[i][1];
            double sensor_y = sensor_fusion[i][2];
            double sensor_vx = sensor_fusion[i][3];
            double sensor_vy = sensor_fusion[i][4];
            double sensor_s = sensor_fusion[i][5];
            double sensor_d = sensor_fusion[i][6];
            double sensor_speed = sqrt(sensor_vx * sensor_vx + sensor_vy * sensor_vy);
            Vehicle sensor_vehicle = Vehicle(sensor_x, sensor_y, sensor_s, sensor_d, 0.0, sensor_speed);
            sensor_vehicle.frenetDToLane();
            vector<Vehicle> preds = sensor_vehicle.generate_predictions(horizon);
            predictions[sensor_id] = preds;
          }

          // Behavior
          //Trajectory trajectory = Trajectory(previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          cout << "[main.cpp] original s:" << ego.s << " d:" << ego.d << " lane:" << ego.lane << " speed:" << ego.speed << endl; 
          vector<Vehicle> trajectory = ego.choose_next_state(predictions);
          Waypoint waypoint = Waypoint(previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          Vehicle final = trajectory[1];
          cout << "[main.cpp] existing lane:" << ego.lane << " final lane:" << final.lane  << endl;
          cout << "[main.cpp] final s:" << final.s << " d:" << final.d << " speed:" << final.speed << " a:" << final.a << endl; 
          if (ego.lane  != final.lane){
            cout << "[main.cpp] ego changing lane from " << ego.lane << " to " << final.lane << endl;
            cout << "[main.cpp] modify " << ego.last_lane_change_s << endl;
            ego.last_lane_change_s = (car_s/1.0);
            cout << "[main.cpp] new last_lane_change_s:" << ego.last_lane_change_s << endl;
          }
          ego.lane = final.lane;
          ego.last_intended_speed = (final.speed/1.0);
          XY waypoint_xy = waypoint.generate_waypoint(trajectory);

          vector<double> next_x_vals = waypoint_xy.x;
          vector<double> next_y_vals = waypoint_xy.y;
          
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}