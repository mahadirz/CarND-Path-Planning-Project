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

// for convenience
using nlohmann::json;
using std::cout;
using std::endl;
using std::string;
using std::vector;

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

  // start in line 1
  // middle lane
  // 0 is left turn
  int lane = 1;

  // have a reference velociy to target
  double ref_vel = 0; //mph

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::cout << "Current path is " << getCurrentPath() << '\n';
  std::cout << "File exist " << is_file_exist(map_file_.c_str()) << '\n';

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
               &map_waypoints_dx, &map_waypoints_dy, &ref_vel, &lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

          bool too_close = false;
          bool safe_to_change_left = false;
          bool safe_to_change_right = false;
          int lane_right = lane + 1 == 3 ? 2 : lane + 1;
          int lane_left = lane - 1 == -1 ? 0 : lane - 1;
          //cout << lane << " " << lane_right << " " << lane_left << endl;

          json sensor_data_debug;

          //find ref_v to use
          safe_to_change_right = false;
          safe_to_change_left = false;
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            // car is in my lane
            float check_car_d = sensor_fusion[i][6];
            string carId = sensor_fusion[i][0].dump();

            if (check_car_d < (2 + (4 * lane) + 2) && check_car_d > (2 + 4 * lane - 2))
            {
              // the car in front
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];

              // if using previous points can project s value output
              check_car_s += ((double)prev_size * 0.02 * check_speed);
              // check_car_s in-front and distance less than 30m
              if ((check_car_s > car_s) & ((check_car_s - car_s) < 30))
              {
                // do logic here, lower reference velocity so we dont crash into the car infront of us
                // could also flag to try to change lanes
                //ref_val = 29.5 ; //mph
                too_close = true;
                cout << "too close" << endl;
              }
            }

            if (lane_left != lane && check_car_d < (2 + (4 * lane_left) + 2) && check_car_d > (2 + 4 * lane_left - 2))
            {
              //  [id, x, y, vx, vy, s, d]
              //left lane
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_s = sensor_fusion[i][5];
              double check_d = sensor_fusion[i][6];
              double check_car_s = sensor_fusion[i][5];

              check_car_s += ((double)prev_size * 0.02 * check_speed);
              //cout << "check left " <<  lane_left << "  lane car ID: " << sensor_fusion[i][0] <<  " Distance to:" << fabs(check_car_s-car_s) << endl;
              sensor_data_debug[carId] = {
                  {"lane", lane_left},
                  {"distance", fabs(check_car_s - car_s)},
                  {"speed", check_speed},
                  {"car_s", check_s},
                  {"car_d", check_d},
                  {"s", car_s},
                  {"d", car_d},
              };
              if ((fabs(check_car_s - car_s) > 50))
              {
                // it's safe to change to left lane
                safe_to_change_left = true;
                //cout << "safe to change left" << endl;
                sensor_data_debug[carId]["safe_to_change_left"] = true;
              }
              else
              {
                safe_to_change_left = false;
                sensor_data_debug[carId]["safe_to_change_left"] = false;
              }
            }


            sensor_data_debug[carId]["safe_to_change_right"] = false;
            if (lane_right != lane && check_car_d < (2 + (4 * lane_right) + 2) && check_car_d > (2 + 4 * lane_right - 2))
            {
              //right lane
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_s = sensor_fusion[i][5];
              double check_d = sensor_fusion[i][6];
              double check_car_s = sensor_fusion[i][5];

              check_car_s += ((double)prev_size * 0.02 * check_speed);
              //cout << "check right " << lane_right << "  lane car ID: " << sensor_fusion[i][0] <<  " Distance to:" << fabs(check_car_s-car_s) << endl;
              sensor_data_debug[carId] = {
                  {"lane", lane_right},
                  {"distance", fabs(check_car_s - car_s)},
                  {"speed", check_speed},
                  {"car_s", check_s},
                  {"car_d", check_d},
                  {"s", car_s},
                  {"d", car_d},
              };
              if ((fabs(check_car_s - car_s) > 50))
              {
                // it's safe to change to right lane
                safe_to_change_right = true;
                //cout << "safe to change right" << endl;
                sensor_data_debug[carId]["safe_to_change_right"] = true;
              }
              else{
                safe_to_change_right = false;
                sensor_data_debug[carId]["safe_to_change_right"] = false;
              }
              
            }

          }

          // if(too_close){
          //   if(lane == 1)
          //       {
          //         lane = 0;
          //       }
          //       else if( lane ==  0)
          //       {
          //         lane = 1;
          //       }
          //       else if( lane ==  2)
          //       {
          //         lane = 1;
          //       }
          // }
          if (too_close)
          {
            cout << sensor_data_debug.dump() << endl;
            cout << "safe_to_change_right " << safe_to_change_right << endl;
            cout << "safe_to_change_left " << safe_to_change_left << endl;
            cout << "too close and we 're making decision from  lane:" << lane;
            if (safe_to_change_right && safe_to_change_left){
              // prioritize left lane
              safe_to_change_right = 0;
            }
            if (safe_to_change_right)
            {
              lane = lane_right;
              cout << " to lane:" << lane << endl;
            }
            else if (safe_to_change_left)
            {
              lane = lane_left;
              cout << " to lane:" << lane << endl;
            }
            else
            {
              cout << " to same lane:" << endl;
            }
            // 5m per second
            ref_vel -= .224;
          }
          else if (ref_vel < 49.5)
          {
            ref_vel += .224;
          }

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // later we will interpolate these waypoints with a spline and fill it in with more points that control speed
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y yaw states
          // either we will reference the starting point as where the car is or at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          //if previos size is almost empty, use the car as starting reference
          if (prev_size < 2)
          {
            // use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

            //use the previous path's end point as starting reference
          }
          else
          {
            // redefine reference state as previous path and point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];

            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          //in frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++)
          {
            //shift car reference angle to 0 degree
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // create spline
          tk::spline s;

          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

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
          double target_y = s(target_x);
          double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

          double x_add_on = 0;

          // fill up the rest of our path planner after filling it with previous points, here we wil always output 50 points
          for (int i = 1; i <= 50 - previous_path_x.size(); i++)
          {
            double N = (target_dist / (0.02 * ref_vel / 2.24));
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);

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
          }

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