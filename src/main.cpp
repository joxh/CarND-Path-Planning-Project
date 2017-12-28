#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

double cyclic_difference(double sf, double si, double length_of_cycle){
	return fmod((sf - si) + length_of_cycle/2, length_of_cycle) - length_of_cycle/2;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  // Persitent state planning variables
  double ref_vel_Mph = 0.0; // Current reference velocity
  int lane = 1;
  
			

  h.onMessage([&max_s, &lane, &ref_vel_Mph, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

			
			double time_between_waypoints = 0.02; // seconds
			int num_waypoints_to_return = 50;

			double lane_width = 4; // Meters
			double waypoint_anchor_sep = 30.0; // Meters

			double brake_acceleration = 5.0; // Meters per second square
			double throttle_acceleration = 3.0; // meters per second squared
			const double Mph_per_mps = 2.24;

			double target_vel_Mph = 48.5; // The speed the car would drive if unimpeded
			double max_allowed_a = 9.0;

			double bring_back_ratio = .1;

			double unsafe_range = 30.0; // Meters
			double unsafe_range_behind = 15.0; // Meters

			

			double ref_vel_mps = ref_vel_Mph / Mph_per_mps;

			double target_vel_mps = target_vel_Mph / Mph_per_mps;

			// Do lane logic:

			int prev_size = previous_path_x.size();

			bool too_close = false;
			bool open_on_left = lane > 0 ? true : false;
			bool open_on_right = lane < 2 ? true: false;

			double car_future_s = car_s;
			if (prev_size > 0){
				car_future_s = end_path_s;
			}

			for(int i = 0; i < sensor_fusion.size(); i++){
				
				double sense_vx = sensor_fusion[i][3];
				double sense_vy = sensor_fusion[i][4];
				double sense_s = sensor_fusion[i][5];
				double sense_d = sensor_fusion[i][6];

				double sense_speed = sqrt(sense_vx * sense_vx + sense_vy * sense_vy);

				bool in_same_lane = sense_d < lane_width*(lane + 1) && sense_d > lane_width*lane;
				bool in_lane_to_left = sense_d < lane_width*lane && sense_d > lane_width*(lane -1);
				bool in_lane_to_right = sense_d < lane_width*(lane + 2) && sense_d > lane_width*(lane + 1);

				double sense_future_s = sense_s + (sense_speed * prev_size * time_between_waypoints);
				bool behind_sensed_car = cyclic_difference(sense_future_s, car_future_s, max_s) > 0;
				bool within_unsafe_range = cyclic_difference(sense_future_s, car_future_s, max_s) < unsafe_range;

				//bool within_change_blocking_range_ahead = behind_sensed_car && within_unsafe_range;
				//bool within_change_blocking_range_behind = (~behind_sensed_car) && ((sense_future_s - car_future_s) > -unsafe_range_behind);
				//bool within_blocking_distance = within_change_blocking_range_ahead || within_change_blocking_range_behind;
				bool within_blocking_distance = abs(cyclic_difference(sense_future_s, car_future_s, max_s)) < unsafe_range;

				if (in_same_lane){
					if (behind_sensed_car && within_unsafe_range){
						too_close = true;
					}
				}

				if (in_lane_to_left && within_blocking_distance){
					open_on_left = false;
				}

				if (in_lane_to_right && within_blocking_distance){
					open_on_right = false;
				}
			}

			if (too_close){
				ref_vel_mps -= brake_acceleration * time_between_waypoints;
				ref_vel_Mph = ref_vel_mps * Mph_per_mps;
				if(open_on_left){
					lane -= 1;
				} else if(open_on_right){
					lane += 1;
				}

			} else if (ref_vel_mps < target_vel_mps){
				ref_vel_mps += throttle_acceleration * time_between_waypoints;
				ref_vel_Mph = ref_vel_mps * Mph_per_mps;
			}

			// Create widely spaced (x,y) waypoints that implement the result of the planning logic
			// They will serve as the anchor points for a spline:
			vector<double> ptsx;
			vector<double> ptsy;

			// Reference frame information for the car frame:
			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw); //This is returned in degrees

			// Use remaining waypoints from last planned path in order to figure out tangent heading
			
			if(prev_size < 2){
				// Need to construct previous points from car's heading
				double implied_step_size = 1.0;
				double prev_car_x = car_x - implied_step_size * cos(ref_yaw);
				double prev_car_y = car_y - implied_step_size * sin(ref_yaw);

				ptsx.push_back(prev_car_x);
				ptsx.push_back(car_x);

				ptsy.push_back(prev_car_y);
				ptsy.push_back(car_y);
			} else {
				// Can use the previous path directly

				// Redefine reference position as being at the previous point
				ref_x = previous_path_x[prev_size -1];
				ref_y = previous_path_y[prev_size -1];

				double ref_x_prev = previous_path_x[prev_size -2];
				double ref_y_prev = previous_path_y[prev_size -2];
				
				ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

				ptsx.push_back(ref_x_prev);
				ptsx.push_back(ref_x);
				
				ptsy.push_back(ref_y_prev);
				ptsy.push_back(ref_y);
			}
			

			
			vector<double> next_wp_0 = getXY(car_s + 1.0 * waypoint_anchor_sep, lane_width * (0.5 + lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp_1 = getXY(car_s + 2.0 * waypoint_anchor_sep, lane_width * (0.5 + lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp_2 = getXY(car_s + 3.0 * waypoint_anchor_sep, lane_width * (0.5 + lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

			ptsx.push_back(next_wp_0[0]);
			ptsy.push_back(next_wp_0[1]);

			ptsx.push_back(next_wp_1[0]);
			ptsy.push_back(next_wp_1[1]);

			ptsx.push_back(next_wp_2[0]);
			ptsy.push_back(next_wp_2[1]);

			// Change coordinate system to local car's reference frame
			for (int i = 0; i < ptsx.size(); i++){
				double shift_x = ptsx[i] - ref_x;
				double shift_y = ptsy[i] - ref_y;

				ptsx[i] = shift_x * cos(ref_yaw) + shift_y * sin(ref_yaw);
				ptsy[i] = -shift_x * sin(ref_yaw) + shift_y * cos(ref_yaw); 
			}

			// create the spline
			tk::spline path_spline_local;
			path_spline_local.set_points(ptsx, ptsy);

			// Fill the first spots with leftover waypoints
			for(int i = 0; i < previous_path_x.size(); i++)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			// Use pythagorean implementation to enforce speed limit
			double tangential_displacement = 30.0; // meters
			double perpendicular_displacement = path_spline_local(tangential_displacement); //Spline calculated
			double total_distance = sqrt(tangential_displacement * tangential_displacement + perpendicular_displacement * perpendicular_displacement);

			double x_add_on = 0.0;
			double N = total_distance / (0.02 * ref_vel_mps);
			for (int i = 0; i < num_waypoints_to_return - previous_path_x.size(); i++){
				double x_point = x_add_on + tangential_displacement / N;
				double y_point = path_spline_local(x_point);
				x_add_on = x_point;

				//convert the x_point and y_point to world_coordinates
				double x_point_world = x_point * cos(ref_yaw) - y_point * sin(ref_yaw) + ref_x;
				double y_point_world = x_point * sin(ref_yaw) + y_point * cos(ref_yaw) + ref_y;
				int num_added_vals = next_x_vals.size();
				if (num_added_vals > 2){
					double x_1 = next_x_vals[num_added_vals - 1];
					double y_1 = next_y_vals[num_added_vals - 1];
					double x_0 = next_x_vals[num_added_vals - 2];
					double y_0 = next_y_vals[num_added_vals - 2];

					double v_x_0 = (x_1 - x_0) / time_between_waypoints;
					double v_y_0 = (y_1 - y_0) / time_between_waypoints;

					double x_null_2 = x_1 + v_x_0 * time_between_waypoints;
					double y_null_2 = y_1 + v_y_0 * time_between_waypoints;

					double v_x_1 = (x_point_world - x_1) / time_between_waypoints;
					double v_y_1 = (y_point_world - y_1) / time_between_waypoints;

					double a_x_0 = (v_x_1 - v_x_0) / time_between_waypoints;
					double a_y_0 = (v_y_1 - v_y_0) / time_between_waypoints;
					double a_mag_0 = sqrt(a_x_0*a_x_0 + a_y_0*a_y_0);
					while(a_mag_0 > max_allowed_a){
						x_point_world = x_point_world + bring_back_ratio * (x_null_2 - x_point_world);
						y_point_world = y_point_world + bring_back_ratio * (y_null_2 - y_point_world);

						v_x_1 = (x_point_world - x_1) / time_between_waypoints;
						v_y_1 = (y_point_world - y_1) / time_between_waypoints;

						a_x_0 = (v_x_1 - v_x_0) / time_between_waypoints;
						a_y_0 = (v_y_1 - v_y_0) / time_between_waypoints;
						a_mag_0 = sqrt(a_x_0*a_x_0 + a_y_0*a_y_0);
					}

				}

				next_x_vals.push_back(x_point_world);
				next_y_vals.push_back(y_point_world);
			}

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
