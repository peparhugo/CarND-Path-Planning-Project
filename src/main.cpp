<<<<<<< HEAD
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
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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

bool checkLane(vector<int> target_lanes, int laneToCheck){
  for (int i = 0; i < target_lanes.size(); i++){
    int target_lane = target_lanes.at(i);
    
    if (target_lane == laneToCheck){
      return true;
    }
  }
  return false;
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

  int lane = 1;
  double ref_vel = 0.0;
  

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
		          //lane change flag
            	bool current_lane_change = false;
            
            	//lane change test, if true then changing lanes is true
            	if (abs( car_d - end_path_d ) >= 0.10){
              	current_lane_change = true;
            	}


		        cout << "Lane change: " << current_lane_change << "\n";
          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            //previous paths length
            int prev_size = previous_path_x.size();
            
            //road paraemeters
            int lane_count = 3;
            double lane_width = 4.0;
            double lane_center = lane_width/2.0;

            //set safe following distance based on car's speed
            //is used to set how far ahead car is planning
            double safe_follow_dist;
            if (car_speed > 0.0){
              safe_follow_dist = 3.0*car_speed*1.60934;
              //safe_follow_dist = 15.0;
            } else {
              safe_follow_dist = 15.0;
            }
            
            //set min safe follow distance
            double min_safe_follow = 15.0;


            //if safge follow distance is less than min, set to min
            if (safe_follow_dist < min_safe_follow){
              safe_follow_dist = min_safe_follow;
            }

          	json msgJson;

	          //if previous path size is zero set car s to end path
            if(prev_size == 0){
              end_path_s = car_s;
            }




            cout << "Current s position: " << car_s << "\n";
            
            //too clase flag intialized and set to false
            bool too_close = false;

            //initialize container vectors for ahead and behind cars
            //will only consider closest car ahead and behind
            vector<int> other_cars_lane;
            vector<double> other_cars_s;
            vector<double> other_car_rel_vel;
            vector<double> other_car_rel_s;
            vector<double> other_car_rel_time;
            vector<int> target_lanes;
            vector <double> closest_car_ahead_by_lane_rel_vel;
            vector <double> closest_car_ahead_by_lane_rel_time;
            vector <double> closest_car_ahead_by_lane_rel_s;
            vector <double> closest_car_behind_by_lane_rel_vel;
            vector <double> closest_car_behind_by_lane_rel_time;
            vector <double> closest_car_behind_by_lane_rel_s;
            vector <bool> closest_car_ahead_by_lane_faster;
            vector <bool> closest_car_behind_by_lane_faster;
            
            //set targets lanes 
            //eg. if far right lane only consider lane change to left lane
            if(lane==0){
              target_lanes.push_back(1);
            } else if(lane==lane_count-1){
              target_lanes.push_back(lane_count-2);
            } else {
              target_lanes.push_back(lane-1);
              target_lanes.push_back(lane+1);
            }

            //set default value for each lane
            for(int i = 0; i < lane_count; i++){
              closest_car_ahead_by_lane_rel_vel.push_back(0.0);
              closest_car_ahead_by_lane_rel_time.push_back(30.0);
              closest_car_ahead_by_lane_rel_s.push_back(9999.0);
              closest_car_behind_by_lane_rel_vel.push_back(0.0);
              closest_car_behind_by_lane_rel_time.push_back(30.0);
              closest_car_behind_by_lane_rel_s.push_back(-9999.0);
              closest_car_ahead_by_lane_faster.push_back(true);
              closest_car_behind_by_lane_faster.push_back(true);
            }

            //loop through sensor fusion information
            for (int i = 0; i < sensor_fusion.size(); i++){
              //other car's d value
              double d = sensor_fusion[i][6];
              int other_lane;
              //loop through lanes and categorize lane position
              for(int k=0; k < lane_count; k++){
                double lane_min = lane_width*double(k);
                double lane_max = lane_width*double((k+1));
                if (d <= lane_max && d > lane_min){
                  other_lane = k;
                }
              }
              
              //set other car's state
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];
              double car_rel_vel = (car_speed - check_speed)*1.60934;
              if (car_rel_vel = 0.0){
                car_rel_vel = 0.000001;
              }
              double car_speed_adjust = 0.0;
              //prevent divide by zero later in the code
              if (car_speed == 0.0) {
                car_speed_adjust = 0.000001;
              }
              //get current car's relative distance
              double car_rel_s = check_car_s - car_s;
              
              //test if car is ahead but past 6945.554, s resets to zero
              if (abs(check_car_s - (car_s-6945.554))<=150){
                car_rel_s = check_car_s - (car_s-6945.554);
              }
              bool other_car_faster;
              bool other_car_ahead;
              
              //categorize vehicle as ahead or behind
              //there is error in this categorization
              //later in code it considers behind vehicles that are very close
              //behind veichles follow distance is about 40
              if (car_rel_s > 0.0){
                other_car_ahead = true;
              } else {
                other_car_ahead = false;
              }

              if(car_rel_vel < 0.0){
                other_car_faster = true;
              } else {
                other_car_faster = false;
              }
              
              //store each lanes closest behind and ahead vehicles states for lane targeting
              if (other_car_ahead){
                double test_ahead = closest_car_ahead_by_lane_rel_s[other_lane];
                if (test_ahead > car_rel_s){
                  closest_car_ahead_by_lane_rel_s[other_lane]=car_rel_s;
                  closest_car_ahead_by_lane_rel_vel[other_lane]=car_rel_vel;
                  closest_car_ahead_by_lane_rel_time[other_lane]=car_rel_s/(car_speed*1.60934+car_speed_adjust);
                  closest_car_ahead_by_lane_faster[other_lane]=other_car_faster;
                }
              } else {
                double test_behind = closest_car_behind_by_lane_rel_s[other_lane];
                  if ( test_behind < car_rel_s){
                  closest_car_behind_by_lane_rel_s[other_lane]=car_rel_s;
                  closest_car_behind_by_lane_rel_vel[other_lane]=car_rel_vel;
                  closest_car_behind_by_lane_rel_time[other_lane]=abs(car_rel_s)/(car_speed*1.60934+car_speed_adjust);
                  closest_car_behind_by_lane_faster[other_lane]=other_car_faster;  
                }
                }
              }

             //initialize target lane variables
              //ideal lane is best lane to be in, car ahead is faster
              int ideal_lane = -1;
              //more time ahead in other lane but does not have to being going faster
              //than our car's speed
              int acceptable_lane = -1;
              bool switch_lanes = false;
              double max_rel_time = 0.0;
              double rel_time = 0.0;
              double current_lane_rel_time = 0.0;

            //determine each lane situation
            for( int i=lane_count-1; i >= 0; i--){
              //access closest vehicles relative states to our car
              double b_car_rel_s = closest_car_behind_by_lane_rel_s[i];
              double b_car_rel_vel = closest_car_behind_by_lane_rel_vel[i];
              double b_car_rel_time = closest_car_behind_by_lane_rel_time[i];
              double a_car_rel_s=closest_car_ahead_by_lane_rel_s[i];
              double a_car_rel_vel = closest_car_ahead_by_lane_rel_vel[i];
              double a_car_rel_time = closest_car_ahead_by_lane_rel_time[i];


              cout << "Lane: " << i << " Ahead Dist: " << a_car_rel_s << "Behind Car: " << b_car_rel_s << "\n";
            //if car is still accelerating from start then don't determine lane actions
            if (car_speed > 25.0){
              //current lanes state
              if (lane == i){
                current_lane_rel_time = a_car_rel_time;
                cout << "This Lane ahead car time: " << a_car_rel_time << "\n";
                //set max time ahead if greater than current max time
                if(current_lane_rel_time > max_rel_time){
                  max_rel_time = current_lane_rel_time;
                }
                //if car ahead is too close, or behind with certain time, then set too close flag true
                if (a_car_rel_time < 0.5 && a_car_rel_time > 0.0){
                  too_close = true;
                }
                
              }
              bool checked_lane = checkLane(target_lanes,i);
              cout << "Lane " << lane << " Check Lane Status: " << checked_lane << "\n";
              //if lane change flag is true, then dont consider changing lanes
              if (current_lane_change==false){
                //if ahead car in other lane is fater and safe distance for change, target lane as ideal lane
                if(  a_car_rel_time < 0.0 && abs(b_car_rel_s) > min_safe_follow && abs(a_car_rel_s) > min_safe_follow ){
                  if(checked_lane){
                    ideal_lane = i;
                    rel_time = a_car_rel_time;
                    max_rel_time = rel_time;
                    
                  }
                }
                //if car ahead is not faster but greater time in that lane, target this lane as acceptable
                if( ideal_lane == -1 && a_car_rel_time > max_rel_time && abs(b_car_rel_s) > min_safe_follow && abs(a_car_rel_s) > min_safe_follow){
                  if(checked_lane){
                    acceptable_lane = i;
                    rel_time = a_car_rel_time;
                    max_rel_time = rel_time;
                    
                  }
                }
              }
              
            }
          }  
            cout << "Ideal Lane : " << ideal_lane << " Accept Lane: " << acceptable_lane << "\n";
            
            //set target lane from states considered above
            //only target lane if 0.15 seconds gained
            //else stay in current lane
            if(ideal_lane==-1 && acceptable_lane ==-1){
              lane = lane; 
            } else if(ideal_lane == -1 && rel_time - current_lane_rel_time > 0.15) {
              lane = acceptable_lane;
            } else if((rel_time - current_lane_rel_time > 0.15 || rel_time <0.0) && ideal_lane !=-1) {
              lane = ideal_lane;
            }
            cout << "Target Lane :::" << lane << "\n";
            //change target speed
            //will always change based on conditions
            //Note: speed can be violated with this and lane change
            //since changing lanes accelerates the vehicle
            
            if ((current_lane_rel_time > 0.5 &&  car_speed < 46.0)){
              ref_vel += .048;
            } else if (current_lane_rel_time <= 0.25 && current_lane_rel_time != 0.0) {
	            ref_vel -= 0.122;
	          } else if(too_close || car_speed > 47.5){
              ref_vel -= .048;
            } else if (car_speed < 26.0) {
	            ref_vel += .112;
	          }
            cout << "Current lane rel time: " << current_lane_rel_time << "\n";

            //initialize vectors to store next paths behavioral points to create spline from
          	vector<double> path_points_x;
          	vector<double> path_points_y;

            //initialize reference values for car's state
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            //if the previous size is less than 2 then use current state
            //for generating next path
            //else use the last 2 previous path points as reference 
            if (prev_size < 2){
              //create previous car position
              double prev_ref_x = ref_x - cos(car_yaw);
              double prev_ref_y = ref_y - sin(car_yaw);

              //store prev and current x
              path_points_x.push_back(prev_ref_x);
              path_points_x.push_back(ref_x);

              //store prev and current y
              path_points_y.push_back(prev_ref_y);
              path_points_y.push_back(ref_y);
            } else {
              //get prev path's last position
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              //get previous path's last position
              double prev_ref_x = previous_path_x[prev_size-2];
              double prev_ref_y = previous_path_y[prev_size-2];

              //calculate reference yaw
              ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);

              //store second last and last positions for next path
              path_points_x.push_back(prev_ref_x);
              path_points_x.push_back(ref_x);

              path_points_y.push_back(prev_ref_y);
              path_points_y.push_back(ref_y);
            }
            cout << lane << "Target Lane \n";

            //set target lane for next path
            double d = 2.0 + 4.0 * lane;
            cout << d << "Target D \n";

            //set next path points at 30, 60 and 90 on target lane
            vector<double> next_wp30 = getXY(end_path_s+30, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp60 = getXY(end_path_s+60, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp90 = getXY(end_path_s+90, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            //store 30, 60, 90 x and y positions for next path
            path_points_x.push_back(next_wp30[0]);
            path_points_y.push_back(next_wp30[1]);

            path_points_x.push_back(next_wp60[0]);
            path_points_y.push_back(next_wp60[1]);

            path_points_x.push_back(next_wp90[0]);
            path_points_y.push_back(next_wp90[1]);        
                        
            //convert x and y points to relative positions based on prev path end point
            for (int i = 0; i < path_points_x.size(); i++){
              double rel_shift_x = path_points_x[i] - ref_x;
              double rel_shift_y = path_points_y[i] - ref_y;

              path_points_x[i] = (rel_shift_x * cos(0-ref_yaw) - rel_shift_y * sin(0-ref_yaw));
              path_points_y[i] = (rel_shift_x * sin(0-ref_yaw) + rel_shift_y * cos(0-ref_yaw));

            }

            tk::spline s;
            //create spline to generate smooth next path
            s.set_points(path_points_x, path_points_y);

            //initialize vector to store next path points
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            //add previous path's points to next path
            for (int i = 0; i < previous_path_x.size(); i++){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            //set additional distance from last path's end point
            double target_x = safe_follow_dist;
            //get y distance from spline based on target x
            double target_y = s(target_x);
            //calculate distance
            double target_dist = sqrt(target_x*target_x + target_y*target_y);
            //increment as each new point is generated for path
            double x_add_on = 0;

            //generate a maximum of point for path
            for (int i = 1; i <= 50 - previous_path_x.size(); i++){
              //break target distance into N numbner of uniform sections of 0.2 seconds
              double N = (target_dist / (0.02*ref_vel*1.60934));

              //next x distance
              double x_point = x_add_on + (target_x)/N;

              //y value from x position based on spline
              double y_point = s(x_point);

              //set x offset to current point
              x_add_on = x_point;

              //store next x and y values
              double x_ref = x_point;
              double y_ref = y_point;

              // convert from relative position back to global coordinates

              x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

              //add reference x and y to get global positions
              x_point += ref_x;
              y_point += ref_y;

              //store points for next path
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);

            }

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
















































































||||||| merged common ancestors
=======
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
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
















































































>>>>>>> fcc97984c7ff682ac9463e75cc719c7ed1e3ef3e
