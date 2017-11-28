#include "PathPlanning.h"
#include "spline.h"
#include <iostream>

using namespace std;

#define PATH_LENGTH 50
#define FACTOR 0.02
#define REF_DISTANCE 20

constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

CPathPlanning::CPathPlanning() {

}

CPathPlanning::~CPathPlanning() {

}

void CPathPlanning::updateCarData(std::vector<std::vector<double>> &sensor_fusion, std::vector<double> &previous_path_x, std::vector<double> &previous_path_y,
									double &car_x, double &car_y, double &car_s, double &car_d, double &car_yaw, double &car_speed) {
	m_car_x = car_x;
	m_car_y = car_y;
	m_car_s = car_s;
	m_car_d = car_d;
	m_car_yaw = car_yaw;
	m_car_speed = car_speed;
	m_sensor_fusion = sensor_fusion;
	m_previous_path_x = previous_path_x;
	m_previous_path_y = previous_path_y;
}
void CPathPlanning::updateWaypoints(double &x, double &y, float &s, float &d_x, float &d_y) {
  	m_map_waypoints_x.push_back(x);
  	m_map_waypoints_y.push_back(y);
  	m_map_waypoints_s.push_back(s);
  	m_map_waypoints_dx.push_back(d_x);
  	m_map_waypoints_dy.push_back(d_y);
}

void CPathPlanning::createTrajectory(vector<double> &next_x_vals, vector<double> &next_y_vals, int lane, double ref_vel, double projecting_dist ) {
	vector<double> ptsx;
	vector<double> ptsy;
	tk::spline s;

	double ref_x = m_car_x;
	double ref_y = m_car_y;
	double ref_yaw = deg2rad(m_car_yaw);
	int prev_size = m_previous_path_x.size();
	vector<double> next_mp0 = getXY(m_car_s + 30, (2 + 4 * lane), m_map_waypoints_s, m_map_waypoints_x, m_map_waypoints_y);
	vector<double> next_mp1 = getXY(m_car_s + 60, (2 + 4 * lane), m_map_waypoints_s, m_map_waypoints_x, m_map_waypoints_y);
	vector<double> next_mp2 = getXY(m_car_s + 90, (2 + 4 * lane), m_map_waypoints_s, m_map_waypoints_x, m_map_waypoints_y);

	if(prev_size < 2)
	{
		double prev_car_x = m_car_x - cos(m_car_yaw);
		double prev_car_y = m_car_y - sin(m_car_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(m_car_x);
		ptsy.push_back(prev_car_y);
		ptsy.push_back(m_car_y);
	}
	else
	{
		ref_x = m_previous_path_x[prev_size-1];
		ref_y = m_previous_path_y[prev_size-1];

		double ref_x_prev = m_previous_path_x[prev_size - 2];
		double ref_y_prev = m_previous_path_y[prev_size - 2];

		ref_yaw = atan2(ref_y-ref_y_prev, ref_x - ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}

	ptsx.push_back(next_mp0[0]);
	ptsx.push_back(next_mp1[0]);
	ptsx.push_back(next_mp2[0]);

	ptsy.push_back(next_mp0[1]);
	ptsy.push_back(next_mp1[1]);
	ptsy.push_back(next_mp2[1]);

	for (int i = 0; i < ptsx.size();i++)
	{
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
	}

	s.set_points(ptsx, ptsy);

	for (int i = 0; i < m_previous_path_x.size();i++)
	{
		next_x_vals.push_back(m_previous_path_x[i]);
		next_y_vals.push_back(m_previous_path_y[i]);
	}

	double target_x = projecting_dist;
	double target_y = s(target_x);
	double target_dist = sqrt(target_x * target_x + target_y * target_y);

	double x_add_on = 0;

	for (int i = 1; i <= (PATH_LENGTH - m_previous_path_x.size()); i++)
	{
		double N = (target_dist/(FACTOR * ref_vel/2.24));
		double x_point = x_add_on + target_x/N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
		y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
	}
}

bool CPathPlanning::isClose(int lane) {

	bool too_close = false;
	for (int i = 0; i < m_sensor_fusion.size(); i++) {
		float d = m_sensor_fusion[i][6];
		if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
		{
			double vx = m_sensor_fusion[i][3];
			double vy = m_sensor_fusion[i][4];
			double check_speed = sqrt(vx * vx + vy * vy);
			double check_car_s = m_sensor_fusion[i][5];

			check_car_s += ((double)m_previous_path_x.size() * FACTOR * check_speed);

			if((check_car_s > m_car_s) && ((check_car_s - m_car_s) < REF_DISTANCE)) {
				too_close = true;
			}
		}
	}
	return too_close;
}

double CPathPlanning::penalizeLaneChange(int target_lane, int current_lane) {
	double cost = 0;
	if (target_lane != current_lane)
		cost = 1.0;
	return cost;
}

double CPathPlanning::changeLane(int target_lane, int current_lane, double projecting_dist) {
	double cost = 0;

	if (target_lane == current_lane)
	{
		cost = 10;
		return cost;
	}

	if (target_lane < 0 || target_lane > 2) {
		cost = 9999.0;
		return cost;
	}

	for (int i = 0; i < m_sensor_fusion.size(); i++) {
		float d = m_sensor_fusion[i][6];
		if (d < (2 + 4 * target_lane + 2) && d > (2 + 4 * target_lane - 2))
		{
			double vx = m_sensor_fusion[i][3];
			double vy = m_sensor_fusion[i][4];
			double check_speed = sqrt(vx * vx + vy * vy);
			double check_car_s = m_sensor_fusion[i][5];

			check_car_s += ((double)m_previous_path_x.size() * FACTOR * check_speed);

			if((check_car_s > m_car_s) && ((check_car_s - m_car_s)< projecting_dist)) {
			cost = 999.0;

			}
			else if ((check_car_s < m_car_s) && ((m_car_s - check_car_s)< projecting_dist))
			{
				cost = 10.0;

				if (check_speed > m_car_speed)
				{
				  cost = 9999.0;
				}
			}

		}
	}
	return cost;
}

double CPathPlanning::laneCost(int target_lane, int current_lane, double projecting_dist) {
	double cost = 0;

	cost += penalizeLaneChange(target_lane, current_lane) + \
			changeLane(target_lane, current_lane, projecting_dist);

	return cost;
}

int CPathPlanning::getLane(int lane, double projecting_dist) {
	int nextLane = lane;

	double cost_left = laneCost(lane - 1, lane, projecting_dist);
	double cost_straigth = laneCost(lane, lane, projecting_dist);
	double cost_rigth = laneCost(lane + 1, lane, projecting_dist);

	if ((cost_rigth < cost_straigth) && (cost_rigth < cost_left))
		nextLane = lane + 1;
	else if ((cost_left < cost_straigth))
		nextLane = lane - 1;

	return nextLane;
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string CPathPlanning::hasData(string s) {
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

double CPathPlanning::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int CPathPlanning::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

int CPathPlanning::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
vector<double> CPathPlanning::getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
vector<double> CPathPlanning::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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




