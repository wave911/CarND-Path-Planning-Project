#ifndef SRC_PATHPLANNING_H_
#define SRC_PATHPLANNING_H_

#include <math.h>
#include <vector>

// For converting back and forth between radians and degrees.
class CPathPlanning {
private:
	std::vector<std::vector<double>> m_sensor_fusion;
	std::vector<double> m_map_waypoints_x,
						m_map_waypoints_y,
						m_map_waypoints_s,
						m_map_waypoints_dx,
						m_map_waypoints_dy,
						m_previous_path_x,
						m_previous_path_y;
  	double 	m_car_x,
			m_car_y,
			m_car_s,
			m_car_d,
			m_car_yaw,
			m_car_speed;
public:
	CPathPlanning();
	virtual ~CPathPlanning();

	void createTrajectory(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals, int lane, double ref_vel, double projecting_dist );
	bool isClose(int lane);
	int getLane(int lane, double projecting_dist);
	std::string hasData(std::string s);
	void updateCarData(std::vector<std::vector<double>> &sensor_fusion, std::vector<double> &previous_path_x, std::vector<double> &previous_path_y,
						double &car_x, double &car_y, double &car_s, double &car_d, double &car_yaw, double &car_speed);
	void updateWaypoints(double &x, double &y, float &s, float &d_x, float &d_y);
private:
	double penalizeLaneChange(int target_lane, int current_lane);
	double changeLane(int target_lane, int current_lane, double projecting_dist);
	double laneCost(int target_lane, int current_lane, double projecting_dist);
	double distance(double x1, double y1, double x2, double y2);
	int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
	int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
	std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
	std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

};



#endif /* SRC_PATHPLANNING_H_ */
