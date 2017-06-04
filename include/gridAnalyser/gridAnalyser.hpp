#include "arc_msgs/State.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include "arc_tools/coordinate_transform.hpp"
#include "arc_tools/timing.hpp"
#include "geometry_msgs/Transform.h"
#include <algorithm>
class gridAnalyser
{
public:
	//Constructor.
	gridAnalyser(const ros::NodeHandle &nh, std::string PATH_NAME);
	//Function which saves the incoming state.
	void getState (const arc_msgs::State::ConstPtr& arc_state);
	//Function which saves the incoming occupancy map.
	void getGridMap (const nav_msgs::OccupancyGrid::ConstPtr& grip_map);
	//Function which saves the tracking error.
	void getTrackingError(const std_msgs::Float64::ConstPtr& t_err);
	//Function to save from TXT-File to a Path copied from Moritz.
	void readPathFromTxt(std::string inFileName);
	//Function which produces inflated map.
	void createDangerZone(const nav_msgs::OccupancyGrid grip_map);
	//Funtion which calculates the n in row-major order from the cell index in (x, y).
	int convertIndex (int x, int y);
	//Function which inflates the obstacle around point (x, y) AND at index n.
	void inflate(int x, int y);
	void inflate(int n);
	//Funktion that projects a global point in the local system (grid index)
	int gridIndexOfGlobalPoint(geometry_msgs::Point P);
	//Function that calculates grid coordinates from index
	geometry_msgs::Vector3 convertIndex(const int i);
	//Comparison of NicoMap and dangerzone
	void compareGrids();
	//Evaluates what to do with obstacle at gridindex i
	void whattodo(const int i);
	//Publishes all 
	void publish_all();
	int indexOfDistanceFront(int i, float d);
	void takeTime();	
private:
	//Ros-Constants:.
	//NodeHandle.
	ros::NodeHandle nh_;
//Path publisher
ros::Publisher path_pub_;
	//Publisher for the boolean.
	ros::Publisher stop_pub_;
	//publisher for the danger grid.
	ros::Publisher danger_pub_;
	//Publisher of distance to nearest obstacle;
	ros::Publisher distance_to_obstacle_pub_;
	//Subscriber for the state.
	ros::Subscriber state_sub_;
	//Subscriber for the incoming map.
	ros::Subscriber grid_map_sub_;
	//Subscriber for the lateral tracking error.
	ros::Subscriber tracking_error_sub_;
	//Map from Nico
	nav_msgs::OccupancyGrid nico_map_;
	//Variable to store the path.
	nav_msgs::Path path_;
	//Variable to store number of cells in direction of travel (y).
	int height_;
	//variable to store the number of cells orthogonal to direction of travel (x). 
	int width_;
	//Variable to store the resolution. 
	float resolution_;
	//Variable to store actual velocity	
	float v_abs_;
	//Variable to store the Grid with inflated Tube around path.
	nav_msgs::OccupancyGrid	tube_map_;
	//Bool to store the stop (safe/unsafe). 
	bool stop_;
	std_msgs::Bool stop_msg_;
	//Distance to nearest Obstacle
	float obstacle_distance_;
	std_msgs::Float64 obstacle_distance_msg_;
	//Variable that stores the actual state
	arc_msgs::State state_;
	//Lenght of path 
	int n_poses_path_;
	//Actual distance of vehicle to path
	float tracking_error_;
	//Number of Cells
	int n_cells_;
	//Counter for number of seen energency cells 
	int crit_counter_;
	//Bool that decides if state or grid callback is executed
	bool jumper_;
	//Bools so that state and grid are initialized. if botj true we can stat analysing
	bool state_init_;
	bool grid_init_;
	//Braking velocity
	float braking_distance_;
	//Diastance of obstacle to actuate emergency stop.
	float emergency_distance_;
	//Timer
	arc_tools::Clock BigBen_;
	arc_tools::Clock BigBen2_;
	ros::Time begin_time_;
	bool big_ben_started_;
	bool distance_manually_;
};
