#include "../include/gridAnalyser/gridAnalyser.hpp"

float QUEUE_LENGTH=10;
float FOS_DANGERGRID=1;
float FOS_BRAKING_DISTANCE=10;
float TOTAL_WIDTH=1.6;
float DISTANCE_LASER_REAR_AXIS=1;
float DISTANCE_FRONT_TO_REAR_AXIS=3;
float NUMBER_OF_EMERGENCY_CELLS=2;	//number of times a cell in less than braking dist has to appear in order to actuate a notstop.
float CRITICAL_OBSTACLE_DISTANCE=2;
float LENGHT_CORRECTION_PATH_GRIDANALYSER=3;	//for displacement of inflated path
float K1_LAD_LASER=2;
float K2_LAD_LASER=5;
float EMERGENCY_DISTANCE_LB=1;	//To test empirically
float EMERGENCY_DISTANCE_UB=6;	//To test empirically
std::string STOP_LASER_TOPIC="laser_stop";
std::string STATE_TOPIC="state";
std::string OBSTACLE_MAP_TOPIC="gridmap";
std::string DANGER_GRID_TOPIC="danger_grid";
std::string OBSTACLE_DISTANCE_TOPIC="distance_to_obstacle";


//Constructor
gridAnalyser::gridAnalyser(const ros::NodeHandle &nh): nh_(nh)
{
	nh.getParam("/control/EMERGENCY_DISTANCE_LB",EMERGENCY_DISTANCE_LB);
	nh.getParam("/control/EMERGENCY_DISTANCE_UB",EMERGENCY_DISTANCE_UB);	
	nh.getParam("/erod/DISTANCE_FRONT_TO_REAR_AXIS",DISTANCE_FRONT_TO_REAR_AXIS);	
	nh.getParam("/control/LENGHT_CORRECTION_PATH_GRIDANALYSER",LENGHT_CORRECTION_PATH_GRIDANALYSER);
	nh.getParam("/control/NUMBER_OF_EMERGENCY_CELLS",NUMBER_OF_EMERGENCY_CELLS);
	nh.getParam("/general/QUEUE_LENGTH",QUEUE_LENGTH);
	nh.getParam("/erod/TOTAL_WIDTH",TOTAL_WIDTH);
	nh.getParam("/safety/FOS_BRAKING_DISTANCE",FOS_BRAKING_DISTANCE);
	nh.getParam("/safety/FOS_DANGERGRID",FOS_DANGERGRID);
	nh.getParam("/erod/DISTANCE_LASER_REAR_AXIS",DISTANCE_LASER_REAR_AXIS);
	nh.getParam("/safety/CRITICAL_OBSTACLE_DISTANCE",CRITICAL_OBSTACLE_DISTANCE );
	nh.getParam("/topic/OBSTACLE_DISTANCE",OBSTACLE_DISTANCE_TOPIC);
	nh.getParam("/topic/STATE",STATE_TOPIC);
	nh.getParam("/topic/OBSTACLE_MAP",OBSTACLE_MAP_TOPIC);
	nh.getParam("/topic/DANGER_GRID",DANGER_GRID_TOPIC);
	nh.getParam("/topic/LASER_STOP",STOP_LASER_TOPIC);
	nh.getParam("/control/K1_LAD_LASER",K1_LAD_LASER);
	nh.getParam("/control/K2_LAD_LASER",K2_LAD_LASER);

	//Initialisation.
/*	state_.pose.pose.position.x=0;
	state_.pose.pose.position.y=0;
	state_.pose.pose.position.z=0;
	state_.pose.pose.orientation.x=0;
	state_.pose.pose.orientation.y=0;
	state_.pose.pose.orientation.z=0;
	state_.pose.pose.orientation.w=1;
	state_.pose_diff=0;
	state_.current_arrayposition=0;
	state_.stop=false;
	
	n_cells_=48000;
	width_=120;
	height_=400;
	resolution_=0.1;
*/
	jumper_=true;	//Variabel that gives alternation btw state and grid callback functions
	grid_init_=false;
	state_init_=false;
	//Publsiher. 
	stop_pub_=nh_.advertise<std_msgs::Bool>(STOP_LASER_TOPIC,QUEUE_LENGTH);
	distance_to_obstacle_pub_=nh_.advertise<std_msgs::Float64>(OBSTACLE_DISTANCE_TOPIC,QUEUE_LENGTH);
	danger_pub_=nh_.advertise<nav_msgs::OccupancyGrid>(DANGER_GRID_TOPIC,QUEUE_LENGTH);			//to visualize danger zone with rviz
	//Subscriber.
	grid_map_sub_=nh_.subscribe(OBSTACLE_MAP_TOPIC, QUEUE_LENGTH, &gridAnalyser::getGridMap, this);
	state_sub_=nh_.subscribe(STATE_TOPIC, QUEUE_LENGTH, &gridAnalyser::getState, this);
	//Read path.
	readPathFromTxt("/home/moritz/.ros/Paths/ObstacleDetection_teach.txt");
	std::cout<<"GRID ANALYSER: Constructor"<<std::endl;
}

//Callback Function which processes incoming state
void gridAnalyser::getState (const arc_msgs::State::ConstPtr& arc_state)
{
if(jumper_==false)
{
/*
std::cout<<"Position: "<<_state->pose.pose.position.x<<" "<<arc_state->pose.pose.position.y<<" "<<arc_state->pose.pose.position.z<<std::endl;
std::cout<<"Quatrnions: "<<arc_state->pose.pose.orientation.x<<" "<<arc_state->pose.pose.orientation.y<<" "<<arc_state->pose.pose.orientation.z<<" "<<arc_state->pose.pose.orientation.w<<std::endl;
std::cout<<"Velocities: "<<arc_state->pose_diff.twist.linear.x<<" "<<arc_state->pose_diff.twist.linear.y<<" "<<arc_state->pose_diff.twist.linear.z<<std::endl;
std::cout<<"Angular velocities: "<<arc_state->pose_diff.twist.angular.x<<" "<<arc_state->pose_diff.twist.angular.y<<" "<<arc_state->pose_diff.twist.angular.z<<std::endl;
*/


	state_=*arc_state;
	//LOOP
	float x_now=state_.pose.pose.position.x;
	float y_now=state_.pose.pose.position.y;
	float x_path=path_.poses[state_.current_arrayposition].pose.position.x;
	float y_path=path_.poses[state_.current_arrayposition].pose.position.y;
	tracking_error_=fabs(arc_tools::globalToLocal(path_.poses[state_.current_arrayposition-1].pose.position,state_).y);
	braking_distance_=pow(state_.pose_diff*3.6/10,2)/2*FOS_BRAKING_DISTANCE;	//Bremsweg empyrisch zu ermitteln
	emergency_distance_=std::max(EMERGENCY_DISTANCE_LB,braking_distance_);
	emergency_distance_=std::min(EMERGENCY_DISTANCE_UB,braking_distance_);
	state_init_=true;
	if(grid_init_==true&&state_init_==true)
	{
		createDangerZone (nico_map_);
		compareGrids();
		publish_all();
	}
	jumper_=true;
}
}
//SAFE NICOMAP
void gridAnalyser::getGridMap(const nav_msgs::OccupancyGrid::ConstPtr& grid_map)
{
if(jumper_==true)
{
	nico_map_=*grid_map;
	//LOOP

	//Calc tracking error.
	//std::cout<<"GRID ANALYSER: Loop NewGrid"<<std::endl;
	float x_now=state_.pose.pose.position.x;
	float y_now=state_.pose.pose.position.y;
	float x_path=path_.poses[state_.current_arrayposition].pose.position.x;
	float y_path=path_.poses[state_.current_arrayposition].pose.position.y;
	tracking_error_=fabs(arc_tools::globalToLocal(path_.poses[state_.current_arrayposition-1].pose.position,state_).y);
	//Jetzt lateral error selber berechnet und ist noch Fehler HINTERACHSE

	n_cells_=nico_map_.info.height*nico_map_.info.width;	//Easy access to this parameter
	width_=grid_map->info.width;					//Easy access to this parameter
	height_=grid_map->info.height;				//Easy access to this parameter
	resolution_=grid_map->info.resolution; 			//Easy access to this parameter
	grid_init_=true;
	if(grid_init_==true&&state_init_==true)
	{
		createDangerZone (nico_map_);
		compareGrids();
		publish_all();
	}
	jumper_=false;
}
}
//DANGERZONE (Schlauch um Pfad)
void gridAnalyser::createDangerZone (const nav_msgs::OccupancyGrid grid_map)
{	
	tube_map_=grid_map;
	for (int i=0;i<n_cells_;i++) tube_map_.data[i]=0;
	//Inflate path
	for(int i=state_.current_arrayposition; i<indexOfDistanceFront(state_.current_arrayposition,5); i++) inflate(gridIndexOfGlobalPoint(path_.poses[i].pose.position));	//statt 5 : state_.current_arrayposition,K2_LAD_LASER+K1_LAD_LASER*braking_distance_
}

//INFLATE XY----------------------------------------------------------------------------------------------------------------------------
void gridAnalyser::inflate(int x, int y)
{	
	//Displace less and less with increasing distance
	float distance=sqrt( pow(y+(width_/2.0),2) + pow(x-(height_/2.0),2) )*resolution_; 
	float displacement=tracking_error_*(1-distance/LENGHT_CORRECTION_PATH_GRIDANALYSER);	//max(tracking_error_*(1-distance/LENGHT_CORRECTION_PATH_GRIDANALYSER),0);
	if (displacement<0) displacement=0;
	if(displacement!=0)
	{
		bool left=false;
		if ( arc_tools::globalToLocal(path_.poses[state_.current_arrayposition].pose.position,state_).y >=0) left=true;
		if (left) y-=round(displacement/resolution_);
		else y+=round(displacement/resolution_);
	}
	//Choose radius
	float Radius_float=((TOTAL_WIDTH/2)*FOS_DANGERGRID)+tracking_error_;	//Radius constant over tube.
	if((x>0) && x<height_ && y<0 && (y>-width_))
	{	
		int R=round(Radius_float)/resolution_;
		for(int i=(x-R); i<=(x+R); i++)
		{
			for(int j=(y-R); j<=(y+R); j++)
			{	
				//calculates dinstance squared between (i, j) and (x, y).
				float d=(((x-i)*(x-i))+((y-j)*(y-j)));
				if((i>0) && (i<height_) && (j<0) && j>-width_ && (d<(R*R)))
				{
					//If (i, j) is on the map and the distance to (x,y) is smaller than the defined.
					tube_map_.data[convertIndex(i, j)]=100;
				}
			}
		}
	}
}
//INFLATEINDEX
void gridAnalyser::inflate(int n)
{	
	geometry_msgs::Vector3 p=convertIndex(n);
	inflate(p.x,p.y);
}

//GRIDCOMPARE
void gridAnalyser::compareGrids()
{	int counter=0;
	int j=0;
	int distance_old=100;
	for(int i=0; i<n_cells_; i++)
	{	
		int a=tube_map_.data[i];
		int b=nico_map_.data[i];
		if ((a!=0)&&(b!=0))
		{	counter++;				//Counts matching cells.
			geometry_msgs::Vector3 p=convertIndex(i);
			float distance_new=sqrt( pow(p.y+(width_/2.0),2) + pow(p.x-(height_/2.0),2) )*resolution_;		//Objektdistanz bezüglich gridMittelpunkt			
			if (distance_new<distance_old)
			{
				distance_old=distance_new;
				j=i;
			}
		}
	}

	if (counter!=0)		//Falls es minestens eine grid Zelle gibt, die obstacle enthält UND im gefahrbereich ist
	{
		std::cout<<"GRID ANALYSER: Etwas auf dem Weg!"<<std::endl;
		whattodo(j);
	}
	else 
	{	
		crit_counter_=0;
		stop_=0;
		obstacle_distance_=100;
	}	
}	
//CONVERT
int gridAnalyser::convertIndex(int x, int y)
{	
	int n=(-y-1+width_*(x-1));	
	return n;
}
geometry_msgs::Vector3 gridAnalyser::convertIndex(const int i)
{	
	int x[2];
	x[1]=-(i%width_)-1;	//y-richtung zelle
	x[0]=int(i/width_)+1;	//x-richtung zelle
	geometry_msgs::Vector3 p;
	p.x=x[0];
	p.y=x[1];
	p.z=0;
	return p;
}

int gridAnalyser::gridIndexOfGlobalPoint(geometry_msgs::Point P)	//Global Point given with x up y right
{	

geometry_msgs::Point N;
	int n=-1;		//Damit wenn es geändert wird gut, ansonsten wird in späteren schritten n=-1 durch die if eliminiert
	geometry_msgs::Point local_msg=arc_tools::globalToLocal(P,state_);
	float x_local=local_msg.x;	//Translation von Nico gemacht von Laser zu rearaxis 
	float y_local=local_msg.y;
	if(	(round(y_local/resolution_)<width_/2) && (round(y_local/resolution_)>(-width_/2)) && 
		(round(x_local/resolution_)>-height_/2) && (round(x_local/resolution_)<height_/2)  )
	{
      	n = round(round(-y_local/resolution_)+round(x_local/resolution_)*width_+width_/2+(height_/2)*width_);
	} 
	return n;
}


//WHATTODO
void gridAnalyser::whattodo(const int i)
{	
	//std::cout<<"Index: "<<i<<std::endl;
	geometry_msgs::Vector3 p=convertIndex(i);
	//std::cout<<"Coordiantes: "<<p.x<<" "<<p.y<<std::endl;
	obstacle_distance_=sqrt(pow(p.x-height_/2,2)+pow(-p.y-width_/2,2))*resolution_+DISTANCE_FRONT_TO_REAR_AXIS;	//Object diastance to forntest point of car.	
		if(obstacle_distance_<emergency_distance_)
		{
			crit_counter_++;
			if(crit_counter_>=NUMBER_OF_EMERGENCY_CELLS)
			{
				std::cout<<"GRID ANALYSER: NOTSTOPP!"<<std::endl;
				stop_=1;
			}
			else
			{
				stop_=0;
				std::cout<<"GRID ANALYSER: crit_counter bei "<<crit_counter_<<std::endl;
			}
			
		}
		else
		{	
			crit_counter_=0;
			stop_=0;
			std::cout<<"GRID ANALYSER: LANGSAMER!"<<std::endl;
		}
}


void gridAnalyser::publish_all()
{
stop_msg_.data=stop_;
stop_pub_.publish(stop_msg_);

obstacle_distance_msg_.data=obstacle_distance_;
distance_to_obstacle_pub_.publish(obstacle_distance_msg_);

danger_pub_.publish(tube_map_);	//Only for visualisation rviz.
}

//READPATH
void gridAnalyser::readPathFromTxt(std::string inFileName)
{
	std::fstream fin; 	
	fin.open (inFileName.c_str());
	if(!fin.is_open())
		{
		std::cout << "GRID ANALYSER: Fehler beim Oeffnen von " <<inFileName << std::endl;
		}
	//Length of File.
	fin.seekg(-2,fin.end); //-2 to cut off the last |.
	int length = fin.tellg();
	fin.seekg (0, fin.beg);
	char * file = new char [length];
	fin.read (file,length);
	std::istringstream stream(file,std::ios::in);
	delete[] file;	
	fin.close () ; //Close.
	int i=0;
	int j;	
	geometry_msgs::PoseStamped temp_pose;
	while(!stream.eof()&& i<length)
		{
		geometry_msgs::PoseStamped temp_pose;	
		path_.poses.push_back(temp_pose);
		stream>>j;
		stream>>path_.poses[j-1].pose.position.x;
		stream>>path_.poses[j-1].pose.position.y;
		stream>>path_.poses[j-1].pose.position.z;	
		stream.ignore (300, '|');
		i++;	
		}
	n_poses_path_=i;

}

geometry_msgs::Point gridAnalyser::GlobalToLocal(geometry_msgs::Point global)
{
	//Translatation
	Eigen::Vector3d glob=arc_tools::transformPointMessageToEigen(global);
	Eigen::Vector3d stat= arc_tools::transformPointMessageToEigen(state_.pose.pose.position);
	Eigen::Vector3d temp=glob-stat;
	//Rotation
	Eigen::Vector4d quat=arc_tools::transformQuatMessageToEigen(state_.pose.pose.orientation);
	Eigen::Vector3d euler=arc_tools::transformEulerQuaternionVector(quat);
	Eigen::Matrix3d R=arc_tools::getRotationMatrix(euler);
	Eigen::Matrix3d T=R.transpose();
	Eigen::Vector3d local=T*temp;
	geometry_msgs::Point local_msg=arc_tools::transformEigenToPointMessage(local);
	return local_msg;
}
/*
arc_msgs::State gridAnalyser::arc_tools::generate2DState(const float x, const float y, const float alpha )
{
	arc_msgs::State state;
	state.pose.pose.position.x=x;
	state.pose.pose.position.y=y;
	geometry_msgs::Vector3 eu;
	eu.x=0;
	eu.y=0;
	eu.z=alpha;
	geometry_msgs::Quaternion quat;
	quat=arc_tools::transformQuaternionEulerMsg(eu);
	state.pose.pose.orientation=quat;
	return state;

}

*/

int gridAnalyser::indexOfDistanceFront(int i, float d)
{
	int j=i;
	float l = 0;
	while(l<d &&j<n_poses_path_-1)
	{
		l += sqrt(	pow(path_.poses[j+1].pose.position.x - path_.poses[j].pose.position.x,2)+
				pow(path_.poses[j+1].pose.position.y - path_.poses[j].pose.position.y,2)+
				pow(path_.poses[j+1].pose.position.z - path_.poses[j].pose.position.z,2));
		if(j+1>n_poses_path_-1){std::cout<<"PURE PURSUIT: LAUFZEITFEHLER::indexOfDistanceFront"<<std::endl;}
		j ++;
	}
	return j+1;
}

float gridAnalyser::radiusOfInflation(int x, int y)
{
	float radius;

	return radius;
}


