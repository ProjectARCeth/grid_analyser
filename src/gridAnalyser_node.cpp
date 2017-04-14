#include "../include/gridAnalyser/gridAnalyser.hpp"
int main(int argc, char **argv)
{	
	ros::init(argc, argv, "grid_analyser");
	ros::NodeHandle node;
	std::string PATH_NAME = *(argv + 1);
	gridAnalyser gridAnalyser_object(node,PATH_NAME);
	ros::spin();
	return 0;
}
