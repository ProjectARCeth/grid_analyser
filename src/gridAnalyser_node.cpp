#include "../include/gridAnalyser/gridAnalyser.hpp"
int main(int argc, char **argv)
{	
	ros::init(argc, argv, "grid_analyser");
	ros::NodeHandle node;
	gridAnalyser gridAnalyser_object(node);
	ros::spin();
	return 0;
}
