#define DO_LOG_LIE

#include "SLAM_server.h"

int main(int argc, char **argv) {
	char buffer[200];
	ros::init(argc, argv, "lamor_slam");
	SLAM_server serverSLAM;
	serverSLAM.update_map();
	ros::spin();
	printf("Exiting...\n");
	return 0;
}
