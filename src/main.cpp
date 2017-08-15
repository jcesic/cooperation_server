#define DO_LOG_LIE

#include "SLAM.h"

int main(int argc, char **argv) {
	char buffer[200];
	ros::init(argc, argv, "lamor_slam");
	SLAM amorSLAM;
	ros::spin();
	printf("Exiting...\n");
	amorSLAM.LESDSF.RecoverState();
	snprintf(buffer, sizeof(buffer), "/media/kruno/Data/TEST/SLAM/results/poses/LX_%02d.txt",amorSLAM.agent_id);
	amorSLAM.printTrajectory(buffer);
	return 0;
}
