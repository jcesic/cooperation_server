#include <ros/ros.h>
#include <rvl_psulmdemo/Match.h>
#include <sensor_msgs/Image.h>
#include <iostream>

#define PI 	3.14159265359;

using namespace std;

class match_img_pub{

private:
    ros::NodeHandle n;
    	
public:
   ros::Publisher match_pub;
   ros::Subscriber img_sub;
   int pubcount;
  
   match_img_pub(){
	   match_pub = n.advertise<rvl_psulmdemo::Match>("/appearance_matches",100);
	   img_sub=n.subscribe("/throttled/image_raw",500,&match_img_pub::imgCallback,this);
	   pubcount = 0;
	   ROS_INFO("Match started v2");
   }
   
   void imgCallback(const sensor_msgs::Image::ConstPtr& msg_img){
		n.getParam("/pubcount", pubcount);
		if(pubcount>0){
			rvl_psulmdemo::Match m_msg;
			m_msg.header.stamp = ros::Time::now();
			m_msg.header.frame_id = "match";
			m_msg.fromImgSeq = pubcount;
			m_msg.toImgSeq.push_back(pubcount-1);
			match_pub.publish(m_msg);
			printf("	UPDATE Pokrenut update izmedu stanja %d %d\n",pubcount,pubcount-1);
		}else{
			printf("pub 0\n");
		}
   }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_odometry");
    match_img_pub *imgmatch = new match_img_pub();
    ros::spin();
    delete imgmatch;

    return 0;
}

