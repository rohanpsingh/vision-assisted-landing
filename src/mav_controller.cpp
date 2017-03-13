#include <ros/ros.h>
#include <iostream>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include "precision_land/PosError.h"

#define FACTOR  0.6

#define MINRC   1100
#define BASERC  1500
#define MAXRC   1900

// Subscriber
ros::Subscriber sub;
ros::Subscriber mavros_state_sub;

// RC publisher
ros::Publisher pub;

// Time control
ros::Time lastTime;

double Roll, Pitch;

// Flight mode
std::string mode;
bool guided;
bool armed;

void mavCallback(const precision_land::PosError::ConstPtr &msg)
{
        // Time since last call
        double timeBetweenMarkers = (ros::Time::now() - lastTime).toSec();
        lastTime = ros::Time::now();

	float ErX = msg->errx;
	float ErY = msg->erry;
	
	ROS_INFO("Error = (%f , %f)", ErX, ErY);

        // Create RC msg
        mavros_msgs::OverrideRCIn rcmsg;


        // Calculate Roll and Pitch depending on the mode
        if (ErX<80 && ErY<80 && ErX>-80 && ErY>-80){
		if(mode == "LOITER"){
            		Roll = BASERC - ErX * FACTOR;
            		Pitch = BASERC - ErY * FACTOR;
		}
        }else{
            Roll = BASERC;
            Pitch = BASERC;
        }  
         
        // Limit the Roll
        if (Roll > MAXRC)
        {
            Roll = MAXRC;
        } else if (Roll < MINRC)
        {
            Roll = MINRC;
        }

        // Limit the Pitch
        if (Pitch > MAXRC)
        {
            Pitch = MAXRC;
        } else if (Pitch < MINRC)
        {
            Pitch = MINRC;
        }

        rcmsg.channels[0] = Roll;     //Roll
        rcmsg.channels[1] = Pitch;    //Pitch
        rcmsg.channels[2] = BASERC;   //Throttle
        rcmsg.channels[3] = 0;        //Yaw
        rcmsg.channels[4] = 0;
        rcmsg.channels[5] = 0;
        rcmsg.channels[6] = 0;
        rcmsg.channels[7] = 0;

        pub.publish(rcmsg);
}


void mavrosStateCb(const mavros_msgs::StateConstPtr &msg)
{
    if(msg->mode == std::string("CMODE(0)"))
        return;
    //ROS_INFO("I heard: [%s] [%d] [%d]", msg->mode.c_str(), msg->armed, msg->guided);
    mode = msg->mode;
    guided = msg->guided==128;
    armed = msg->armed==128;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mav_controller");
    ros::NodeHandle nh;
    lastTime = ros::Time::now();

    sub = nh.subscribe("errors_in_pos", 1, mavCallback);
    mavros_state_sub = nh.subscribe("/mavros/state", 1, mavrosStateCb);
    pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);;
    ros::spin();
}
