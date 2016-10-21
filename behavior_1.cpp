#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sensor_msgs/joint_state.h>

//up-down motion

//global publisher for cartesian velocity
ros::Publisher velocity_pub;

//finger 1
//finger 2
//finger 3 DNE, but could store anyway? 

//joints to take note of... 

//out/cartesian_velocity //subscriber, may be used to set velocity
//out/joint_velocity //subscriber, may be used to set velocity
//out/finger_position //raw angular position in degrees
//out/joint_angles //raw angular position in degrees
//out/joint_state //topic, publishes transformed joint angles in radians via sensor_msgs

/*
string[] name
float64[] position
float64[] velocity
float64[] effort
*/

//out/tool_position

bool heard_vel = false;


