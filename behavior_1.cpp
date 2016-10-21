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


//spoon acceptance pose
/*
fri@marvin:~/viscosity_ws/src/segbot_arm/segbot_arm_tutorials$ rosrun segbot_arm_tutorials ex1_subscribing_to_topics 
[ INFO] [1477074469.354660031]: Current joint positions:
[ INFO] [1477074469.354791623]: header: 
  seq: 600
  stamp: 1477074469.341749274
  frame_id: 
name[]
  name[0]: mico_joint_1
  name[1]: mico_joint_2
  name[2]: mico_joint_3
  name[3]: mico_joint_4
  name[4]: mico_joint_5
  name[5]: mico_joint_6
  name[6]: mico_joint_finger_1
  name[7]: mico_joint_finger_2
position[]
  position[0]: 0.662198
  position[1]: -0.765186
  position[2]: -0.281049
  position[3]: -0.650929
  position[4]: 2.023
  position[5]: 2.35381
  position[6]: -0.00084
  position[7]: -0.00084
velocity[]
  velocity[0]: 0
  velocity[1]: 0
  velocity[2]: 0
  velocity[3]: 0
  velocity[4]: 0
  velocity[5]: 0
  velocity[6]: 0
  velocity[7]: 0
effort[]
  effort[0]: 6.08548e+14
  effort[1]: 1.22894e-26
  effort[2]: -2.03711e-18
  effort[3]: 4.59149e-41
  effort[4]: -2.03711e-18
  effort[5]: 4.59149e-41
  effort[6]: 0
  effort[7]: 0

[ INFO] [1477074469.354838094]: Current joint efforts:
[ INFO] [1477074469.354904775]: header: 
  seq: 599
  stamp: 0.000000000
  frame_id: 
name[]
  name[0]: jaco_joint_1
  name[1]: jaco_joint_2
  name[2]: jaco_joint_3
  name[3]: jaco_joint_4
  name[4]: jaco_joint_5
  name[5]: jaco_joint_6
position[]
velocity[]
effort[]
  effort[0]: 0.0508463
  effort[1]: -0.329351
  effort[2]: 0.388175
  effort[3]: -0.0921688
  effort[4]: -0.175491
  effort[5]: -0.149602

[ INFO] [1477074469.354927740]: Current finger positions:
[ INFO] [1477074469.354948797]: finger1: -6
finger2: -6
finger3: 0

[ INFO] [1477074469.354970198]: Current tool pose:
[ INFO] [1477074469.355011625]: header: 
  seq: 600
  stamp: 1477074469.345129396
  frame_id: mico_link_base
pose: 
  position: 
    x: 0.283979
    y: 0.476259
    z: 0.357072
  orientation: 
    x: -0.0458191
    y: 0.662054
    z: 0.726168
    w: 0.179623
*/

//beginning of motion pose
/*
fri@marvin:~/viscosity_ws/src/segbot_arm/segbot_arm_tutorials$ rosrun segbot_arm_tutorials ex1_subscribing_to_topics 
[ INFO] [1477074469.354660031]: Current joint positions:
[ INFO] [1477074469.354791623]: header: 
  seq: 600
  stamp: 1477074469.341749274
  frame_id: 
name[]
  name[0]: mico_joint_1
  name[1]: mico_joint_2
  name[2]: mico_joint_3
  name[3]: mico_joint_4
  name[4]: mico_joint_5
  name[5]: mico_joint_6
  name[6]: mico_joint_finger_1
  name[7]: mico_joint_finger_2
position[]
  position[0]: 0.662198
  position[1]: -0.765186
  position[2]: -0.281049
  position[3]: -0.650929
  position[4]: 2.023
  position[5]: 2.35381
  position[6]: -0.00084
  position[7]: -0.00084
velocity[]
  velocity[0]: 0
  velocity[1]: 0
  velocity[2]: 0
  velocity[3]: 0
  velocity[4]: 0
  velocity[5]: 0
  velocity[6]: 0
  velocity[7]: 0
effort[]
  effort[0]: 6.08548e+14
  effort[1]: 1.22894e-26
  effort[2]: -2.03711e-18
  effort[3]: 4.59149e-41
  effort[4]: -2.03711e-18
  effort[5]: 4.59149e-41
  effort[6]: 0
  effort[7]: 0

[ INFO] [1477074469.354838094]: Current joint efforts:
[ INFO] [1477074469.354904775]: header: 
  seq: 599
  stamp: 0.000000000
  frame_id: 
name[]
  name[0]: jaco_joint_1
  name[1]: jaco_joint_2
  name[2]: jaco_joint_3
  name[3]: jaco_joint_4
  name[4]: jaco_joint_5
  name[5]: jaco_joint_6
position[]
velocity[]
effort[]
  effort[0]: 0.0508463
  effort[1]: -0.329351
  effort[2]: 0.388175
  effort[3]: -0.0921688
  effort[4]: -0.175491
  effort[5]: -0.149602

[ INFO] [1477074469.354927740]: Current finger positions:
[ INFO] [1477074469.354948797]: finger1: -6
finger2: -6
finger3: 0

[ INFO] [1477074469.354970198]: Current tool pose:
[ INFO] [1477074469.355011625]: header: 
  seq: 600
  stamp: 1477074469.345129396
  frame_id: mico_link_base
pose: 
  position: 
    x: 0.283979
    y: 0.476259
    z: 0.357072
  orientation: 
    x: -0.0458191
    y: 0.662054
    z: 0.726168
    w: 0.179623

*/


bool heard_vel = false;


