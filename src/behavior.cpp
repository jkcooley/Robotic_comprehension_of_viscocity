//tutorials used: https://github.com/utexas-bwi/segbot_arm/tree/master/segbot_arm_tutorials/src

//TODO: delete unnecessary methods / comments

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <jaco_msgs/FingerPosition.h>
#include <jaco_msgs/ArmJointAnglesAction.h>
#include <jaco_msgs/ArmPoseAction.h>
#include <jaco_msgs/HomeArm.h>
#include <segbot_arm_manipulation/arm_utils.h>
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <complex.h>

#define JOINTS 8

//describes the state of a set of torque-controlled joints - name, position, velocity, effort (http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/JointState.html)
sensor_msgs::JointState joint_state;
bool heard_state;

//pose with reference coordinate frame and timestamp (http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)
geometry_msgs::PoseStamped pose_stamped;
bool heard_pose_stamped;

//http://docs.ros.org/hydro/api/jaco_msgs/html/msg/FingerPosition.html - note that this robot does not have a finger 3
jaco_msgs::FingerPosition finger_pose;
//jaco_ros::jaco_arm_driver/out/finger_position finger_pose;
bool heard_finger_pose;

//global publisher for cartesian velocity
ros::Publisher velocity_pub;

//callback function for joint state
void joint_state_callback(const sensor_msgs::JointStateConstPtr &message)
{
	//if data from all the joints has been recieved, set joint_state and heard_state accordingly
	if (message->position.size() == JOINTS)
	{
		joint_state = *message;
		heard_state = true;
	}
}

//callback function for stamped position
//changed from sensor_msgs because PoseStamped is a geometry_msgs message
void pose_stamped_callback(const geometry_msgs::PoseStampedConstPtr &message)
{
	pose_stamped = *message;
        heard_pose_stamped = true;
}

//callback function for finger position
void finger_pose_callback(const jaco_msgs::FingerPositionConstPtr &message)
{
	finger_pose = *message;
        heard_finger_pose = true;
}

//get the states from the arm
void get_data()
{
	heard_state = false;
	heard_pose_stamped = false;
	heard_finger_pose = false;
	
	//40 Hz
	ros::Rate r(40.0);
	
	while (ros::ok())
	{
		ros::spinOnce();	
		
		//if the data has been acquired, exit; else, loop back and try again while able
		if (heard_state && heard_pose_stamped && heard_finger_pose)
			return;
		
		r.sleep();
	}
}

//from https://github.com/utexas-bwi/segbot_arm/blob/experiments/object_exploration/object_exploration/src/interact_arm.cpp 
//completely unchanged... should probably fix that (TODO)
bool goToLocation(float position[]){
	
	actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ac("/mico_arm_driver/joint_angles/arm_joint_angles", true);
	jaco_msgs::ArmJointAnglesGoal goal;
	goal.angles.joint1 = position[0];
	goal.angles.joint2 = position[1];
	goal.angles.joint3 = position[2];
	goal.angles.joint4 = position[3];
	goal.angles.joint5 = position[4];
	goal.angles.joint6 = position[5];
	ac.waitForServer();
	ac.sendGoal(goal);
	ROS_INFO("Trajectory goal sent");
	ac.waitForResult();
}

//moves the arm forward and backwards (along the z-axis)
void back_and_forth(ros::NodeHandle node_handle, double velocity, int numRepetitions)
{
	//publish the velocities
	ros::Publisher pub_velocity = node_handle.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);

	//construct message
	geometry_msgs::TwistStamped velocityMsg;
	velocityMsg.twist.linear.x = 0.0;
	velocityMsg.twist.linear.y = 0.0;
	velocityMsg.twist.angular.x = 0.0;
	velocityMsg.twist.angular.y = 0.0;
	velocityMsg.twist.angular.z = 0.0;

	//TODO: can we put the duration and stuff here?

	for(int rep = 0; rep < numRepetitions; rep++)
	{
		velocityMsg.twist.linear.z = velocity; 
	
		//TODO: 2 secs or 1 sec?
		double duration = 2.0; //2 seconds
		double elapsed_time = 0.0;
	
		double pub_rate = 40.0; //we publish at 40 hz
		ros::Rate r(pub_rate);
	
		while (ros::ok())
		{
			//collect messages
			ros::spinOnce();
		
			//publish velocity message
			pub_velocity.publish(velocityMsg);
		
			r.sleep();
		
			elapsed_time += (1.0/pub_rate);
		
			if (elapsed_time > duration)
				break;
		}
		
		velocityMsg.twist.linear.z = velocity * -1;
	
		elapsed_time = 0.0;

		while (ros::ok())
		{
			//collect messages
			ros::spinOnce();
		
			//publish velocity message
			pub_velocity.publish(velocityMsg);
		
			r.sleep();
		
			elapsed_time += (1.0/pub_rate);
		
			if (elapsed_time > duration)
				break;
		}
	}

	//publish 0 velocity command -- otherwise arm will continue moving with the last command for 0.25 seconds
	velocityMsg.twist.linear.z = 0.0; 
	pub_velocity.publish(velocityMsg);
}


//moves the arm up and down (along the x-axis)
void up_down_behavior(ros::NodeHandle node_handle, double velocity, int numRepetitions)
{
	//publish the velocities
	ros::Publisher pub_velocity = node_handle.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);

	//construct message
	geometry_msgs::TwistStamped velocityMsg;
	velocityMsg.twist.linear.y = 0.0;
	velocityMsg.twist.linear.z = 0.0; 
	velocityMsg.twist.angular.x = 0.0;
	velocityMsg.twist.angular.y = 0.0;
	velocityMsg.twist.angular.z = 0.0;
	
	
	for (int i = 0; i < numRepetitions; i++){

		velocityMsg.twist.linear.x = velocity;

		//TODO: 2 secs or 1 sec?
		double duration = 2.0; //2 seconds
		double elapsed_time = 0.0;
		
		double pub_rate = 40.0; //we publish at 40 hz
		ros::Rate r(pub_rate);
		
		//TODO: should this be an if instead? 	
		while (ros::ok())
		{
			//collect messages
			ros::spinOnce();
			
			//publish velocity message
			pub_velocity.publish(velocityMsg);
			
			r.sleep();
			
			elapsed_time += (1.0/pub_rate);
			
			if (elapsed_time > duration)
				break;
		}
			
		velocityMsg.twist.linear.x = velocity * -1;
		
		elapsed_time = 0.0;

		while (ros::ok())
		{
			//collect messages
			ros::spinOnce();
			
			//publish velocity message
			pub_velocity.publish(velocityMsg);
			
			r.sleep();
			
			elapsed_time += (1.0/pub_rate);
			
			if (elapsed_time > duration)
				break;
		}
		
		//publish 0 velocity command -- otherwise arm will continue moving with the last command for 0.25 seconds
		velocityMsg.twist.linear.x = 0.0; 
		pub_velocity.publish(velocityMsg);
	}
}


//moves the arm in a circle (using x and y axes)
void circle_behavior(ros::NodeHandle node_handle, double velocity, int numRepetitions, double radius)
{
	//publish the velocities
	ros::Publisher pub_velocity = node_handle.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);

	//construct message
	geometry_msgs::TwistStamped velocityMsg;
	velocityMsg.twist.linear.x = 0.0;
	velocityMsg.twist.linear.y = 0.0;
	velocityMsg.twist.angular.x = 0.0;
	velocityMsg.twist.angular.y = 0.0;
	velocityMsg.twist.angular.z = 0.0;

	for(int rep = 0; rep < numRepetitions; rep++)
	{
		double angle = 0;

		while(angle <= 360)
		{
			//want to move in an x and y direction at the same time...
			//use sin / cos? 
			//equation of a circle: x = rcos(t), y = rsin(t)
			//TODO: figure this out
//			velocityMsg.twist.linear.x = velocity * radius * (double)csin(angle);
//			velocityMsg.twist.linear.y = velocity * radius * sin(angle); 				
			//TODO: 2 secs or 1 sec?
			double duration = 1.0; //2 seconds
			double elapsed_time = 0.0;
	
			double pub_rate = 40.0; //we publish at 40 hz
			ros::Rate r(pub_rate);
	
			while (ros::ok())
			{
				//collect messages
				ros::spinOnce();
	
				//publish velocity message
				pub_velocity.publish(velocityMsg);
		
				r.sleep();
		
				elapsed_time += (1.0/pub_rate);
		
				if (elapsed_time > duration)
					break;
			}
		
			velocityMsg.twist.linear.z = -1 * velocity;
		
			angle += 10;
		}
	}

	//publish 0 velocity command -- otherwise arm will continue moving with the last command for 0.25 seconds
	velocityMsg.twist.linear.z = 0.0; 
	pub_velocity.publish(velocityMsg);
}


//call functions to get data
int main(int argc, char **argv)
{
	//name the node
	ros::init(argc, argv, "behavior_1");

	ros::NodeHandle node_handle;

	//subscribe to topics
	ros::Subscriber joint_pose_subscriber = node_handle.subscribe("/joint_states", 1, joint_state_callback);
	ros::Subscriber pose_stamped_subscriber = node_handle.subscribe("/mico_arm_driver/out/tool_position", 1, pose_stamped_callback);	
	ros::Subscriber finger_pose_subscriber = node_handle.subscribe("/mico_arm_driver/out/finger_position", 1, finger_pose_callback);

	//publish the velocities
	ros::Publisher velocity_publisher = node_handle.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);
	
	//try test behavior
	up_down_behavior(node_handle, 0.2, 1);

	circle_behavior(node_handle, .2, 1, 5);
	
	back_and_forth(node_handle, .2, 1);

	//get the states from the arm
/*	get_data();

	geometry_msgs::PoseStamped goal_x;
	goal_x.header.frame_id = "mico_link_base";
	goal_x.pose.position.x = 0.389813;
	goal_x.pose.position.y = -0.0222033;
	goal_x.pose.position.z = 0.22019;
	goal_x.pose.orientation.x = 0.734704;
	goal_x.pose.orientation.y = 0.734704;
	goal_x.pose.orientation.z = 0.0426369;
	goal_x.pose.orientation.w = 0.0219083;
	
//	segbot_arm_manipulation::moveToPoseMoveIt(node_handle, goal_x);

	geometry_msgs::TwistStamped message;
	
	int direction = 1;

	double startTime = ros::Time::now().toSec();

	double endTime = startTime + 20;
	
	
	double pub_rate = 40.0; //we publish at 40 hz
	ros::Rate rate(pub_rate);

	while(endTime > ros::Time::now().toSec())
	{
		//move one direction (should be forward or backwards)
		message.twist.linear.x = 0.07 * direction;

		velocity_publisher.publish(message);
	
		//from http://wiki.ros.org/roscpp/Overview/Time
		ros::Duration(.25).sleep(); // sleep for one second

		//move the opposite direction
		direction *= -1;
	}

	message.twist.linear.x = 0;

	velocity_publisher.publish(message);
//<<<<<<< HEAD

	
	
//=======
//>>>>>>> de0d018490766d6478de01cd85741b2c8644dc1b

	//whisk
	//get back into position 
//<<<<<<< HEAD
//	segbot_arm_manipulation::moveToPoseMoveIt(node_handle, goal_x);
//=======
//	segbot_arm_manipulation::moveToPoseMoveIt(node_handle, goal_x);

	
//>>>>>>> 37c6f83915c730c2f4d3bb738b791d43031a1aa1
*/
}
