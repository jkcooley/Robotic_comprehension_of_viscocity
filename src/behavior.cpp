//tutorials used: https://github.com/utexas-bwi/segbot_arm/tree/master/segbot_arm_tutorials/src

//TODO: delete unnecessary methods / comments (what of this do we use?)

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
#include <iostream>

#define JOINTS 8

//describes the state of a set of torque-controlled joints - name, position, velocity, effort (http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/JointState.html)
sensor_msgs::JointState joint_state;
bool heard_state;

//pose with reference coordinate frame and timestamp (http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)
geometry_msgs::PoseStamped pose_stamped;
bool heard_pose_stamped;

//http://docs.ros.org/hydro/api/jaco_msgs/html/msg/FingerPosition.html - note that this robot does not have a finger 3
sensor_msgs::JointState joint_efforts;
//jaco_ros::jaco_arm_driver/out/finger_position finger_pose;
bool heard_efforts;
std::vector<float> efforts_data;

//flag for recording
bool record_haptics;
std::vector<float> pose_stamped_data; 

//global publisher for cartesian velocity
ros::Publisher velocity_pub;

//the name of the csv file to store the data in 
std::string file_name;

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
    
    if (record_haptics){
		//TODO: add the current message to a vector of poses
		
	}
}

//callback function for joint efforts
void joint_efforts_callback(const sensor_msgs::JointStateConstPtr &message)
{
	joint_efforts = *message;
    heard_efforts = true;
    
    if(record_haptics)
	{
//		efforts_data.push_back(joint_efforts);
	}
}

//moves the arm up and down (along the z-axis - keep in mind that the robot is tilted to stir)
void up_down(ros::NodeHandle node_handle, double velocity, int numRepetitions)
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


//moves the arm back and forth (along the x-axis - keep in mind that the robot is tilted to stir)
void back_and_forth(ros::NodeHandle node_handle, double velocity, int numRepetitions)
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
	
	
	for (int i = 0; i < numRepetitions; i++)
	{
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

	for(int rep = 0; rep < numRepetitions; rep++)
	{
	/*
	//construct message
	geometry_msgs::TwistStamped velocityMsg;
	velocityMsg.twist.linear.x = 0.0;
	velocityMsg.twist.linear.y = 0.0;
	velocityMsg.twist.angular.x = 0.0;
	velocityMsg.twist.angular.y = 0.0;
	velocityMsg.twist.angular.z = 0.0;
	*/
	double timeoutSeconds = 8.0;
	int rateHertz = 100;
	geometry_msgs::TwistStamped velocityMsg;
	
	double linearAngleX = 0;
	double linearVelX;
	double linearAngleZ = 0;
	double linearVelZ;
	//should make the circle bigger or smaller
	double magnitude = 0.2;
	
	ros::Rate r(rateHertz);
	for(int i = 0; i < (int)timeoutSeconds * rateHertz; i++) {
		
		linearAngleX += (2*PI)/(timeoutSeconds * rateHertz);
		linearVelX = magnitude * sin(linearAngleX);
		linearAngleZ += (2*PI)/(timeoutSeconds * rateHertz);
		linearVelZ = magnitude * cos(linearAngleZ);
		
		velocityMsg.twist.linear.x = -linearVelX;
		velocityMsg.twist.linear.y = 0.0;
		velocityMsg.twist.linear.z = linearVelZ;
		
		velocityMsg.twist.angular.x = 0.0;
		velocityMsg.twist.angular.y = 0.0;
		velocityMsg.twist.angular.z = 0.0;
		
		ROS_INFO("linearVelZ = %f", linearVelZ);
		ROS_INFO("linearVelX = %f", linearVelX);
		pub_velocity.publish(velocityMsg);
		
		ros::spinOnce();
		
		r.sleep();
	}

	/*
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
	}*/

	//publish 0 velocity command -- otherwise arm will continue moving with the last command for 0.25 seconds
	//velocityMsg.twist.linear.z = 0.0; 
}
}

//publish all 0s (for space between the actions)
void pause(ros::NodeHandle node_handle, double duration)
{
	//publish the velocities
        ros::Publisher pub_velocity = node_handle.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);

        //construct message
        geometry_msgs::TwistStamped velocityMsg;
        velocityMsg.twist.linear.x = 0.0;
        velocityMsg.twist.linear.y = 0.0;
        velocityMsg.twist.linear.z = 0.0;
        velocityMsg.twist.angular.x = 0.0;
        velocityMsg.twist.angular.y = 0.0;
        velocityMsg.twist.angular.z = 0.0;

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
}

//get user input for # trials
int getIterations(std::string message)
{
	//print the message passed (tell the user what to input)
	std::cout << message;

	while (true)
	{
		std::string input = "";
		
		getline(std::cin, input);
		
		if (input.compare("\n"))
			std::cout <<  message;
		
		else if (input.compare("quit") == 0)
			break;
		
		else 
		{
			int iterations;
			sscanf(input.c_str(), "%d", &iterations);
			return iterations;
		}
	}
}

//get user input for the name of the liquid being tested
std::string getLiquid(std::string message)
{
	//print the message passed (tell the user what to input)
	std::cout << message;

	while (true)
	{
		std::string input = "";
		
		getline(std::cin, input);
		
		if (input.compare("\n") == 0)
			std::cout <<  message;
		
		else if (input.compare("quit") == 0)
			break;
			
		else 
			return input;
	}
}

//write to file
/*void writeToFile(std::string file_path, int vectorLength, std::vector theData)
{
	std::ofstream file;
	file.open(file_path.c_str());
	
	ros::spinOnce();
	
	file << "Data format: ";
	
	for(int index = 0; index < vectorLength; index++)
	{
		//TODO: pass in the vector 
		file << theData.at(index) << ", ";
	}
	
	file << "\n";
}*/


//call functions to get data
int main(int argc, char **argv)
{
	//name the node
	ros::init(argc, argv, "behavior_1");

	ros::NodeHandle node_handle;

	//subscribe to topics
	ros::Subscriber joint_pose_subscriber = node_handle.subscribe("/joint_states", 1, joint_state_callback);
	ros::Subscriber pose_stamped_subscriber = node_handle.subscribe("/mico_arm_driver/out/tool_position", 1, pose_stamped_callback);	
	ros::Subscriber joint_efforts_subscriber = node_handle.subscribe("/mico_arm_driver/out/joint_efforts", 1, joint_efforts_callback);
	//ros::Subscriber finger_pose_subscriber = node_handle.subscribe("/mico_arm_driver/out/finger_position", 1, finger_pose_callback);

	//publish the velocities
	ros::Publisher velocity_publisher = node_handle.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);
	
	std::string liquid = getLiquid("Enter the name of the liquid being tested: ");
	
	int iterations = getIterations("Enter the number of iterations to perform: ");
	
	//start recording data
	record_haptics = true;

	for(int trial = 0; trial < iterations; trial++)
	{
		//TODO: initialize empty vectors for each topic
		
		up_down(node_handle, 0.2, 1);
		
		//behaviorName_trial#_liquid_typeOfDataRecorded.csv
//		std::string file_path = "/path/to/file" + "/example.csv";
		
//		int vectorLength = efforts_data.size();
		
//		writeToFile(file_path, vectorLength);
		
		//TODO: save the vectors to CSV or TXT

	//	circle_behavior(node_handle, .2, 1, 5);
	
		pause(node_handle, 5);

		back_and_forth(node_handle, .2, 1);
	}
	
	record_haptics = false;
}
