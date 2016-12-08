//tutorials used: https://github.com/utexas-bwi/segbot_arm/tree/master/segbot_arm_tutorials/src
//Names: JosieKate Cooley and Jacqueline Gibson
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
#include <complex.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#define JOINTS 8

//describes the state of a set of torque-controlled joints - name, position, velocity, effort (http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/JointState.html)
sensor_msgs::JointState joint_state;
bool heard_state;
std::vector<sensor_msgs::JointState> joint_state_data; 

//pose with reference coordinate frame and timestamp (http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)
geometry_msgs::PoseStamped pose_stamped;
bool heard_pose_stamped;
std::vector<geometry_msgs::PoseStamped> pose_stamped_data; 

//http://docs.ros.org/hydro/api/jaco_msgs/html/msg/FingerPosition.html - note that this robot does not have a finger 3
sensor_msgs::JointState joint_efforts;
bool heard_efforts;
std::vector<sensor_msgs::JointState> efforts_data;
//TODO: remove this? 
//jaco_ros::jaco_arm_driver/out/finger_position finger_pose;

//flag for recording
bool record_haptics;

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
    
    	if (record_haptics)
	{
		//TODO: add the current message to a vector of poses
		pose_stamped_data.push_back(pose_stamped);
	}
}

//callback function for joint efforts
void joint_efforts_callback(const sensor_msgs::JointStateConstPtr &message)
{
	joint_efforts = *message;
    	heard_efforts = true;
    
    	if (record_haptics)
    	{
//		efforts_data.push_back(joint_efforts);
	}
}

//moves the arm up and down (along the z-axis - keep in mind that the robot is tilted to stir)
void up_and_down(ros::NodeHandle node_handle, double velocity, int num_repetitions)
{
	//publish the velocities
	ros::Publisher pub_velocity = node_handle.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);

	//construct message
	geometry_msgs::TwistStamped velocity_message;
	velocity_message.twist.linear.x = 0.0;
	velocity_message.twist.linear.y = 0.0;
	velocity_message.twist.angular.x = 0.0;
	velocity_message.twist.angular.y = 0.0;
	velocity_message.twist.angular.z = 0.0;

	//TODO: can we put the duration and stuff here?

	for(int rep = 0; rep < num_repetitions; rep++)
	{
		velocity_message.twist.linear.z = velocity; 
	
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
			pub_velocity.publish(velocity_message);
		
			r.sleep();
		
			elapsed_time += (1.0/pub_rate);
		
			if (elapsed_time > duration)
				break;
		}
		
		velocity_message.twist.linear.z = velocity * -1;
	
		elapsed_time = 0.0;

		while (ros::ok())
		{
			//collect messages
			ros::spinOnce();
		
			//publish velocity message
			pub_velocity.publish(velocity_message);
		
			r.sleep();
		
			elapsed_time += (1.0/pub_rate);
		
			if (elapsed_time > duration)
				break;
		}
	}

	//publish 0 velocity command -- otherwise arm will continue moving with the last command for 0.25 seconds
	velocity_message.twist.linear.z = 0.0; 
	pub_velocity.publish(velocity_message);
}


//moves the arm back and forth (along the x-axis - keep in mind that the robot is tilted to stir)
void back_and_forth(ros::NodeHandle node_handle, double velocity, int num_repetitions)
{
	//publish the velocities
	ros::Publisher pub_velocity = node_handle.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);

	//construct message
	geometry_msgs::TwistStamped velocity_message;
	velocity_message.twist.linear.y = 0.0;
	velocity_message.twist.linear.z = 0.0; 
	velocity_message.twist.angular.x = 0.0;
	velocity_message.twist.angular.y = 0.0;
	velocity_message.twist.angular.z = 0.0;
	
	
	for (int i = 0; i < num_repetitions; i++)
	{
		velocity_message.twist.linear.x = velocity;

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
			pub_velocity.publish(velocity_message);
			
			r.sleep();
			
			elapsed_time += (1.0/pub_rate);
			
			if (elapsed_time > duration)
				break;
		}
			
		velocity_message.twist.linear.x = velocity * -1;
		
		elapsed_time = 0.0;

		while (ros::ok())
		{
			//collect messages
			ros::spinOnce();
			
			//publish velocity message
			pub_velocity.publish(velocity_message);
			
			r.sleep();
			
			elapsed_time += (1.0/pub_rate);
			
			if (elapsed_time > duration)
				break;
		}

		//publish 0 velocity command -- otherwise arm will continue moving with the last command for 0.25 seconds
		velocity_message.twist.linear.x = 0.0; 
		pub_velocity.publish(velocity_message);
	}
}

//moves the arm in a circle (using x and y axes)
void circle(ros::NodeHandle node_handle, double velocity, int num_repetitions, double radius)
{
	//publish the velocities
	ros::Publisher pub_velocity = node_handle.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);

	for (int rep = 0; rep < num_repetitions; rep++)
	{
		double timeout_seconds = 8.0;
		int rate_hertz = 100;
		geometry_msgs::TwistStamped velocity_message;
	
		double linear_angle_x = 0;
		double linear_vel_x;
		double linear_angle_z = 0;
		double linear_vel_z;
		//should make the circle bigger or smaller
		double magnitude = 0.2;
	
		ros::Rate r(rate_hertz);
		for (int i = 0; i < (int)timeout_seconds * rate_hertz; i++) 
		{	
			linear_angle_x += (2 * PI) / (timeout_seconds * rate_hertz);
			linear_vel_x = magnitude * sin(linear_angle_x);
			linear_angle_z += (2 * PI) / (timeout_seconds * rate_hertz);
			linear_vel_z = magnitude * cos(linear_angle_z);
		
			velocity_message.twist.linear.x = -linear_vel_x;
			velocity_message.twist.linear.y = 0.0;
			velocity_message.twist.linear.z = linear_vel_z;
		
			velocity_message.twist.angular.x = 0.0;
			velocity_message.twist.angular.y = 0.0;
			velocity_message.twist.angular.z = 0.0;
		
			ROS_INFO("linear_vel_z = %f", linear_vel_z);
			ROS_INFO("linear_vel_x = %f", linear_vel_x);
			pub_velocity.publish(velocity_message);
		
			ros::spinOnce();
		
			r.sleep();
		}
	}
}

//publish all 0s (for space between the actions)
void pause(ros::NodeHandle node_handle, double duration)
{
	//publish the velocities
        ros::Publisher pub_velocity = node_handle.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);

        //construct message
        geometry_msgs::TwistStamped velocity_message;
        velocity_message.twist.linear.x = 0.0;
        velocity_message.twist.linear.y = 0.0;
        velocity_message.twist.linear.z = 0.0;
        velocity_message.twist.angular.x = 0.0;
        velocity_message.twist.angular.y = 0.0;
        velocity_message.twist.angular.z = 0.0;

        double elapsed_time = 0.0;

        double pub_rate = 40.0; //we publish at 40 hz
        ros::Rate r(pub_rate);
 
        while (ros::ok())
        {
		//collect messages
                ros::spinOnce();

                //publish velocity message
                pub_velocity.publish(velocity_message);

                r.sleep();

                elapsed_time += (1.0/pub_rate);

                if (elapsed_time > duration)
                	break;
	}
}

//get user input for # trials
int get_iterations(std::string message)
{
	//print the message passed (tell the user what to input)
	std::cout << message;

	while (true)
	{
		std::string input = "";
		
		//get user input
		getline(std::cin, input);
		
	 	//if a newline was entered, print the request again
		if (input.compare("") == 0)
			std::cout <<  message;
		
		//if "quit" was entered, quit
		else if (input.compare("quit") == 0)
			return -1;

		//read the number of iterations from the user input
		else 
		{
			int iterations;
			sscanf(input.c_str(), "%d", &iterations);

			ROS_INFO("number of iterations: %d", iterations);

			return iterations;
		}
	}
}

//get user input for the name of the liquid being tested
std::string get_liquid(std::string message)
{
	//print the message passed (tell the user what to input)
	std::cout << message;

	while (true)
	{
		std::string input = "";
		
		//get user input
		getline(std::cin, input);
	
		//if a newline was entered, print the request again	
		if (input.compare("") == 0)
			std::cout <<  message;
		
		//if "quit" was entered, quit
		else if (input.compare("quit") == 0)
			return input;
			
		//return the user input (should be the name of the liquid)
		else 
		{
			ROS_INFO_STREAM("liquid: " << input);
	
			return input;
		}
	}
}

//write to file
void write_to_file(std::string file_name, int vector_length, std::vector<float> the_data)
{
	//set file up to be written to
	std::fstream file_stream;
	file_stream.open(file_name.c_str(), std::fstream::in);
	
	ros::spinOnce();
	
	//TODO: add data format
	file_stream << "Data format: ";
	
	for(int index = 0; index < vector_length; index++)
	{
		//pass in the vector 
		file_stream << the_data.at(index) << ", ";
	}
	
	file_stream << "\n";

	file_stream.close();
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
	ros::Subscriber joint_efforts_subscriber = node_handle.subscribe("/mico_arm_driver/out/joint_efforts", 1, joint_efforts_callback);
	//ros::Subscriber finger_pose_subscriber = node_handle.subscribe("/mico_arm_driver/out/finger_position", 1, finger_pose_callback);

	//publish the velocities
	ros::Publisher velocity_publisher = node_handle.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);
	
	std::string liquid = get_liquid("Enter the name of the liquid being tested: ");

	int iterations = get_iterations("Enter the number of iterations to perform: ");
	
	if (liquid.compare("quit") == 0 || iterations == -1)
	{
		return -1;
	}

	//start recording data
	record_haptics = true;

	int repetitions = 1, vector_length = efforts_data.size();
	double velocity = 0.2, pause_time = 3;
	//the name of the csv file to store the data in 
	std::string file_name;

	//run trials for up_and_down
	for (int trial = 1; trial <= iterations; trial++)
	{
		//TODO: initialize empty vectors for each topic
		//TODO: replace hapctics...maybe make this a method?
		file_name = "up_and_down_" + liquid + "_haptics" + "_trial_" + boost::to_string(trial) + ".csv";
		ROS_INFO_STREAM("file_name: " << file_name);
		//pause between trials
		pause(node_handle, pause_time);
		up_and_down(node_handle, velocity, repetitions);
		write_to_file(file_name, vector_length, efforts_data);
	}
	
	//run trials for back_and_forth
	for (int trial = 1; trial <= iterations; trial++)
	{
		file_name = "back_and_forth_" + liquid + "_haptics" + "_trial_" + boost::to_string(trial) + ".csv";
		ROS_INFO_STREAM("file_name: " << file_name);
		pause(node_handle, pause_time);
		back_and_forth(node_handle, velocity, repetitions);
		write_to_file(file_name, vector_length, efforts_data);
	}

	//run trials for 
	for (int trial = 1; trial <= iterations; trial++)
	{
		file_name = "circle_" + liquid + "_haptics" + "_trial_" + boost::to_string(trial) + ".csv";
		ROS_INFO_STREAM("file_name: " << file_name);
		pause(node_handle, pause_time);
		circle(node_handle, velocity, repetitions, 5);
		write_to_file(file_name, vector_length, efforts_data);
	}

	record_haptics = false;
}
