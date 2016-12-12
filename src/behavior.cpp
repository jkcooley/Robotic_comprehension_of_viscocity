//tutorials used: https://github.com/utexas-bwi/segbot_arm/tree/master/segbot_arm_tutorials/src
//Names: JosieKate Cooley and Jacqueline Gibson

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
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
#include <ros/package.h>
#include <segbot_arm_manipulation/arm_positions_db.h>
#include <std_msgs/Float64.h>

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

//torque and force data (http://docs.ros.org/api/geometry_msgs/html/msg/WrenchStamped.html)
geometry_msgs::WrenchStamped wrench_stamped;
bool heard_wrench_stamped;
std::vector<geometry_msgs::WrenchStamped> wrench_stamped_data;

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

	if (record_haptics)
	{
		joint_state_data.push_back(joint_state);
	}
}

//callback function for stamped position
void pose_stamped_callback(const geometry_msgs::PoseStampedConstPtr &message)
{
	pose_stamped = *message;
    	heard_pose_stamped = true;
    
    	if (record_haptics)
	{
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
		efforts_data.push_back(joint_efforts);
	}
}

//callback function for wrench stamped
void wrench_stamped_callback(const geometry_msgs::WrenchStampedConstPtr &message)
{
	wrench_stamped = *message;
	heard_wrench_stamped = true;
	
	if (record_haptics)
	{
		wrench_stamped_data.push_back(wrench_stamped);
	}
}

//moves the arm up and down (along the z-axis - keep in mind that the robot is tilted to stir)
void up_and_down(ros::NodeHandle node_handle, double velocity, int num_repetitions, double duration)
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

	for(int rep = 0; rep < num_repetitions; rep++)
	{
		velocity_message.twist.linear.z = velocity; 
	
		double elapsed_time = 0.0;
	
		//publish at 40Hz
		double pub_rate = 40.0;
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
void back_and_forth(ros::NodeHandle node_handle, double velocity, int num_repetitions, double duration)
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

		double elapsed_time = 0.0;
		
		//publish at 40Hz
		double pub_rate = 40.0;
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
		double linear_angle_y = 0;
		double linear_vel_y;
		
		//should make the circle bigger or smaller
		double magnitude = radius;
	
		ros::Rate r(rate_hertz);
		
		for (int i = 0; i < (int)timeout_seconds * rate_hertz; i++) 
		{	
			linear_angle_x += (2 * PI) / (timeout_seconds * rate_hertz);
			linear_vel_x = magnitude * sin(linear_angle_x);
			linear_angle_y += (2 * PI) / (timeout_seconds * rate_hertz);
			linear_vel_y = magnitude * cos(linear_angle_y);
		
			velocity_message.twist.linear.x = -linear_vel_x;
			velocity_message.twist.linear.y = linear_vel_y;
			velocity_message.twist.linear.z = 0.0;
			velocity_message.twist.angular.x = 0.0;
			velocity_message.twist.angular.y = 0.0;
			velocity_message.twist.angular.z = 0.0;
		
			pub_velocity.publish(velocity_message);
		
			ros::spinOnce();
		
			r.sleep();
		}
	}
}

//moves the hand in a twisting motion for five seconds in either director (L and R)
void twist(ros::NodeHandle node_handle, double velocity, int num_repetitions, double duration)
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

	for(int rep = 0; rep < num_repetitions; rep++)
	{
		velocity_message.twist.angular.z = velocity;

		double elapsed_time = 0.0;
	
		//we publish at 40Hz
		double pub_rate = 40.0;
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
		
		velocity_message.twist.angular.z = velocity * -1;
	
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
	velocity_message.twist.angular.z = 0.0; 
	pub_velocity.publish(velocity_message);
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

	//publish at 40Hz
        double pub_rate = 40.0; 
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

//get user input for the name of the material being tested
std::string get_material(std::string message)
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
			ROS_INFO_STREAM("material: " << input);
	
			return input;
		}
	}
}

//write to file
void write_to_file(std::string joint_state_file_name, int joint_state_vector_length, std::vector<sensor_msgs::JointState> joint_state_data, std::string efforts_file_name, int efforts_vector_length, std::vector<sensor_msgs::JointState> efforts_data, std::string pose_stamped_file_name, int pose_stamped_vector_length, std::vector<geometry_msgs::PoseStamped> pose_stamped_data, std::string wrench_stamped_file_name, int wrench_stamped_vector_length, std::vector<geometry_msgs::WrenchStamped> wrench_stamped_data)
{
	//set file up to be written to
	std::ofstream joint_state_file_stream;
	std::ofstream efforts_file_stream;
	std::ofstream pose_stamped_file_stream;
	std::ofstream wrench_stamped_file_stream;
	joint_state_file_stream.open(joint_state_file_name.c_str(), std::ofstream::out);
	efforts_file_stream.open(efforts_file_name.c_str(), std::ofstream::out);
	pose_stamped_file_stream.open(pose_stamped_file_name.c_str(), std::ofstream::out);	
	wrench_stamped_file_stream.open(wrench_stamped_file_name.c_str(), std::ofstream::out);	
	
	ros::spinOnce();
	
	joint_state_file_stream << "http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html\nData format:\nHeader header\nstring[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n\n\n";
	efforts_file_stream << "http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html\nData format:\nHeader header\nstring[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n\n\n";
	pose_stamped_file_stream << "http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html\nData format:\nHeader header\nPose pose\n\n\n";
	wrench_stamped_file_stream << "http://docs.ros.org/api/geometry_msgs/html/msg/WrenchStamped.html\nData format:\nHeader header\nWrench wrench\n\n\n";

	//store joint_state_data
	for (int index = 0; index < joint_state_vector_length; index++)
	{
		sensor_msgs::JointState current = joint_state_data[index];	

	        joint_state_file_stream << "[";

        	for (int index = 0; index < current.position.size(); index++)
        	{
                	joint_state_file_stream << current.velocity[index] << ", ";
        	}

		joint_state_file_stream << "]\n";

		joint_state_file_stream << "[";

                for (int index = 0; index < current.velocity.size(); index++)
                {
                        joint_state_file_stream << current.velocity[index] << ", ";
                }

                joint_state_file_stream << "]\n";
	}
			
	//store efforts_data
	for (int index = 0; index < efforts_vector_length; index++)
	{
		sensor_msgs::JointState current = efforts_data[index];

		efforts_file_stream << "[";

                for (int index = 0; index < current.effort.size(); index++)
                {
                        efforts_file_stream << current.effort[index] << ", ";
                }

                efforts_file_stream << "]\n";
	}

	//store pose_stamped_data 
	for (int index = 0; index < pose_stamped_vector_length; index++)
	{
		geometry_msgs::PoseStamped current = pose_stamped_data[index];

		pose_stamped_file_stream << current.header.stamp.sec;
		pose_stamped_file_stream << current.header.stamp.nsec;
		pose_stamped_file_stream << current.pose.position.x;
		pose_stamped_file_stream << current.pose.position.y;
		pose_stamped_file_stream << current.pose.position.z;
		pose_stamped_file_stream << current.pose.orientation.x;
		pose_stamped_file_stream << current.pose.orientation.y;
		pose_stamped_file_stream << current.pose.orientation.z;
		pose_stamped_file_stream << current.pose.orientation.w;
	}

	//store wrench_stamped_data
	for (int index = 0; index < wrench_stamped_vector_length; index++)
	{
		geometry_msgs::WrenchStamped current = wrench_stamped_data[index];		
	
		wrench_stamped_file_stream << current.header.stamp.sec;
		wrench_stamped_file_stream << current.header.stamp.nsec;
		pose_stamped_file_stream << current.wrench.force.x;
		pose_stamped_file_stream << current.wrench.force.y;
		pose_stamped_file_stream << current.wrench.force.z;
		pose_stamped_file_stream << current.wrench.force.x;
		pose_stamped_file_stream << current.wrench.force.y;
		pose_stamped_file_stream << current.wrench.force.z;
	}

	joint_state_file_stream << "\n";
	efforts_file_stream << "\n";
	pose_stamped_file_stream << "\n";
	wrench_stamped_file_stream << "\n";

	joint_state_file_stream.close();
	efforts_file_stream.close();
	pose_stamped_file_stream.close();
	wrench_stamped_file_stream.close();
}

//move the arm into the starting position
void go_to_start(std::string message)
{
	std::cout << message;

	while (true)
	{
		char c = std::cin.get();
	
		if (c == '\n')
			break;

		else if (c == 'q')
		{
			ros::shutdown();
			exit(1);
		}
		else 
		{
			std::cout <<  message;
		}
	}
}

//call functions to get data
int main(int argc, char **argv)
{
	ros::init(argc, argv, "behavior");

	ros::NodeHandle node_handle;

	//subscribe to topics
	ros::Subscriber joint_pose_subscriber = node_handle.subscribe("/joint_states", 1, joint_state_callback);
	ros::Subscriber pose_stamped_subscriber = node_handle.subscribe("/mico_arm_driver/out/tool_position", 1, pose_stamped_callback);	
	ros::Subscriber wrench_stamped_subscriber = node_handle.subscribe("/mico_arm_driver/out/tool_wrench", 1, wrench_stamped_callback);	
	ros::Subscriber joint_efforts_subscriber = node_handle.subscribe("/mico_arm_driver/out/joint_efforts", 1, joint_efforts_callback);

	//publish the velocities
	ros::Publisher velocity_publisher = node_handle.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);
	
	std::string liquid = get_material("Enter the name of the substance being tested: ");

	std::string stir = get_material("Enter the type of material being used to stir: ");

	int iterations = get_iterations("Enter the number of iterations to perform: ");
	
	if (liquid.compare("quit") == 0 || stir.compare("quit") == 0 || iterations == -1)
	{
		return -1;
	}

	//get starting position
	std::string j_pos_filename = ros::package::getPath("robotic_comprehension_of_viscosity") + "/data/jointspace_position_db.txt";
	std::string start_pos_filename = ros::package::getPath("robotic_comprehension_of_viscosity") + "/data/toolspace_position_db.txt";
	ArmPositionDB positionDB(j_pos_filename, start_pos_filename);

	//move the arm to a pose using MoveIt
	go_to_start("Press [Enter] to move the arm to its starting position: ");
	geometry_msgs::PoseStamped start_pose = positionDB.getToolPositionStamped("start","/mico_link_base");
				
	//now go to the pose
	segbot_arm_manipulation::moveToPoseMoveIt(node_handle, start_pose);

	//start recording data
	record_haptics = true;

	int repetitions = 1, joint_state_vector_length = joint_state_data.size(), efforts_vector_length = efforts_data.size(), pose_stamped_vector_length = pose_stamped_data.size(), wrench_stamped_vector_length = wrench_stamped_data.size();
	double velocity = .1, pause_time = 3, duration = 2;
	//the name of the csv file to store the data in 
	std::string joint_state_file_name, efforts_file_name, pose_stamped_file_name, wrench_stamped_file_name, path = "/home/users/fri/viscosity_data/";

	for (int run = 1; run < 5; run++)
	{
		//run trials for up_and_down
		for (int trial = 1; trial <= iterations; trial++)
		{
			joint_state_file_name = path + "up_and_down_" + stir + "_stir_" + liquid + "_joint_state" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			efforts_file_name = path + "up_and_down_" + stir + "_stir_" + liquid + "_efforts" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			pose_stamped_file_name = path + "up_and_down_" + stir + "_stir_" + liquid + "_pose_stamped" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			wrench_stamped_file_name = path + "up_and_down_" + stir + "_stir_" + liquid + "_wrench_stamped" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			ROS_INFO_STREAM("joint_state_file_name: " << joint_state_file_name);
			ROS_INFO_STREAM("efforts_file_name: " << efforts_file_name);
			ROS_INFO_STREAM("pose_stamped_file_name: " << pose_stamped_file_name);
			ROS_INFO_STREAM("wrench_stamped_file_name: " << wrench_stamped_file_name);
			//pause between trials
			pause(node_handle, pause_time);
			up_and_down(node_handle, velocity, repetitions, duration);
			write_to_file(joint_state_file_name, joint_state_vector_length, joint_state_data, efforts_file_name, efforts_vector_length, efforts_data, pose_stamped_file_name, pose_stamped_vector_length, pose_stamped_data, wrench_stamped_file_name, wrench_stamped_vector_length, wrench_stamped_data);
		}	
	
			//run trials for back_and_forth
			for (int trial = 1; trial <= iterations; trial++)
			{
			joint_state_file_name = path + "back_and_forth_" + stir + "_stir_" + liquid + "_joint_state" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			efforts_file_name = path + "back_and_forth_" + stir + "_stir_" + liquid + "_efforts" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			pose_stamped_file_name = path + "back_and_forth_" + stir + "_stir_" + liquid + "_pose_stamped" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			wrench_stamped_file_name = path + "back_and_forth_" + stir + "_stir_" + liquid + "_wrench_stamped" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			ROS_INFO_STREAM("joint_state_file_name: " << joint_state_file_name);
			ROS_INFO_STREAM("efforts_file_name: " << efforts_file_name);
			ROS_INFO_STREAM("pose_stamped_file_name: " << pose_stamped_file_name);
			ROS_INFO_STREAM("wrench_stamped_file_name: " << wrench_stamped_file_name);
			pause(node_handle, pause_time);
			back_and_forth(node_handle, velocity, repetitions, duration);
			write_to_file(joint_state_file_name, joint_state_vector_length, joint_state_data, efforts_file_name, efforts_vector_length, efforts_data, pose_stamped_file_name, pose_stamped_vector_length, pose_stamped_data, wrench_stamped_file_name, wrench_stamped_vector_length, wrench_stamped_data);
	}

		//run trials for circle
		for (int trial = 1; trial <= iterations; trial++)
		{
			joint_state_file_name = path + "circle_" + stir + "_stir_" + liquid + "_joint_state" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			efforts_file_name = path + "circle_" + stir + "_stir_" + liquid + "_efforts" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			pose_stamped_file_name = path + "circle_" + stir + "_stir_" + liquid + "_pose_stamped" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			wrench_stamped_file_name = path + "circle_" + stir + "_stir_" + liquid + "_wrench_stamped" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			ROS_INFO_STREAM("joint_state_file_name: " << joint_state_file_name);
			ROS_INFO_STREAM("efforts_file_name: " << efforts_file_name);
			ROS_INFO_STREAM("pose_stamped_file_name: " << pose_stamped_file_name);
			ROS_INFO_STREAM("wrench_stamped_file_name: " << wrench_stamped_file_name);
			pause(node_handle, pause_time);
			circle(node_handle, velocity, repetitions, .05);
			write_to_file(joint_state_file_name, joint_state_vector_length, joint_state_data, efforts_file_name, efforts_vector_length, efforts_data, pose_stamped_file_name, pose_stamped_vector_length, pose_stamped_data, wrench_stamped_file_name, wrench_stamped_vector_length, wrench_stamped_data);
		}

		//run trials for twist
/*		for (int trial = 1; trial <= iterations; trial++)
		{
			joint_state_file_name = path + "twist_" + stir + "_stir_" + liquid + "_joint_state" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			efforts_file_name = path + "twist_" + stir + "_stir_" + liquid + "_efforts" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			pose_stamped_file_name = path + "twist_" + stir + "_stir_" + liquid + "_pose_stamped" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			wrench_stamped_file_name = path + "twist_" + stir + "_stir_" + liquid + "_wrench_stamped" + "_trial_" + boost::to_string(trial) + "_run_" + boost::to_string(run) + ".csv";
			ROS_INFO_STREAM("joint_state_file_name: " << joint_state_file_name);
			ROS_INFO_STREAM("efforts_file_name: " << efforts_file_name);
			ROS_INFO_STREAM("pose_stamped_file_name: " << pose_stamped_file_name);
			ROS_INFO_STREAM("wrench_stamped_file_name: " << wrench_stamped_file_name);
			pause(node_handle, pause_time);
//			twist(node_handle, velocity, repetitions, duration);
			circle(node_handle, velocity, repetitions, .05);
			write_to_file(joint_state_file_name, joint_state_vector_length, joint_state_data, efforts_file_name, efforts_vector_length, efforts_data, pose_stamped_file_name, pose_stamped_vector_length, pose_stamped_data, wrench_stamped_file_name, wrench_stamped_vector_length, wrench_stamped_data);
		}*/
	
		velocity *= 2;
		duration /= 2;
	}

	record_haptics = false;
}
