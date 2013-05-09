#include <ros/ros.h>
#include "puppeteer_msgs/RobotCommands.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <float.h>
#include <time.h>
#include <assert.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#define FREQ (50.0)
#define PERIOD (1/FREQ)

std::string filename;

template <typename T> int sgn(T val)
{
    return (val > T(0)) - (val < T(0));
}       

template<typename T>
    T fromString(const std::string& s)
{
     std::istringstream stream (s);
     T t;
     stream >> t;
     return t;
}

//---------------------------------------------------------------------------
// Class Definitions
//---------------------------------------------------------------------------
class PendControl{

private:
    typedef struct
    {
	int RobotMY;
	float DT;
	unsigned int num;
	float vals[][2]; // unknown length;
			 // t,robot_trans_vel
    } Trajectory;       

    int operating_condition;
    Trajectory *traj;
    ros::NodeHandle n_;
    ros::Publisher serial_pub;
    ros::Timer timer;
    bool start_flag;
    unsigned int num;
    ros::Time base_time;
    ros::Time current_time;

    puppeteer_msgs::RobotCommands command;

public:
    PendControl(){
	// Initialize necessary variables:
	if(ros::param::has("/operating_condition"))
	    ros::param::set("/operating_condition", 0);
	else
	{
	    ROS_WARN("Cannot Find Parameter: /operating_condition");
	    ros::param::set("/operating_condition", 0);
	}

	// read in the trajectory
	traj = ReadControls(filename);
	
	// Define publisher for the serial commands:
	serial_pub = n_.advertise<puppeteer_msgs::RobotCommands>
	    ("serial_commands", 1);
	// Send a start flag:
	ros::Duration(1.0).sleep();
	send_start_flag();
	// define a timer and associated callback:
	timer = n_.createTimer(ros::Duration(PERIOD),
			       &PendControl::timercb, this);
    }

    void send_start_flag(void)
	{
	    ROS_DEBUG("Sending start flag");
	    // First set the parameters for the service call
	    command.robot_index = traj->RobotMY;
	    command.header.stamp = ros::Time::now();
	    command.type = 'm';
	    command.div = 0;
	    serial_pub.publish(command);
	    return;
	} // END OF send_start_flag()


    void timercb(const ros::TimerEvent& e)
	{
	    ROS_DEBUG("Control timer callback triggered");
	    static double running_time = 0.0;
	    ros::param::getCached("/operating_condition", operating_condition);

	    if( operating_condition == 0 ||
		operating_condition == 1 ||
		operating_condition == 3 )
	    {
		start_flag = true;
		return;
	    }
	    else if (operating_condition == 2)
	    {
		if (start_flag)
		{
		    ROS_INFO("Beginning movement execution");
		    base_time = ros::Time::now();
		    start_flag = false;
		}
		current_time = ros::Time::now();
		running_time = (current_time-base_time).toSec();
		if (running_time <= traj->vals[num-1][0])
		    set_command_values(running_time);
		else
		{
		    // stop robot!
		    ROS_INFO("Trajectory Finished!");
		    command.robot_index = traj->RobotMY;
		    command.header.stamp = current_time;
		    command.type = 'h';
		    command.v_left = 0.0;
		    command.v_right = 0.0;
		    command.v_top = 0.0;
		    command.div = 3;
		    // set operating_condition to stop
		    ros::param::set("/operating_condition", 3);
		    start_flag = true;
		}
	    }
	    else if (operating_condition == 4)
	    {
		ROS_WARN_THROTTLE(1, "Emergency Stop Detected!");
		// then let's keep sending the stop command just so we
		// make sure the robots stop:
		command.robot_index = traj->RobotMY;
		command.header.stamp = ros::Time::now();
		command.type = 'h';
		command.v_left = 0.0;
		command.v_right = 0.0;
		command.v_top = 0.0;
		command.div = 3;
		start_flag = true;
	    }
	    service_request();
	    return;
	} // END OF timercb()


    void service_request(void)
	{
	    serial_pub.publish(command);
	    return;
	} // END OF service_request()

    void set_command_values(float time)
	{
	    ROS_DEBUG("Interpolating desired commands");
	    unsigned int index;
	    float mult, v;
	    for (index=0; index<num; index++)
	    {
		if (traj->vals[index][0] > time)
		    break;
	    }
	    mult = (time-traj->vals[index-1][0])/traj->DT;
	    v = (traj->vals[index-1][1]) +
		mult*(traj->vals[index][1]-traj->vals[index-1][1]);
	    
	    ROS_DEBUG("Sending control values: v = %f\tw = %f",v,0.0);
	    // Set service parameters:
	    command.type = 'd';
	    command.robot_index = traj->RobotMY;
	    command.header.stamp = current_time;
	    command.v_robot = v;
	    command.w_robot = 0.0;
	    command.rdot = 0.0;
	    command.div = 4;
	    return;
	}

    Trajectory *ReadControls(std::string filename)
	{
	    unsigned int i;
	    float temp_float;
	    std::string line, temp;
	    Trajectory *traj;
	    std::ifstream file;
	    file.open(filename.c_str(), std::fstream::in);
	    // Read line telling us the number of data points:
	    getline(file, line);
	    std::stringstream ss(line);
	    ss >> temp >> num;
	    ROS_DEBUG("Number of time points = %d",num);

	    // Now we can initialize the trajectory struct:
	    size_t alloc;
	    alloc = sizeof(*traj) + sizeof(traj->vals[0])*num;
	    traj = (Trajectory*) malloc(alloc);
	    
	    // Now, we can start reading in the important file stuff:
	    for (i=0; i<num; i++)
	    {
		
		// fill out t
		getline(file, line, ',');
		std::stringstream ss(line);
		ss >> temp_float;
		traj->vals[i][0] = temp_float;
		// fill out robot vel
		getline(file, line);
		std::stringstream ss2(line);
		ss2 >> temp_float;
		traj->vals[i][1] = temp_float;

	    }
	    file.close();
	    
	    // Now we can set DT and the robot_index
	    ros::param::get("robot_index", traj->RobotMY);
	    traj->DT = traj->vals[1][0]-traj->vals[0][0];
	    traj->num = num;

	    return traj;
	} // END OF ReadControls()


};

// command_line parsing:
void command_line_parser(int argc, char** argv)
{
    std::string working_dir, file;
   
    // First set the global working directory to the location of the
    // binary:
    working_dir = argv[0];

    int fflag = 0, pflag = 0, rflag = 0;
    int robot_index = 0;
    int index;
    int c;
     
    opterr = 0;
     
    while ((c = getopt (argc, argv, "f:p:r:")) != -1)
    {
	switch (c)
	{
	case 'f':
	    fflag = 1;
	    file = optarg;
	    break;
	case 'p':
	    pflag = 1;
	    working_dir = optarg;
	    break;
	case 'r':
	    rflag = 1;
	    robot_index = atoi(optarg);
	    break;
	case ':':
	    fprintf(stderr,
		    "No argument given for command line option %c \n\r", c);
	    break;
	default:
	    fprintf(stderr, "Usage: %s [-f filename] [-p path-to-file]\n",
		    argv[0]);
	    exit(EXIT_FAILURE);
	}
    }
     
    for (index = optind; index < argc; index++)
	printf ("Non-option argument %s\n", argv[index]);
    if (pflag != 1)
    {
	// Then we just use the default path:
	std::size_t found = working_dir.find("bin");
	std::string tmp_dir = working_dir.substr(0, found);
	working_dir = tmp_dir+"data/";
    }
 
    if (fflag == 0)
    {
	// No file was given:
	file = "default.txt";
    }

    // did we spec rflag on command line?
    if (rflag != 1)
    {
	// let's see if there is a parameter already
	if(!ros::param::has("robot_index"))
	{
	    // use a default:
	    robot_index = 1;
	    ROS_INFO("Setting robot_index to default (%d)",robot_index);
	    ros::param::set("robot_index", robot_index);
	}
	else
	    ros::param::get("robot_index", robot_index);
    }
    else
    {
	// since we did spec it, let's set it
	ros::param::set("robot_index", robot_index);
    }
  
    // Get filenames:
    filename = working_dir + file;
    ROS_INFO("Filename: %s",filename.c_str());
    return;
}


//---------------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    ROSCONSOLE_AUTOINIT;
    
    // startup node
    ros::init(argc, argv, "pend_controller");
    ros::NodeHandle n;

    command_line_parser(argc, argv);

    PendControl controller;

    // infinite loop
    ros::spin();

    return 0;
}
