// for ROS nodes basic needs
#include <ros/ros.h>

// for reading the vmem size from /proc/self/status
#include <fstream>
#include <string>
#include <sstream>

// the custom-made latency test message
#include "pubsub/latency_test_message.h"

int main(int argc, char **argv)
{
	float publish_rate = 1;
	float shutdown_after_sending = 10;
	float message_payload_size = 0;
	long int sent_count = 0;
	char* message_string;

	ros::init(argc, argv, "talker");

	ros::NodeHandle nh;
	pubsub::latency_test_message msg;

	ros::Publisher chatter_pub = nh.advertise<pubsub::latency_test_message>("chatter", 1000);

	ros::param::get("~/publish_rate", publish_rate);
	ros::param::get("~/shutdown_after_sending", shutdown_after_sending);
	ros::param::get("~/message_payload_size", message_payload_size);

	if (message_payload_size > 0) {
		message_string = (char *)malloc((size_t)message_payload_size);
		for (long int i=0; i < message_payload_size; i++) {
			message_string[i] = i%10 + '0';
		}
		msg.payload.append(message_string);
		free(message_string);
	}

	ros::Rate loop_rate((double)publish_rate);

	ROS_INFO("Publisher ('talker') loop starts now with publish rate %ld", (long int)publish_rate);
	ROS_INFO("Publisher ('talker') will shutdown after publishing %ld messages", (long int)shutdown_after_sending);
	ROS_INFO("Publisher ('talker') will publish messages with a size of %ld bytes", (long int)message_payload_size);

	while (ros::ok() && sent_count < (long int)shutdown_after_sending)
	{
		msg.header.seq = sent_count;

		// begin vmem size code
		// source copied from MIRA benchmark and adapted for re-use here

		std::ifstream statStream("/proc/self/status", std::ios_base::in);
		std::string line;
		bool vmem_found = false;
		while(!std::getline(statStream, line).eof() && !vmem_found)
		{
			// format of line we are looking for:
			// VmSize:	   28812 kB
			if (line.find("VmSize") != std::string::npos)
			{
				vmem_found = true;
				sscanf(line.c_str(),"VmSize:%ld kB", &(msg.virtual_memory_size));
				// important note: a throttled console message uses quite some extra cpu power, so
				//				   be aware of that when you turn debug on using e.g. rqt_logger_level
				ROS_DEBUG_THROTTLE(1.0,">>>> sscanf of line %s gives pub virtual_memory_size %ld",
									line.c_str(), msg.virtual_memory_size);
			}
		}
		// end vmem size code

		ros::WallTime wallTime = ros::WallTime::now();
		msg.header.stamp.sec = wallTime.sec;
		msg.header.stamp.nsec = wallTime.nsec;

		chatter_pub.publish(msg);
		sent_count++;

		loop_rate.sleep();
	}

	ros::shutdown();

	return 0;
} //main

