// for ROS nodes basic needs
#include <ros/ros.h>

// for reading the vmem size from /proc/self/status
#include <fstream>
#include <string>
#include <sstream>

// the custom-made latency test message
#include "pubsub/latency_test_message.h"

class Listener {
public:
	Listener(long int report_count_interval) {
		report_count_interval_ = report_count_interval;
		received_counter_ = 0;
		last_sequence_received_ = 0;
	};

	void chatterCallback(const pubsub::latency_test_message& msg);

protected:
	long int report_count_interval_;
	long int received_counter_;
	long int last_sequence_received_;
};

#define NANO 1000000000.0

void Listener::chatterCallback(const pubsub::latency_test_message& msg)
{
	ros::WallTime now = ros::WallTime::now();
	received_counter_++;
	if (last_sequence_received_ == 0) {
		last_sequence_received_ = msg.header.seq;
	} else {
		if (msg.header.seq != (last_sequence_received_ + 1)) {
			ROS_ERROR("Possible message data corruption detected; expecting msg seq %ld but received %d",
						last_sequence_received_+1, msg.header.seq);
		}
	}
	if (received_counter_ % (long int)report_count_interval_ == 0) {
		// begin vmem size code
		// source copied from MIRA benchmark and adapted for re-use here

		std::ifstream statStream("/proc/self/status", std::ios_base::in);
		std::string line;
		bool vmem_found = false;
		uint64_t virtual_memory_size = 0;
		while(!std::getline(statStream, line).eof() && !vmem_found)
		{
			// format of line we are looking for:
			// VmSize:	   28812 kB
			if (line.find("VmSize") != std::string::npos)
			{
				vmem_found = true;
				sscanf(line.c_str(),"VmSize:%ld kB", &virtual_memory_size);
				// important note: a throttled console message uses quite some extra cpu power, so
				//				   be aware of that when you turn debug on using e.g. rqt_logger_level
				ROS_DEBUG_THROTTLE(1.0,">>>> sscanf of line %s gives sub virtual_memory_size %ld",
									line.c_str(), virtual_memory_size);
			}
		}
		// end vmem size code

		ROS_DEBUG("Last sequence number was %ld, now showing the next message:", last_sequence_received_);
		ROS_INFO("seq %d siz %ld latency %1.9f s. vmem pub-sub: %ld-%ld kB",
					msg.header.seq, msg.payload.size(),
					(float)(((now.sec * NANO + now.nsec) - (msg.header.stamp.sec * NANO + msg.header.stamp.nsec))/NANO),
					msg.virtual_memory_size,
					virtual_memory_size);
	}
	last_sequence_received_ = msg.header.seq;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle nh;

	float report_count_interval = 600;
	std::string transport_hint = "normal";

	ros::param::get("~/report_count_interval", report_count_interval);
	ros::param::get("~/transport_hint", transport_hint);

	ROS_INFO("Subscriber ('listener') report count interval is set to %ld", (long int)report_count_interval);
	ROS_INFO("Subscriber ('listener') transport_hint is set to %s", transport_hint.c_str());

	Listener listener((long int)report_count_interval);

	ros::Subscriber sub;
	if (transport_hint == "udp") {
		sub = nh.subscribe("chatter", 1000, &Listener::chatterCallback, &listener, ros::TransportHints().udp());
		ROS_WARN("Using UDPROS for data transport");
	} else if (transport_hint == "tcp_nodelay") {
		sub = nh.subscribe("chatter", 1000, &Listener::chatterCallback, &listener, ros::TransportHints().tcpNoDelay(true));
		ROS_WARN("Using TCPROS with nodelay option for data transport");
	} else {
		sub = nh.subscribe("chatter", 1000, &Listener::chatterCallback, &listener);
	}

	ros::spin();

	return 0;
} //main

