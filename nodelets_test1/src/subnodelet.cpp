// for ROS nodelets basic needs
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

// for reading the vmem size from /proc/self/status
#include <fstream>
#include <string>
#include <sstream>

// the custom-made latency test message
#include "nodelets_test1/latency_test_message.h"

namespace nodelets_test1
{

	class Subscriber : public nodelet::Nodelet
	{
	public:
		Subscriber() : report_count_interval_(60), received_counter_(0), last_sequence_received_(0)
		{
			NODELET_INFO("Nodelet subscriber constructor entered");
		}

	private:
		virtual void onInit()
		{
			NODELET_INFO("Nodelet subscriber onInit() entered");
			ros::NodeHandle& nh = getNodeHandle();
			ros::NodeHandle& private_nh = getPrivateNodeHandle();
			private_nh.getParam("report_count_interval", report_count_interval_);
			NODELET_INFO("Nodelet subscriber report count interval is set to %ld", (long int)report_count_interval_);
			sub_ = nh.subscribe("/nodelets_test1_out", 1000, &Subscriber::callback_, this);
		}

#define NANO 1000000000.0

		void callback_(const nodelets_test1::latency_test_messagePtr msg) // if full msg received, ptr will be created automatically
		{
			now_ = ros::WallTime::now();
			received_counter_++;
			if (last_sequence_received_ == 0) {
				last_sequence_received_ = msg->header.seq;
			} else {
				if (msg->header.seq != (last_sequence_received_ + 1)) {
					NODELET_ERROR("Possible message data corruption detected; expecting msg seq %ld but received %d",
									last_sequence_received_+1, msg->header.seq);
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
						NODELET_DEBUG_THROTTLE(1.0,">>>> sscanf of line %s gives sub virtual_memory_size %ld",
												line.c_str(), virtual_memory_size);
					}
				}
				// end vmem size code

				NODELET_DEBUG("Last sequence number was %ld, now showing the next message:", last_sequence_received_);
				NODELET_INFO("seq %d siz %ld latency %1.9f s. vmem pub-sub: %ld-%ld kB; ptr pub-sub: %p-%p",
								msg->header.seq, msg->payload.size(),
								(float)(((now_.sec * NANO + now_.nsec) - (msg->header.stamp.sec * NANO + msg->header.stamp.nsec))/NANO),
								msg->virtual_memory_size,
								virtual_memory_size,
								// msg can not be typecasted to a void* directly, hence (void *)&(*msg)
								(void *)msg->latency_test_messagePtr,(void *)&(*msg));

			}
			last_sequence_received_ = msg->header.seq;
		}

		ros::Subscriber sub_;
		double report_count_interval_;
		long int received_counter_;
		long int last_sequence_received_;
		ros::WallTime now_;

	}; // class

	PLUGINLIB_EXPORT_CLASS(nodelets_test1::Subscriber, nodelet::Nodelet);
} // namespace
