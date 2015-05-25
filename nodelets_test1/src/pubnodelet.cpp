// for ROS nodelets basic needs
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

// for creating the main publishing thread
#include <boost/thread.hpp>
#include <boost/bind.hpp>

// for reading the vmem size from /proc/self/status
#include <fstream>
#include <string>
#include <sstream>

// the custom-made latency test message
#include "nodelets_test1/latency_test_message.h"

namespace nodelets_test1
{

	class Publisher : public nodelet::Nodelet
	{
	public:

		~Publisher()
		{
			thread.interrupt();
			thread.join();
			if (message_string_) free(message_string_);
		}

		Publisher() : subPtr_(NULL)
		, publish_rate_(1)
		, shutdown_after_sending_(1)
		, message_payload_size_(0)
		, use_message_pointer_(false)
		, every_message_new_(false)
		, sent_count_(0)
		, message_string_(NULL)
		{
			NODELET_INFO("Nodelet publisher constructor entered");
		}
		boost::thread thread;

	private:
		virtual void onInit()
		{
			NODELET_INFO("Nodelet publisher onInit() entered");
			ros::NodeHandle& nh_ = getNodeHandle();
			ros::NodeHandle& private_nh_ = getPrivateNodeHandle();
			private_nh_.getParam("publish_rate", publish_rate_);
			private_nh_.getParam("shutdown_after_sending", shutdown_after_sending_);
			private_nh_.getParam("message_payload_size", message_payload_size_);
			private_nh_.getParam("use_message_pointer", use_message_pointer_);
			private_nh_.getParam("every_message_new", every_message_new_);
			pub_ = nh_.advertise<latency_test_message>("/nodelets_test1_out", 1000);

			if (use_message_pointer_) {
				NODELET_WARN("Using shared memory message pointer for fastest transport");
			} else {
				NODELET_WARN("No message pointer will be used, sending full messages");
			}

			if (every_message_new_) {
				NODELET_WARN("Each message will be instantiated just before publishing");
			} else {
				NODELET_WARN("A pre-allocated message will be used, publish() will send that object each time");
			}

			NODELET_INFO("Nodelet publisher will shutdown after publishing %ld messages", (long int)shutdown_after_sending_);
			NODELET_INFO("Nodelet publisher will publish messages with a payload size of %ld bytes", (long int)message_payload_size_);

			// create the payload content
			if (message_payload_size_ > 0) {
				message_string_ = (char *)malloc((size_t)message_payload_size_);
				for (long int i=0; i < message_payload_size_; i++) {
					message_string_[i] = i%10 + '0';
				}
			}

			// we need to create our own thread
			thread = boost::thread(boost::bind(&Publisher::Publish_loop, this));
		}

		void addPayload(latency_test_messagePtr messagePtr)
		{
			if (message_payload_size_ > 0) {
				messagePtr->payload.append(message_string_);
			}
		}

		void publishMessage(latency_test_messagePtr messagePtr)
		{
			messagePtr->header.seq = sent_count_;

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
					sscanf(line.c_str(),"VmSize:%ld kB", &(messagePtr->virtual_memory_size));
					// important note: a throttled console message uses quite some extra cpu power, so
					//				   be aware of that when you turn debug on using e.g. rqt_logger_level.
					NODELET_DEBUG_THROTTLE(1.0,">>>> sscanf of line %s gives pub virtual_memory_size %ld",
											line.c_str(), messagePtr->virtual_memory_size);
				}
			}
			// end vmem size code

			messagePtr->latency_test_messagePtr = (uint64_t)&(*messagePtr);  // to verify at receiver's end

			ros::WallTime wallTime = ros::WallTime::now();
			messagePtr->header.stamp.sec = wallTime.sec;
			messagePtr->header.stamp.nsec = wallTime.nsec;

			if (use_message_pointer_) {
				pub_.publish(messagePtr);         // important note: we pass a pointer to the message, not its data
			} else {
				pub_.publish(*messagePtr);        // important note: we pass the message data
			}
		}

		void Publish_loop()
		{
			ros::Rate loop_rate(publish_rate_);
			latency_test_messagePtr preallocMessagePtr;
			latency_test_message::Ptr newMessagePtr;

			if (!every_message_new_) {
				preallocMessagePtr.reset(new latency_test_message);
				addPayload(preallocMessagePtr);
			}

			NODELET_INFO("Nodelet publisher loop starts now with publish rate %ld", (long int)publish_rate_);
			while (!boost::this_thread::interruption_requested() && sent_count_< (long int)shutdown_after_sending_) {

				if (every_message_new_) {
					latency_test_messagePtr newMessagePtr(new latency_test_message);
					addPayload(newMessagePtr);
					publishMessage(newMessagePtr);
				} else {
					publishMessage(preallocMessagePtr);
				}

				sent_count_++;

				loop_rate.sleep();
			}
			ros::shutdown();
		}

		ros::Publisher pub_;
		ros::Subscriber* subPtr_;
		double publish_rate_;
		double shutdown_after_sending_;
		double message_payload_size_;
		bool use_message_pointer_;
		bool every_message_new_;
		long int sent_count_;
		char * message_string_;

	}; // class


	PLUGINLIB_EXPORT_CLASS(nodelets_test1::Publisher, nodelet::Nodelet);
} // namespace
