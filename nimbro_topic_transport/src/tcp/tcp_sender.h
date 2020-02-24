// TCP sender
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TCP_SENDER_H
#define TCP_SENDER_H

#include <ros/node_handle.h>
#include <topic_tools/shape_shifter.h>
#include <arpa/inet.h>

#include "tcp_packet.h"

#if WITH_CONFIG_SERVER
#include <config_server/parameter.h>
#endif

#include <map>
#include <mutex>

#include "ros/message_event.h"

#include <std_srvs/Empty.h>

#include <nimbro_topic_transport/SenderStats.h>

namespace nimbro_topic_transport
{

class TCPSender
{
public:
	enum CompressionType
	{
		COMPRESSION_NONE,
		COMPRESSION_BZ2,
		COMPRESSION_ZSTD,
	};

	struct MessageOptions
    {
	  int flags;
	  CompressionType compression;
	  int compressionLevel;
    };

	TCPSender();
	~TCPSender();

	bool connect();

    void send(const std::string& topic, const std::string& publisher, MessageOptions& options, const topic_tools::ShapeShifter::ConstPtr& shifter, const bool reconnect = true);
    void messageCallback(const std::string& topic, MessageOptions& options,
        const ros::MessageEvent<topic_tools::ShapeShifter const>& shifter);
    void sendLatched();
    bool sendLatchedCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
private:
	void updateStats();
	void resendUnsentLatched();

	ros::NodeHandle m_nh;
	int m_fd;

	int m_addrFamily;
	sockaddr_storage m_addr;
	socklen_t m_addrLen;

	int m_sourcePort;
	std::vector<ros::Subscriber> m_subs;
	std::vector<uint8_t> m_packet;
	std::vector<uint8_t> m_compressionBuf;
    std::vector<std::string> m_ignoredPubs;
  std::recursive_mutex m_sendMutex;

  //! key = topic:caller_id, value = message:options
	std::map<std::pair<std::string, std::string>, std::pair<topic_tools::ShapeShifter::ConstPtr, MessageOptions> > m_latchedMessages;
	std::vector<std::tuple<std::string, std::string, MessageOptions, topic_tools::ShapeShifter::ConstPtr, bool>> m_unsentLatchedMessages;

#if WITH_CONFIG_SERVER
	std::map<std::string, boost::shared_ptr<config_server::Parameter<bool>>> m_enableTopic;
#endif

	nimbro_topic_transport::SenderStats m_stats;
	ros::Publisher m_pub_stats;
	ros::WallDuration m_statsInterval;
	ros::WallTimer m_statsTimer;
	ros::Timer m_resendLatchedTimer;
	uint64_t m_sentBytesInStatsInterval;
	std::map<std::string, uint64_t> m_topicSendBytesInStatsInteral;
    ros::ServiceServer m_latchedMessageRequestServer;
};

}

#endif
