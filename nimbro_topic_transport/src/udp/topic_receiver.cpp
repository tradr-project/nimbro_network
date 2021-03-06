// Topic receiver (part of udp_receiver)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "topic_receiver.h"

#include <bzlib.h>

#if WITH_ZSTD
#include <zstd.h>
#endif

namespace nimbro_topic_transport
{

bool Message::decompress(Message* dest)
{
	unsigned long long destLen = 1024;

	ROS_DEBUG("decompressing message with flags %d", (int)header.flags);
	if(header.flags & UDP_FLAG_COMPRESSED)
	{
		dest->payload.resize(destLen);

		while(1)
		{
			unsigned int decompressedLen = destLen;

			int ret = BZ2_bzBuffToBuffDecompress((char*)dest->payload.data(), &decompressedLen, (char*)payload.data(), payload.size(), 0, 0);
			destLen = decompressedLen;

			if(ret == BZ_OUTBUFF_FULL)
			{
				destLen *= 2;
				dest->payload.resize(destLen);
				continue;
			}

			if(ret != BZ_OK)
			{
				ROS_ERROR("Could not decompress message");
				return false;
			}

			break;
		}
	}
	else if(header.flags & UDP_FLAG_ZSTD)
	{
#if WITH_ZSTD
		destLen = ZSTD_getDecompressedSize(payload.data(), payload.size());
		if(destLen == 0 || destLen >= (1 << 28)) // 256 MiB
		{
			ROS_WARN_THROTTLE(1.0, "Could not infer decompressed size. Maybe your message is larger than 256MiB?");
			destLen = 5ULL * 1024ULL * 1024ULL; // 5MiB
		}

		dest->payload.resize(destLen);


		ROS_DEBUG("Decompressing with ZSTD from %lu bytes", payload.size());
		size_t size = ZSTD_decompress(dest->payload.data(), destLen, payload.data(), payload.size());
		if(ZSTD_isError(size))
		{
			ROS_ERROR("Could not decompress message using ZSTD: %s", ZSTD_getErrorName(size));
			return false;
		}
		ROS_DEBUG("ZSTD: decompressed %lu to %lu", payload.size(), size);

		destLen = size;
#else
		ROS_ERROR_THROTTLE(1.0, "Received ZSTD compressed message, and I don't have ZSTD support compiled in. Dropping.");
#endif
	}

	dest->payload.resize(destLen);
	dest->header = header;
	dest->id = id;
	dest->size = destLen;
	return true;
}

TopicReceiver::TopicReceiver()
 : waiting_for_subscriber(true)
 , m_decompressionThreadRunning(false)
{
}

TopicReceiver::~TopicReceiver()
{
	{
		boost::unique_lock<boost::mutex> lock(m_mutex);
		m_decompressionThreadShouldExit = true;
	}
	m_cond.notify_one();
}

void TopicReceiver::takeForDecompression(const boost::shared_ptr<Message>& msg)
{
	ROS_DEBUG("takeForDecompression: %d", (int)msg->header.flags);
	if(!m_decompressionThreadRunning)
	{
		m_decompressionThreadShouldExit = false;
		m_decompressionThread = boost::thread(boost::bind(&TopicReceiver::decompress, this));
		m_decompressionThreadRunning = true;
	}

	boost::unique_lock<boost::mutex> lock(m_mutex);
	m_compressedMsg = msg;
	m_cond.notify_one();
}

void TopicReceiver::decompress()
{
	while(1)
	{
		boost::shared_ptr<Message> currentMessage;

		{
			boost::unique_lock<boost::mutex> lock(m_mutex);

			while(!m_compressedMsg && !m_decompressionThreadShouldExit)
				m_cond.wait(lock);

			if(m_decompressionThreadShouldExit)
				return;

			currentMessage = m_compressedMsg;
			m_compressedMsg.reset();
		}

		Message decompressed;
		currentMessage->decompress(&decompressed);

		boost::shared_ptr<topic_tools::ShapeShifter> shapeShifter(new topic_tools::ShapeShifter);
		shapeShifter->morph(md5_str, decompressed.header.topic_type, msg_def, "");

		shapeShifter->read(decompressed);

		publish(shapeShifter);
	}
}

void TopicReceiver::publish(const boost::shared_ptr<topic_tools::ShapeShifter>& msg)
{
	if(!waiting_for_subscriber)
	{
		publisher.publish(msg);
	}
	else
	{
		m_msgInQueue = msg;
		m_queueTime = ros::WallTime::now();
	}
}

void TopicReceiver::publishCompressed(const CompressedMsgConstPtr& msg)
{
	if(!waiting_for_subscriber)
	{
		publisher.publish(msg);
	}
	else
	{
		m_compressedMsgInQueue = msg;
		m_queueTime = ros::WallTime::now();
	}
}

void TopicReceiver::handleSubscriber()
{
	if(!waiting_for_subscriber)
		return;

	if(!m_compressedMsg && !m_msgInQueue)
	{
		// No msg arrived before the first subscriber
		waiting_for_subscriber = false;
		return;
	}

	ros::WallTime now = ros::WallTime::now();
	if(now - m_queueTime > ros::WallDuration(1.0))
	{
		// Message to old, drop it
		waiting_for_subscriber = false;
		m_msgInQueue.reset();
		m_compressedMsgInQueue.reset();
		return;
	}

	if(m_msgInQueue)
	{
		publisher.publish(m_msgInQueue);
		m_msgInQueue.reset();
	}
	else if(m_compressedMsgInQueue)
	{
		publisher.publish(m_compressedMsgInQueue);
		m_compressedMsgInQueue.reset();
	}

	waiting_for_subscriber = false;
}

}

