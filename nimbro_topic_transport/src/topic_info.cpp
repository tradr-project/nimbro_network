// Provides information about topic types
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "topic_info.h"

#include <ros_babel_fish/generation/providers/integrated_description_provider.h>

namespace nimbro_topic_transport
{

namespace topic_info
{

static ros_babel_fish::IntegratedDescriptionProvider descriptionProvider;

std::string getMsgDef(const std::string& type)
{
  return descriptionProvider.getMessageDescription(type)->specification;
}

std::string getMd5Sum(const std::string& type)
{
  return descriptionProvider.getMessageDescription(type)->md5;
}

void packMD5(const std::string& str, LEValue< 4 >* dest)
{
	for(int i = 0; i < 4; ++i)
	{
		std::string md5_part = str.substr(8*i, 8);
		uint32_t md5_num = strtoll(md5_part.c_str(), 0, 16);
		dest[i] = md5_num;
	}
}

void unpackMD5(const LEValue< 4 >* src, std::string* dest)
{
	dest->clear();
	for(int i = 0; i < 4; ++i)
	{
		char buf[10];
		snprintf(buf, sizeof(buf), "%08x", src[i]());
		*dest += buf;
	}
}

}

}
