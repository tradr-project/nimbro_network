// Common methods for TCP & UDP transport
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "common.h"

#include <ros_babel_fish/generation/providers/integrated_description_provider.h>

namespace nimbro_service_transport
{

static ros_babel_fish::IntegratedDescriptionProvider descriptionProvider;

std::string getServiceMD5(const std::string& type)
{
	return descriptionProvider.getServiceDescription(type)->md5;
}

}

