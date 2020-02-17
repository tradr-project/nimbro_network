#include <memory>

// Throttle /tf bandwidth
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <memory>
#include <set>
#include <string>
#include <vector>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <tf2/exceptions.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

std::unique_ptr<tf2_ros::Buffer> g_tf;
ros::Publisher pub;
std::set<std::string> frameIds;
bool sendStatic;
bool onlyChildFrames;

void sendTransforms()
{
    std::vector<std::string> frames;
    g_tf->_getFrameStrings(frames);

    tf2_msgs::TFMessage msg;
    msg.transforms.reserve(frames.size());

    for (const auto& frame: frames)
    {
        std::string parentFrame;
        if(!g_tf->_getParent(frame, ros::Time(0), parentFrame))
            continue;

        if (!frameIds.empty()) {
            if (frameIds.find(frame) == frameIds.end() && (onlyChildFrames || frameIds.find(parentFrame) == frameIds.end()))
                continue;
        }

        geometry_msgs::TransformStamped transform;
        try
        {
            transform = g_tf->lookupTransform(
                    parentFrame,
                    frame,
                    ros::Time(0), ros::Duration(0)
            );
        }
        catch(tf2::TransformException&)
        {
            continue;
        }

        if (!sendStatic && transform.header.stamp.sec == 0)
          continue;

        msg.transforms.push_back(transform);
    }

    pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_throttle");

    ros::NodeHandle nh("~");

    g_tf = std::make_unique<tf2_ros::Buffer>(ros::Duration(10.0));
    tf2_ros::TransformListener listener(*g_tf);

    double rate;
    nh.param("rate", rate, 4.0);

    XmlRpc::XmlRpcValue frameIdsParam;
    std::stringstream frameIdsString;
    nh.getParam("frame_ids", frameIdsParam);
    if (frameIdsParam.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (size_t i = 0; i < frameIdsParam.size(); ++i) {
            if (frameIdsParam[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
                frameIds.insert(static_cast<std::string>(frameIdsParam[i]));
                frameIdsString << " " << static_cast<std::string>(frameIdsParam[i]);
            } else {
                ROS_ERROR("Unknown value for frame_ids of type %i encountered. "
                          "Please, supply only frame names.", frameIdsParam[i].getType());
            }
        }
        ROS_INFO("Republishing only these TF frames:%s", frameIdsString.str().c_str());
    } else {
        ROS_INFO("Republishing all TF messages.");
    }

    sendStatic = nh.param("send_static_tfs", true);
    onlyChildFrames = nh.param("only_child_frames", false);

    ros::Timer timer = nh.createTimer(
            ros::Duration(1.0 / rate),
            boost::bind(&sendTransforms)
    );
    pub = nh.advertise<tf2_msgs::TFMessage>("tf", 1);

    ros::spin();

    return 0;
}
