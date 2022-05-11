#include <algorithm>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <roboteq/roboteq_serial.hpp>
#include <roboteq/roboteq_multi_device.hpp>


MultiRoboteqDevice roboteq;
ros::Subscriber cmd_sub;
ros::Publisher pub;


void BaseCallback(const Response& response, const std::string& id)
{
    if (response.first)
    {
        ROS_INFO("%s: %s", id.c_str(), response.second.instruction.c_str());
    } else
    {
        ROS_WARN("Bad response");
    }
}

void CommandCallback(const std_msgs::StringConstPtr& msg)
{
    roboteq.for_each([&msg](auto& d)
    {
        ROS_INFO("Writing to %s: %s", d.first.c_str(), msg->data.c_str());
        if (!d.second->Write(msg->data))
            ROS_WARN("%s: Write error", d.first.c_str());
    });
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboteq_demo");

    roboteq.AddDevice<RoboteqSerialDevice>("front")->device().setPort("/dev/ttyUSB0");
    roboteq.AddDevice<RoboteqSerialDevice>("rear")->device().setPort("/dev/ttyUSB1");

    bool success = true;
    roboteq.for_each([&success](auto& d){success &= d.second->Open();});
    if (success)
    {
        ros::NodeHandle nh;
        cmd_sub = nh.subscribe("/burro_base/command", 1, &CommandCallback);
        roboteq.for_each([](auto& d){d.second->Subscribe(BaseCallback);});

        ros::spin();
        roboteq.for_each([](auto& d){d.second->Close();});
    }

    return 0;
}