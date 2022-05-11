#ifndef ROBOTEQ_ROBOTEQ_MULTI_DEVICE_HPP
#define ROBOTEQ_ROBOTEQ_MULTI_DEVICE_HPP

#include <roboteq/roboteq.hpp>

class MultiRoboteqDevice
{
    std::map<std::string, RoboteqDevice::Pointer> devices_;

public:
    template <class Device>
    std::shared_ptr<Device> AddDevice(
        const std::string& name,
        std::optional<std::shared_ptr<Device> > device = std::nullopt)
    {
        devices_[name] = device ? device.value() : std::make_shared<Device>();
        devices_[name]->SetId(name);
        return Get<Device>(name);
    }

    template <class Device>
    std::shared_ptr<Device> Get(const std::string& id)
    {
        return std::dynamic_pointer_cast<Device>(devices_.at(id));
    }

    bool Ready()
    {
        bool ready = true;
        for_each([&ready](auto& d){ready &= d.second->Ready();});
        return ready;
    }

    std::map<std::string, RoboteqDevice::Pointer>& devices()
    {
        return devices_;
    }

    void for_each(const auto &function)
    {
        std::for_each(devices_.begin(), devices_.end(), function);
    }
};

#endif