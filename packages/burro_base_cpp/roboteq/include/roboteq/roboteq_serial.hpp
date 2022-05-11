#ifndef ROBOTEQ_ROBOTEQ_SERIAL_HPP
#define ROBOTEQ_ROBOTEQ_SERIAL_HPP

#include <roboteq/roboteq.hpp>
#include <serial/serial.h>

class RoboteqSerialDevice : public RoboteqDevice
{
    serial::Serial device_;
public:
	RoboteqSerialDevice();

    virtual bool Open();
    virtual bool Close();
    virtual bool Read(std::string& response);
    virtual bool Write(const std::string& instruction);
    virtual bool Ready();

    serial::Serial& device(){return device_;}
};

#endif