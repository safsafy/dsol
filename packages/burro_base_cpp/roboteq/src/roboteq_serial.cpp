#include <roboteq/roboteq_serial.hpp>


RoboteqSerialDevice::RoboteqSerialDevice()
{
    device_.setPort("/dev/ttyUSB1");
    device_.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    device_.setTimeout(to);
}

bool RoboteqSerialDevice::Open()
{
    try
    {
        device_.open();
    }
    catch (serial::IOException& e)
    {
        std::cerr << "Unable to open device on " << device_.getPort() << std::endl;
        std::cerr << e.what() << std::endl;
    }
    return device_.isOpen();
}

bool RoboteqSerialDevice::Close()
{
    if (device_.isOpen())
    {
        std::cout << "Closing device on " << device_.getPort() << std::endl;
        device_.close();
    }
    return true;
}

bool RoboteqSerialDevice::Read(std::string& response)
{
    try
    {
        response += device_.readline(65536, "\r");
    } catch (serial::IOException& e)
    {
        std::cerr << "Read failed on " << device_.getPort() << std::endl;
        std::cerr << e.what() << std::endl;
        return false;
    }
    return !response.empty();
}

bool RoboteqSerialDevice::Write(const std::string& instruction)
{
    try
    {
        device_.write(instruction);
    } catch (serial::IOException& e)
    {
        std::cerr << "Write failed: " << instruction << " on " << device_.getPort() << std::endl;
        std::cerr << e.what() << std::endl;
        return false;
    }
    return true;
}

bool RoboteqSerialDevice::Ready()
{
    return device_.isOpen();
}