class RoboteqFtdiDevice : public RoboteqDevice
{
    ftdi::Ftdi device_;
public:
    virtual bool Open();
    virtual bool Close();
    virtual bool Read(const std::string& response) const;
    virtual bool Write(const std::string& instruction) const;
}


bool RoboteqFtdiDevice::open(const std::string& port)
{
    try
    {
        device_.setPort(port);
        device_.setBaudrate(115200);
        serial::Timeout to(1000);
        device_.setTimeout(to);
        device_.open();
    }
    catch (serial::IOException& e)
    {
        std::cerr << "Unable to open port: " << port << std::endl;
        std::cerr << e.what() << std::endl;
    }
    return device_.isOpen();
}

bool RoboteqFtdiDevice::Close()
{
    if (device_.isOpen())
    {
        std::cout << "Closing device on " << device_.port << std::endl
        device_.close();
    }
    return true;
}


bool RoboteqFtdiDevice::Read(std::string& response) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
        response += device_.read(device_.available());
    } catch (serial::IOException& e)
    {
        std::cerr << "Read failed: " << device_.port << std::endl;
        std::cerr << e.what() << std::end;
        return false;
    }
    return true;
}

bool RoboteqFtdiDevice::Write(const std::string& instruction) const
{
    try
    {
        device_.write(instruction);
    } catch (serial::IOException& e)
    {
        std::cerr << "Write failed: " << instruction << " on " << device_.port << std::endl;
        std::cerr << e.what() << std::endl;
        return false;
    }
    return true;
}