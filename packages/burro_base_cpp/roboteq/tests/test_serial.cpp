#include <roboteq/roboteq_serial.hpp>

int main()
{
    RoboteqSerialDevice device;
    if (device.Open())
    {
        for (int i = 0; i < 10; i++)
        {
            std::string r;
            device.Read(r);
            std::cout << r << "\n";
            sleep(0.01);
        }
    }
    return 0;
}