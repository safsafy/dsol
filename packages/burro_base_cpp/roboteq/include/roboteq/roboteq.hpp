#ifndef ROBOTEQ_ROBOTEQ_HPP
#define ROBOTEQ_ROBOTEQ_HPP

#include <iostream>
#include <mutex>
#include <thread>
#include <queue>
#include <map>
#include <memory>
#include <chrono>
#include <functional>

#include <roboteq/commands.hpp>


inline void sleep(double seconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds((long int)(seconds * 1e3)));
}


////// Device //////

class RoboteqDevice
{
public:
    typedef std::shared_ptr<RoboteqDevice> Pointer;
    using Callback = std::function<void(Response& response, const std::string& id)>;

private:
    std::map<std::string, Callback> callbacks_;
    std::thread read_thread_;
    std::thread callback_thread_;
    std::queue<Response> response_queue_;

    std::string id_;

protected:
    std::mutex worker_mutex_;

public:
    RoboteqDevice();
    ~RoboteqDevice();

    /* Initiates communication channel with the device */
    virtual bool Open() = 0;
    /* Terminates communication channel with the device */
    virtual bool Close(){return false;};
    /* Reads a line of data from the device buffer */
    virtual bool Read(std::string& response) = 0;
    /* Writes a line to the device */
    virtual bool Write(const std::string& instruction) = 0;
    /* Returns true if the device is ready for read/write */
    virtual bool Ready(){return false;}

    /* Sets the ID for this motor driver */
    void SetId(const std::string& id){id_ = id;}
    const std::string& GetId() const {return id_;}

    bool Read(Response& response);
    bool Write(const Command& command);
    bool Write(const std::list<Command>& commands);
    void ReadDispatcher();
    void CallbackDispatcher();
    void Subscribe(const Callback& callback, const std::optional<std::string>& cmd = std::nullopt);
    void Unsubscribe(const std::optional<std::string>& cmd = std::nullopt);
};

#endif