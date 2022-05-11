

#include <roboteq/roboteq.hpp>

////// Device //////
RoboteqDevice::RoboteqDevice(){}

RoboteqDevice::~RoboteqDevice()
{
    Unsubscribe();

    read_thread_.join();
    callback_thread_.join();

    Close();
}

bool RoboteqDevice::Write(const Command& command)
{
    return Write(command.Compile());
}

bool RoboteqDevice::Write(const std::list<Command>& commands)
{
    bool success = true;

    for (auto& command : commands)
    {
        success &= Write(command);
    }
    return success;
}

bool RoboteqDevice::Read(Response& response)
{
    std::string input;
    response.first = Read(input);

    if (response.first)
    {
        response.second = Command(input);
    }

    return response.first;
}

void RoboteqDevice::Subscribe(
    const Callback& callback,
    const std::optional<std::string>& cmd)
{
    if (cmd)
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        callbacks_[cmd.value()] = callback;
    }
    else
    {
        Unsubscribe();
        std::lock_guard<std::mutex> lock(worker_mutex_);
        callbacks_["default"] = callback;
        read_thread_ = std::thread(&RoboteqDevice::ReadDispatcher, this);
        callback_thread_ = std::thread(&RoboteqDevice::CallbackDispatcher, this);
    }
}

void RoboteqDevice::Unsubscribe(const std::optional<std::string>& cmd)
{
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (cmd)
    {
        callbacks_.erase(cmd.value());
    }
    else
    {
        callbacks_.clear();
    }
}

void RoboteqDevice::ReadDispatcher()
{
    while (callbacks_.size())
    {
        Response response;
        if (Ready() && Read(response))
        {
           response_queue_.push(response);
        }
        sleep(0.005);
    }
}

void RoboteqDevice::CallbackDispatcher()
{
    while(callbacks_.size())
    {
        {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        while (!response_queue_.empty())
        {
            callbacks_["default"](response_queue_.front(), id_);
            response_queue_.pop();
        }
        }
        sleep(0.005);
    }
}
