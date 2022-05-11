#ifndef ROBOTEQ_COMMANDS_HPP
#define ROBOTEQ_COMMANDS_HPP

#include <string>
#include <sstream>
#include <list>


struct Command;
typedef std::pair<bool, Command> Response;

struct Command
{
    std::string instruction;
    std::string channel;
    std::list<std::string> data;
    std::string description;
    double timestamp;

    Command(){};
    Command(const std::string& response)
    {
       instruction = response;
    }

    std::string Compile() const
    {
        std::stringstream ss;
        ss << instruction;
        if (!channel.empty()) ss << " " << channel;
        for (auto& d : data)
        {
            ss << " " << d;
        }
        ss << "_";
        return ss.str();
    }
};

#endif