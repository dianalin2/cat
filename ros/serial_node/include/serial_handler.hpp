#ifndef SERIAL_HANDLER_HPP
#define SERIAL_HANDLER_HPP

#include <string>
#include <boost/asio.hpp>

class SerialHandler
{
private:
    boost::asio::io_service ioService;
    boost::asio::serial_port serialPort;

public:
    SerialHandler(const std::string &port, unsigned int baud_rate);
    std::string readData();
};

#endif
