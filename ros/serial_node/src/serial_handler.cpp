#include "serial_handler.hpp"
#include "rclcpp/rclcpp.hpp"

SerialHandler::SerialHandler(const std::string &port, unsigned int baudRate)
    : ioService(), serialPort(ioService, port)
{
    try
    {
        serialPort.open(port);
        serialPort.set_option(boost::asio::serial_port_base::baud_rate(baudRate));
        serialPort.set_option(boost::asio::serial_port_base::character_size(8));
        serialPort.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serialPort.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serialPort.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    }
    catch (const boost::system::system_error &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("SerialHandler"), "Failed to open serial port: %s", e.what());
    }
}

std::string SerialHandler::readData()
{
    // Read 8 bytes from the serial port
    boost::asio::streambuf buffer;
    boost::asio::read(serialPort, buffer, boost::asio::transfer_exactly(8));

    std::istream is(&buffer);
    std::string data(8, '\0');
    is.read(&data[0], 8);

    if (is.gcount() < 8)
        RCLCPP_WARN(rclcpp::get_logger("SerialHandler"), "Less than 8 bytes read from serial port");

    return data;
}
