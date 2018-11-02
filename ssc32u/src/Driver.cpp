#include <ssc32u/Driver.h>

#include <chrono>
#include <iostream>
#include <sstream>

namespace ssc32drv
{
SSC32::SSC32(const std::string &cfgPath) : port(io), settings(cfgPath)
{
	if (!this->Connect(this->settings.port, this->settings.baudrate)) {
		throw std::runtime_error("Could not open port");
	}
}

SSC32::~SSC32()
{
	this->Disconnect();
}

bool SSC32::Connect(const std::string &portname, uint32_t baudrate)
{
	using namespace boost::asio;
	this->port.open(portname);

	this->port.set_option(serial_port::baud_rate(baudrate));
	this->port.set_option(
		serial_port::flow_control(serial_port::flow_control::none));

	return this->port.is_open();
}

void SSC32::SendCommand(std::shared_ptr<ServoCmd> cmd) const
{
	std::string buf = cmd->GetBuffer();

	buf += " \r";
	boost::asio::write(this->port, boost::asio::buffer(buf));
};

void SSC32::Disconnect()
{
	this->io.stop();
}

bool SSC32::IsConnected() const
{
	return this->port.is_open();
}

void SSC32::StopServo(const uint8_t channel) const
{
	this->SendCommand(std::make_shared<StopCmd>(StopCmd(channel)));
}

void SSC32::StopAllServos() const
{
	// For each possible channel
	for (uint8_t ch = 0; ch < 32; ch++) {
		this->StopServo(ch);
	}
}

void SSC32::MoveServo(const MoveCmd &cmd) const
{
	this->SendCommand(std::make_shared<MoveCmd>(cmd));
}

void SSC32::MoveServo(const MultiMoveCmd &multicmd) const
{
	this->SendCommand(std::make_shared<MultiMoveCmd>(multicmd));
}

void SSC32::CancelCommand(const ServoCmd &cmd) const
{
	ServoCmd cancelCmd = cmd;
	cancelCmd.AddCancelSpecifier();
	this->SendCommand(std::make_shared<ServoCmd>(cancelCmd));
}

bool SSC32::LastMoveComplete() const
{
	QueryDoneCmd cmd = QueryDoneCmd();
	this->SendCommand(std::make_shared<QueryDoneCmd>(cmd));

	char rawBuf[1];
	this->port.read_some(boost::asio::buffer(rawBuf));

	return rawBuf[0] == '.';
}

uint16_t SSC32::GetFeedback(const uint8_t channel) const
{
	QueryPWMCmd query = QueryPWMCmd(channel);
	this->SendCommand(std::make_shared<QueryPWMCmd>(query));

	std::uint8_t rawBuf[1];
	this->port.read_some(boost::asio::buffer(rawBuf));

	return rawBuf[0] * 10;
}
}