#include <ssc32u/Commands.h>

#include <iostream>
#include <sstream>

namespace ssc32drv
{
MoveCmd::MoveCmd(const uint8_t channel, const uint16_t pulseWidth,
	const uint16_t time, const int32_t speed) :
	channel(channel),
	pulseWidth(pulseWidth), time(time), speed(speed)
{
	if (channel >= 32) {
		throw std::invalid_argument("Channel must be lower than 32");
	}

	this->Serialise();
}

void MoveCmd::Serialise()
{
	std::stringstream cmdstream;

	cmdstream << "#" << +this->channel << " P" << this->pulseWidth;

	if (speed > 0) {
		cmdstream << " S" << this->speed;
	}

	cmdstream << " T" << this->time;

	this->cmdstr = cmdstream.str();
}

void MoveCmd::IgnoreTime()
{
	const auto timepos = this->cmdstr.find_last_of('T');
	this->cmdstr = this->cmdstr.substr(0, timepos);
}

MultiMoveCmd::MultiMoveCmd(std::vector<MoveCmd> cmds)
{
	std::string tempstr;
	for (auto &cmd : cmds) {
		if (&cmd != &cmds.back()) {
			cmd.IgnoreTime();
		}
		tempstr = cmd.GetBuffer();
		this->cmdstr += tempstr;
	}
}

QueryPWMCmd::QueryPWMCmd(const uint8_t channel) : channel(channel)
{
	this->Serialise();
}

void QueryPWMCmd::Serialise()
{
	std::stringstream cmdstream;
	cmdstream << "QP " << +this->channel;
	this->cmdstr = cmdstream.str();
}

StopCmd::StopCmd(const uint8_t channel) : channel(channel)
{
	this->Serialise();
}

void StopCmd::Serialise()
{
	std::stringstream cmdstream;
	cmdstream << "STOP " << +this->channel;
	this->cmdstr = cmdstream.str();
}
} // namespace ssc32drv