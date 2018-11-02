#ifndef COMMANDS_HPP
#define COMMANDS_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace ssc32drv
{
/// \brief Abstract class for a servo command.
class ServoCmd
{
public:
	/// \brief Constructor
	ServoCmd()
	{
	}

	/// \brief Get the string buffer that will go over the line.
	/// \return A const reference to the string buffer.
	const std::string &GetBuffer() const
	{
		return cmdstr;
	}

	/// \brief {
	/// 	Adds <esc> to the end of the buffer.
	///		This is usefull when you want to send the same command
	///again but cancel it.
	/// 	Use SSC32::cancelCommand() instead.
	/// }
	void AddCancelSpecifier()
	{
		cmdstr += "<esc>";
	}

protected:
	/// \brief Serialise the given parameters, can be overriden.
	virtual void Serialise(){};

	/// \brief The actual string used to store the output.
	std::string cmdstr;
};

/// \brief Move a certain servo.
class MoveCmd : public ServoCmd
{
public:
	/// \brief Constructs the command.
	/// \param channel the channel of the servo.
	/// \param pulseWidth he PWM to move to.
	/// \param time time in microsecond to travel from current to desired
	/// pos, this affects all servos. Maximum 65535. \param speed optional
	/// Servo movement speed in microseconds per second.
	MoveCmd(const uint8_t channel, const uint16_t pulseWidth,
		const uint16_t time, const int32_t speed = -1);

	/// \brief Remove the time from this command.
	void IgnoreTime();

private:
	/// \brief Serialise the given parameters.
	void Serialise() override;

	/// \brief The channel this move is meant for.
	const uint8_t channel;

	/// \brief The pulse width to set the channel to. In milliseconds
	const uint16_t pulseWidth;

	/// \brief The time to reach the desired destination. In milliseconds
	const uint32_t time;

	/// \brief The desired speed of the servo in microseconds per second.
	const int32_t speed;
};

/// \brief This command moves multiple servos.
class MultiMoveCmd : public ServoCmd
{
public:
	/// \brief {
	/// 	Construct a MultiMoveCmd from a vector of MoveCmd objects.
	/// 	The time of the last element in the vector will be used.
	/// }
	/// \param cmds the vector of MoveCmd commands.
	MultiMoveCmd(std::vector<MoveCmd> cmds);

private:
	std::vector<MoveCmd> cmds;
};

/// \brief This command requests if the last move command is finished.
class QueryDoneCmd : public ServoCmd
{
public:
	/// \brief Constructor
	QueryDoneCmd()
	{
		this->Serialise();
	}

private:
	/// \brief Serialise the given parameters.
	void Serialise() override
	{
		cmdstr = "Q";
	}
};

/// \brief This command requests the pwm of a certain channel.
class QueryPWMCmd : public ServoCmd
{
public:
	/// \brief Query a channel for the current PWM setting.
	/// \param channel channel the channel to request the PWM from.
	QueryPWMCmd(const uint8_t channel);

private:
	/// \brief Serialise the given parameters.
	void Serialise() override;

	/// \brief The channel to query.
	const uint8_t channel;
};

/// \brief This command stops a servo from moving.
class StopCmd : public ServoCmd
{
public:
	/// \brief Constructor
	/// \param channel the channel to stop moving.
	StopCmd(const uint8_t channel);

private:
	/// \brief Serialise the given parameters.
	void Serialise() override;

	/// \brief The channel to stop.
	const uint8_t channel;
};
}

#endif