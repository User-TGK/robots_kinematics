#ifndef SSC32_DRIVER
#define SSC32_DRIVER

#include <cstdint>
#include <memory>
#include <stdio.h>
#include <string>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <ssc32u/Commands.h>
#include <ssc32u/ConfigParser.h>

namespace ssc32drv
{
/// \brief The main SSC32U controller class.
class SSC32
{
public:
	/// \brief {
	/// 	Constructor.
	/// 	See further documentation for a template of the config.
	/// 	Throws std::runtime_error when it cannot connect.
	/// }
	/// \param jsonData The json configuration string.
	SSC32(const std::string &jsonData);

	/// \brief Destructor.Disconnects from the serial port.
	~SSC32();

	/// \brief Checks if serial i/o is operational.
	/// \return true if the connection is still active.
	bool IsConnected() const;

	/// \brief Moves a single servo. This method moves a single servo
	/// according to the given MoveCmd. \param cmd A valid MoveCmd class
	/// with an optional speed.
	void MoveServo(const MoveCmd &cmd) const;

	/// \brief {
	/// 	Moves multiple servos.
	/// 	This method moves multiple servos according to the given
	/// MultiMoveCmd. 	NOTE: The time of the last element given to
	/// MultiMoveCmd will be used as time for all.
	/// }
	/// \param multicmd A valid MultiMoveCmd class which contains multiple
	/// MoveCmd objects.
	void MoveServo(const MultiMoveCmd &multicmd) const;

	/// \brief Cancel a certain command.
	/// \param cmd The cmd to cancel.
	void CancelCommand(const ServoCmd &cmd) const;

	/// \brief Stop all the servos from moving. (This will result in an
	/// incomplete move).
	void StopAllServos() const;

	/// \brief Stop a single server. (This will result in an incomplete
	/// move). \param channel The servo channel to stop.
	void StopServo(const uint8_t channel) const;

	/// \brief {
	/// 	Check if the last sent servo move command finished.
	/// 	There will be a serial delay of at least 50uS to 5mS before the
	/// response is sent. 	Note: if stopServo (or stopAllServos) is called,
	/// 	this will not return true because it never completed.
	/// }
	/// \return true when the move is complete.
	bool LastMoveComplete() const;

	/// \brief {
	/// 	Get the PWM of a certain channel with a resolution of 10 uS.
	/// 	There will be a serial delay of at least 50uS to 5mS
	/// 	before the response is sent Typically the response will
	/// 	be started within 100uS.
	/// }
	/// \param channel The channel to get the current PWM from.
	/// \return The current PWM of the specified channel.
	uint16_t GetFeedback(const uint8_t channel) const;

private:
	/// \brief Connect to a serial port.
	/// \param portname The path to the serial port.
	/// \param baudrate The baudrate to use.
	/// \return true if the connection is established.
	/// \exception Throws std::runtime_exception on error.
	bool Connect(const std::string &portname, uint32_t baudrate);

	/// \brief Disconnect from the serial port.
	void Disconnect();

	/// \brief Send a certain ServoCmd command. All servo commands inherit
	/// from ServoCmd. \param cmd The command to send over serial.
	void SendCommand(std::shared_ptr<ServoCmd> cmd) const;

	/// \brief The IO service object.
	boost::asio::io_service io;

	/// \brief The underlying serial port object,
	mutable boost::asio::serial_port port;

	/// \brief The settings of the driver.
	const ConnectionSettings settings;
};
} /* namespace ssc32drv */

#endif