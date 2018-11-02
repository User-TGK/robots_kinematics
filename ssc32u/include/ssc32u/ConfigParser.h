#ifndef CONFIG_PARSER_HPP
#define CONFIG_PARSER_HPP

#include <string>

namespace ssc32drv
{
/// \brief This struct loads and holds config data.
struct ConnectionSettings
{
	/// \brief {
	///		Constructor. Leads data from json string.
	///		This file must be according to a certain template,
	///		this will be given at the software design description.
	/// }
	/// \param jsonData The json data to use for the configuration.
	ConnectionSettings(const std::string &jsonData);

	/// \brief The port of the ssc32u is connected on.
	std::string port;
	/// \brief The baudrate setting of the serial port.
	uint32_t baudrate;
};
}

#endif