#include <ssc32u/ConfigParser.h>

#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <iostream>

namespace ssc32drv
{
ConnectionSettings::ConnectionSettings(const std::string &jsonData)
{
	namespace pt = boost::property_tree;
	pt::ptree tree;

	boost::iostreams::array_source as(&jsonData[0], jsonData.size());
	boost::iostreams::stream<boost::iostreams::array_source> is(as);

	pt::read_json(is, tree);
	this->port = tree.get<std::string>("port");
	this->baudrate = tree.get("baudrate", 0);
}
}