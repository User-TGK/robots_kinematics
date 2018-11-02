#ifndef PARSER_HPP_
#define PARSER_HPP_

#include <Specification.hpp>

#include <regex>

/**
 * @brief the parser contains all functionality to parse batch files and specifications.
 * 
 */
class Parser
{
public:
    /**
     * @brief default constructor of the specification parser
     */
    Parser();

    /**
     * @brief destructor of the specification parser
     */
    virtual ~Parser();

    /**
     * @brief function that tries to create a specification object of a string
     * 
     * @param str the string to be converted to a specification
     * @param specification reference to the specification where the string data will be stored in
     * @return true if the string was successfully converted
     * @return false if the string wasnt successfully converted
     * @exception if the string could not be converted
     */
    bool tryParseSpecificationFromStr(const std::string& str, Specification& specification);

    /**
     * @brief function that tries to convert a batchfile into a list of specifications 
     * 
     * @param batchConfigPath the path to the batchfile that contains the specifications
     * @param specifications the list where the converted specifications will be stored in
     * @return true if the convertion was successfull
     * @return false if the batch file couldnt be converted
     * @exception if the batchfile contains syntax errors
     */
    bool tryParseBatchConfig(const std::string& batchConfigPath, std::vector<Specification>& specifications);
    
    /**
     * @brief function that converts a string into a supported color 
     * (read src/SpecificationConfig.hpp for the supported colors)
     * 
     * @param str the string to be converted
     * @return const supportedColor the supported color instance
     * @exception if an invalid string was given
     */
    supportedColor colorFromStr(const std::string& str) const;

    /**
     * @brief function that converts a string into a supported shape
     * (read src/SpecificationConfig.hpp for the supported shapes)
     * 
     * @param str the string to be converted
     * @return const supportedShape the supported shape instance
     * @exception if an invalid string was given
     */
    supportedShape shapeFromStr(const std::string& str) const;

    /**
     * @brief function that converts a color into its string representation
     * 
     * @param color the color instance to be converted
     * @return const std::string the string representation of the color
     */
    const std::string strFromColor(const supportedColor& color);

    /**
     * @brief function that converts a shape into its string representation
     * 
     * @param shape the shape instance to be converted
     * @return const std::string the string representation of the shape
     */
    const std::string strFromShape(const supportedShape& shape);

private:
    /** The regular expression that describes the specification [shape][whitespace][color] */
    std::regex specificationRegex = std::regex("^([\\S|\\s]+)\\s+(\\S+)");
};

#endif // PARSER_HPP_