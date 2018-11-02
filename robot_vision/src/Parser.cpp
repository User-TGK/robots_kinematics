#include <Parser.hpp>

#include <fstream>

Parser::Parser()
{
}

Parser::~Parser()
{
}

supportedColor Parser::colorFromStr(const std::string& str) const
{
    auto searchResult = supportedColorMap.find(str);
    if (searchResult == supportedColorMap.end())
    {
        throw std::invalid_argument("Exception in 'src/Parser.cpp': the string " + str + " is not a supported color.");
    }
    return searchResult->second;
}

supportedShape Parser::shapeFromStr(const std::string& str) const
{
    auto searchResult = supportedShapeMap.find(str);
    if (searchResult == supportedShapeMap.end())
    {
        throw std::invalid_argument("Exception in 'src/Parser.cpp': the string " + str + " is not a supported shape.");
    }
    return searchResult->second;
}

const std::string Parser::strFromColor(const supportedColor& color)
{
    for (const auto& pair : supportedColorMap)
    {
        if (pair.second == color)
        {
            return pair.first;
        }
    }
    throw std::invalid_argument("Exception in 'src/Parser.cpp': parsing a supported color to string datatype.");
}

const std::string Parser::strFromShape(const supportedShape& shape)
{
    for (const auto& pair : supportedShapeMap)
    {
        if (pair.second == shape)
        {
            return pair.first;
        }
    }
    throw std::invalid_argument("Exception in 'src/Parser.cpp': parsing a supported shape to string datatype.");
}

bool Parser::tryParseSpecificationFromStr(const std::string& str, Specification& specification)
{
    std::smatch result;
    if (!std::regex_match(str, result, this->specificationRegex))
    {
        throw std::invalid_argument("(One of) the specification(s) was/were invalid. A specification must be in the following form: '[shape] [color]'.");       
    }
    specification.shape = this->shapeFromStr(result[1]);
    specification.color = this->colorFromStr(result[2]);
    
    return true;
}

bool Parser::tryParseBatchConfig(const std::string& batchConfigPath, std::vector<Specification>& specifications)
{
    std::string spec;
    std::ifstream batchFile(batchConfigPath);

    if (batchFile.is_open())
    {
        while (getline(batchFile, spec))
        {
            // Strip each string from comments first
            spec = spec.substr(0, spec.find("#", 0));

            if (spec != "\n" && spec != "")
            {
                Specification newSpecification;
                tryParseSpecificationFromStr(spec, newSpecification);

                specifications.push_back(newSpecification);
            }
        }
    }
    else 
    {
        return false;
    }
    return true;
}