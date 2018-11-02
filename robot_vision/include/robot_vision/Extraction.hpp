#ifndef EXTRACTION_HPP_
#define EXTRACTION_HPP_

#include <Parser.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

/**
 * @brief The extractor class filters a raw image untill a simple binary image is left with only the pixels remaining
 * that were in range of the configured color range (minus noise). The feature-extraction must always happen before
 * feature-detection (read Detection.hpp).
 * 
 */
class Extractor
{
public:
    /**
     * @brief Construct a new Extractor object
     */
    Extractor();

    /**
     * @brief Destroy the Extractor object
     */
    virtual ~Extractor();

    /**
     * @brief function that extracts the image based on a selected color
     * All pixels that are not in range of the color-range and noise will be removed
     * 
     * @param original the raw (not converted) image object
     * @param color the requested color
     * @return cv::Mat the extracted image (with only the relevant pixels remaining)
     */
    cv::Mat extract(const cv::Mat& original, const supportedColor& color) const;

    /**
     * @brief function that parses a config file with color ranges to a map
     * 
     * @return std::map<supportedColor, colorRange> the converted ranges stored in a map
     */
    std::map<supportedColor, colorRange> mappedColorRangesMapFromFile() const;

    /**
     * @brief function that parses a map with color ranges to a config file
     * 
     * @param aMappedColorRangesMap the map to be parsed
     */
    void mappedColorRangesMapToFile(const std::map<supportedColor, colorRange>& aMappedColorRangesMap);

private:
    /** This map contains the color ranges (HSV format) for each supported color */
    std::map<supportedColor, colorRange> mappedColorRangesMap;

    /** Regex that described the color ranges config specifications: each line contains settings for one color,
     *  that matches the follow specification: [color name] [minHue] [maxHue] [minSat] [maxSat] [minValue] [maxValue]
     *  A valid example is: 'rood 0 255 0 255 0 255' (with one whitespace as delimiter) */
    std::regex rangeRegex = std::regex("(\\S+)\\s{1}(\\d+)\\s{1}(\\d+)\\s{1}(\\d+)\\s{1}(\\d+)\\s{1}(\\d+)\\s{1}(\\d+)");

    /**
     * @brief function that splits a string on a delimiter
     * 
     * @param str the string to be parsed
     * @param delim the delimiter the string is parsed on
     * @return std::vector<std::string> the parsed string
     */
    std::vector<std::string> strSplit(const std::string& str, const std::string& delim) const;
};

#endif //EXTRACTION_HPP_