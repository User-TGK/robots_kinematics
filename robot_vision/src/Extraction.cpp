#include <Extraction.hpp>

#include <opencv2/photo.hpp>

#include <iostream>
#include <sstream> 
#include <fstream>
#include <stdexcept>
#include <ros/package.h>

Extractor::Extractor()
{
    this->mappedColorRangesMap = mappedColorRangesMapFromFile();
}

Extractor::~Extractor()
{
}

cv::Mat Extractor::extract(const cv::Mat& original, const supportedColor& color) const
{
    cv::Mat originalCopy, denoised, bilateralFiltered, hsvFiltered, morphology, threshFiltered;
    originalCopy = original;

    // Remove all (color-) noise from the images
    cv::fastNlMeansDenoisingColored(originalCopy, denoised, 5, 15, 5, 11);

    cv::bilateralFilter(denoised, bilateralFiltered, 15, 80, 80);

    // Converting BGR (original color format) to HSV
    cv::cvtColor(bilateralFiltered, hsvFiltered, CV_BGR2HSV);

    // This find will always give a result since the color was validated before subject to the filter process
    auto searchResult = mappedColorRangesMap.find(color);
    auto range = searchResult->second;

    // Remove all pixels that are not in the selected color range
    cv::inRange(hsvFiltered, cv::Scalar(range.minHue, range.minSaturation, range.minValue), 
                            cv::Scalar(range.maxHue, range.maxSaturation, range.maxValue), threshFiltered);

    // Morphology filters
    unsigned int filterSize = originalCopy.cols / 250;
    cv::Mat element = cv::getStructuringElement(0, cv::Size(filterSize, filterSize));
    
    cv::morphologyEx(threshFiltered, morphology, cv::MORPH_CLOSE, element);
    cv::morphologyEx(morphology, morphology, cv::MORPH_CLOSE, element);
    cv::morphologyEx(morphology, morphology, cv::MORPH_OPEN, element);

    return morphology;
}

std::map<supportedColor, colorRange> Extractor::mappedColorRangesMapFromFile() const
{
    std::map<supportedColor, colorRange> aMappedColorRangesMap;
    Parser parser;

    std::string rangeLine;
    std::ifstream calibrationFile((ros::package::getPath("robot_vision") + "/calibration/calibration.ini").c_str());

    if (calibrationFile.is_open())
    {
        while(getline(calibrationFile, rangeLine))
        {
            if (rangeLine != "\n" && rangeLine != "")
            {
                std::smatch sResult;
                if(!std::regex_search(rangeLine, sResult, this->rangeRegex))
                {
                    throw std::invalid_argument("The calibration configuration is corrupt. Please clean calibration/calibration.ini and replace it with calibration/calibrationbackup.ini.");
                }

                std::vector<std::string> result = this->strSplit(rangeLine, " ");

                colorRange convertedRange;
                auto convertedColor = parser.colorFromStr(result[0]);

                convertedRange.minHue = std::stoi(result[1]);
                convertedRange.maxHue = std::stoi(result[2]);
                convertedRange.minSaturation = std::stoi(result[3]);
                convertedRange.maxSaturation = std::stoi(result[4]);
                convertedRange.minValue = std::stoi(result[5]);
                convertedRange.maxValue = std::stoi(result[6]);

                aMappedColorRangesMap.insert(std::pair<supportedColor, colorRange>(convertedColor, convertedRange));
            }
        }
        calibrationFile.close();
    }
    return aMappedColorRangesMap;
}

void Extractor::mappedColorRangesMapToFile(const std::map<supportedColor, colorRange>& aMappedColorRangesMap)
{
    Parser parser;
    std::ofstream calibrationFile((ros::package::getPath("robot_vision") + "/calibration/calibration.ini").c_str());

    if (calibrationFile.is_open())
    {
        for (const auto& pair : aMappedColorRangesMap)
        {
            std::stringstream rangeLine;
            rangeLine   << parser.strFromColor(pair.first) << " " << pair.second.minHue << " " 
                        << pair.second.maxHue << " " << pair.second.minSaturation << " " << pair.second.maxSaturation
                        << " " << pair.second.minValue << " " << pair.second.maxValue << "\n";
      
            calibrationFile << rangeLine.str();
        }
        calibrationFile.close();
    }
    else
    {
        throw std::invalid_argument("The calibration configuration is corrupt. Please clean calibration/calibration.ini and replace it with calibration/calibrationbackup.ini.");
    }
}

std::vector<std::string> Extractor::strSplit(const std::string& str, const std::string& delim) const
{
    std::vector<std::string> tokens;
    std::size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
    return tokens;
}