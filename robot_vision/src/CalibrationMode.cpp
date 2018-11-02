#include <CalibrationMode.hpp>

#include <opencv2/photo.hpp>

CalibrationMode::CalibrationMode(const DisplayMode& aDisplayMode, const std::string& aName, const std::string& aCalibrationWindowName):
    Application(aDisplayMode, aName), 
    calibrationWindowName(aCalibrationWindowName), 
    mappedColorRangesMap(filterExecutor.extractor.mappedColorRangesMapFromFile())
{
    calMinHue = 0;
    calMinSaturation = 0;
    calMinValue = 0;
    calMaxHue = 0;
    calMaxSaturation = 0;
    calMaxValue = 0;

    cv::namedWindow(aCalibrationWindowName, cv::WINDOW_NORMAL);
    cv::namedWindow(sliderWindow, cv::WINDOW_NORMAL);

    cv::createTrackbar("Kleur", sliderWindow, &this->selectedColor, 5, calibrate, this);

    cv::createTrackbar("minHue", sliderWindow, &this->calMinHue, 178, calibrate, this);
    cv::createTrackbar("maxHue", sliderWindow, &this->calMaxHue, 179, calibrate, this);
    cv::createTrackbar("minSaturation", sliderWindow, &this->calMinSaturation, 254, calibrate, this);
    cv::createTrackbar("maxSaturation", sliderWindow, &this->calMaxSaturation, 255, calibrate, this);
    cv::createTrackbar("minValue", sliderWindow, &this->calMinValue, 254, calibrate, this);
    cv::createTrackbar("maxValue", sliderWindow, &this->calMaxValue, 255, calibrate, this);

    cv::createTrackbar("SAVE", sliderWindow, &this->saved, 1);
}

CalibrationMode::~CalibrationMode()
{
}

void CalibrationMode::updateSetRange()
{
    switch (selectedColor)
    {
        case 0:
        {
            setColorRange(supportedColor::black);
            break;
        }
        case 1:
        {
            setColorRange(supportedColor::blue);
            break;
        }
        case 2:
        {
            setColorRange(supportedColor::green);
            break;
        }
        case 3:
        {
            setColorRange(supportedColor::red);
            break;
        }
        case 4:
        {
            setColorRange(supportedColor::white);
            break;
        }
        case 5:
        {
            setColorRange(supportedColor::yellow);
            break;
        }
    }
}

void CalibrationMode::setColorRange(const supportedColor& color)
{
    auto searchResult = mappedColorRangesMap.find(color);
    auto range = searchResult->second;

    if (previousSelected != selectedColor)
    {
        calMinHue = range.minHue;
        calMaxHue = range.maxHue;
        calMinSaturation = range.minSaturation;
        calMaxSaturation = range.maxSaturation;
        calMinValue = range.minValue;
        calMaxValue = range.maxValue;

        cv::setTrackbarPos("minHue", sliderWindow, range.minHue);
        cv::setTrackbarPos("maxHue", sliderWindow, range.maxHue);
        cv::setTrackbarPos("minSaturation", sliderWindow, range.minSaturation);
        cv::setTrackbarPos("maxSaturation", sliderWindow, range.maxSaturation);
        cv::setTrackbarPos("minValue", sliderWindow, range.minValue);
        cv::setTrackbarPos("maxValue", sliderWindow, range.maxValue);
        previousSelected = selectedColor;
    }
    else
    {
        searchResult->second.minHue = calMinHue;
        searchResult->second.maxHue = calMaxHue;
        searchResult->second.minSaturation = calMinSaturation;
        searchResult->second.maxSaturation = calMaxSaturation;
        searchResult->second.minValue = calMinValue;
        searchResult->second.maxValue = calMaxValue;
    }
}

void CalibrationMode::run()
{
    cv::Mat originalCopy, hsvFiltered, bilateral;
    std::cout << "Selecteer een kleur en pas de threshholds aan: " << std::endl;
    std::cout << "0 zwart | 1 blauw | 2 groen | 3 rood | 4 wit | 5 geel " << std::endl;
    std::cout << "Bevestig met save slider." << std::endl;

    while (saved == 0)
    {
        try 
        {
            getNewFrame();
            originalCopy = frame;
            
            cv::fastNlMeansDenoisingColored(frame, originalCopy, 5, 15, 5, 11);
            cv::bilateralFilter(originalCopy, bilateral, 15, 80, 80);
            cv::cvtColor(bilateral, hsvFiltered, CV_BGR2HSV);

            cv::inRange(hsvFiltered, cv::Scalar(calMinHue, calMinSaturation, calMinValue), 
                        cv::Scalar(calMaxHue, calMaxSaturation, calMaxValue), hsvFiltered);

            // Morphology filters
            unsigned int filterSize = originalCopy.cols / 250;
            cv::Mat element = cv::getStructuringElement(0, cv::Size(filterSize, filterSize));
            
            cv::morphologyEx(hsvFiltered, hsvFiltered, cv::MORPH_CLOSE, element);
            cv::morphologyEx(hsvFiltered, hsvFiltered, cv::MORPH_CLOSE, element);
            cv::morphologyEx(hsvFiltered, hsvFiltered, cv::MORPH_OPEN, element);

            cv::imshow(windowName, originalCopy);
            cv::imshow(calibrationWindowName, hsvFiltered);
            cv::waitKey(frameRefreshRate);
        }
        catch (const std::runtime_error& error)
        {
            std::cout << "Runtime error: " << error.what() << std::endl;
            return;
        }
    }
    originalCopy.release();
    hsvFiltered.release();
    bilateral.release();
    
    filterExecutor.extractor.mappedColorRangesMapToFile(mappedColorRangesMap);
}