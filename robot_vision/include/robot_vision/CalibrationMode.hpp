#ifndef CALIBRATION_MODE_HPP
#define CALIBRATION_MODE_HPP

#include <Application.hpp>

/**
 * @brief the calibrationmode is used to calibrate the color ranges. The mode will open up a window with
 * sliders to adjust the color ranges for each color, a window with live frames from the camera and a
 * window that shows the last frame after feature-extraction.
 * 
 */
class CalibrationMode: public Application
{
public:
    /**
     * @brief Construct a new Calibration Mode object
     * 
     * @param aDisplayMode the displaymode to be set
     * @param aName the name of the camera window
     * @param aCalibrationWindowName 
     */
    CalibrationMode(const DisplayMode& aDisplayMode, const std::string& aName, const std::string& aCalibrationWindowName);
    
    /**
     * @brief Destroy the Calibration Mode object
     */
    virtual ~CalibrationMode();

    /**
     * @brief function that runs the application: overrides the pure virtual run -function in Application.hpp
     */
    void run();

    /**
     * @brief static function used as callback by the trackbar sliders from openCV
     * 
     * @param v
     * @param ptr this instance of the object instance
     */
    static void calibrate(int v, void *ptr)
    {
        v += 1; // To prevent wExtra complains

        CalibrationMode* calibration = static_cast<CalibrationMode*>(ptr);
        calibration->updateSetRange();
    }

    /**
     * @brief function that updates the ranges set on the slider to the map
     */
    void updateSetRange();

private:
    /** The name of the window that displays the color calibration */
    std::string calibrationWindowName;

    /** The name of the window that displays the sliders (which are used to calibrate) */
    std::string sliderWindow = "CALIBRATE";

    /** Map that is updated on slider update */
    std::map<supportedColor, colorRange> mappedColorRangesMap;

    /** Boolean that is true if the application should be active and false if inactive */
    int saved = 0;

    /** Integers that contain the current value of a slider */
    int calMinHue, calMinSaturation, calMinValue, calMaxHue, calMaxSaturation, calMaxValue;

    /** Contains the previous selected color */
    int previousSelected = -1;

    /** Contains the current selected color */
    int selectedColor = 0;

    /**
    * @brief Set the Color Range object
    * 
    * @param color the color to be set
    */
    void setColorRange(const supportedColor& color);
};

#endif //CALIBRATION_MODE_HPP