#ifndef FILTER_EXECUTOR_HPP_
#define FILTER_EXECUTOR_HPP_

#include <opencv2/opencv.hpp>

#include <Detection.hpp>
#include <Extraction.hpp>
#include <Parser.hpp>

/**
 * @brief the filter executor combines the provided functionality by the detection and extraction classes.
 * It will make sure every image is filtered properly and only the shape requested in a specification is marked.
 * When finished it will make sure the result of a specification is showed to the user.
 * 
 */
class FilterExecutor
{
public:
    /**
     * @brief Construct a new Filter Executor object, also the real world coordinates of the calibration squares
     * will be set
     * @param aDisplayMode the displaymode
     */
    explicit FilterExecutor(const DisplayMode& aDisplayMode);

    /**
     * @brief Destroy the Filter Executor object
     */
    virtual ~FilterExecutor();

    /**
     * @brief function that calls the necessary extraction -and detection function(s) to run a filter
     * 
     * @param original the original frame where the filter has to be executed upon
     * @param specification the specification contains information about the filter request
     * @return cv::Mat the original image where the filter has run upon
     */
    cv::Mat filter(const cv::Mat& original, const Specification& specification, bool& found, std::vector<FilterDetails>& result, std::clock_t& clockTicks);

    /**
     * @brief function that verifies if a set of contours is a specific function
     * 
     * @param shape the shape to be detected
     * @param contours the detected set of contours
     * @param index the position in the contours vector that indicates the current contour to be handled
     * @param filterDetails instance of struct FilterDetails that contain the result of the filter process
     * @return true if the contours matched the shape to be detected
     * @return false if the contours didn't match the shape to be detected
     */
    bool isObject(const supportedShape& shape, const std::vector<std::vector<cv::Point>>& contours, 
                  int index, FilterDetails& filterDetails) const;

    /**
     * @brief Set the Calibration Squares object
     * 
     * @param original the frame where the calibration squares are visible
     * @return true if the calibration squares were successfully found
     * @return false if the calibration squares were not successfully found
     */
    bool setCalibrationSquares(cv::Mat& original);

    /**
     * @brief function that will calibrate the camera by using openCV's solvepnp
     * https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html?highlight=solvepnp#cv2.solvePnP
     * 
     * @return true 
     * @return false 
     */
    bool calibrateCameraPos();

    /** Extration instance extracs an image based on a color specified in a specification */
    Extractor extractor;

    /** Detection instance detects certain objects after image extraction */
    Detector detector;

    /** Function that will get the center coordinate of the white circle where the object should be put down */
    const geometry_msgs::Point getTargetPos(const cv::Mat& frame);

    /**
     * @brief function that displays the result according to the requested display-option
     * 
     * @param original the original image the result has to be displayed on
     * @param displayMode the way the results are displayed
     * @param originalSpec the original specification
     * @param clockTicks the nr of clock ticks it took if no specifications were found
     */
    void showResult(cv::Mat& original, const Specification& originalSpec,
            const std::clock_t& clockTicks, const geometry_msgs::Point targetPos, const std::vector<FilterDetails>& result) const;

    /** Function that will again set the calibration squares (after solvepnp) and than calculate the pixel per meter */
    bool setPixelPerMeter(const cv::Mat& frame);

private:

    /** The way the filter results are displayed */
    DisplayMode displayMode;

    cv::Point2f topLeftSquareCenter, topRightSquareCenter, bottomLeftSquareCenter, bottomRightSquareCenter;
    double pixelPerMeter;
    cv::Mat cameraMatrix;

    cv::Mat distCoeffs;

    std::vector<cv::Point2f> corners;
    std::vector<cv::Point3f> objectPoints;

    cv::Point3f realTopLeft;
    cv::Point3f realTopRight;
    cv::Point3f realBottomLeft;
    cv::Point3f realBottomRight;

    cv::Mat rvec, tvec;

    /**
     * @brief Get the Real Coordinate object seen from the top left square center position
     * 
     * @param coordinate the coordinate in pixels to be converted
     * @return const geometry_msgs::Point the converted coordinate seen from the topleft square in meters
     */
    const geometry_msgs::Point getRealCoordinate(const geometry_msgs::Point& coordinate) const;

    void computeC2MC1(const cv::Mat &R1, const cv::Mat &tvec1, const cv::Mat &R2, const cv::Mat &tvec2, cv::Mat &R_1to2, cv::Mat &tvec_1to2);
};

#endif //FILTER_EXECUTOR