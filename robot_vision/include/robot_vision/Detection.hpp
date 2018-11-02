#ifndef DETECTION_HPP_
#define DETECTION_HPP_

#include <SpecificationConfig.hpp>

#include <geometry_msgs/Point.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <ctime>
#include <map>
#include <optional>

/**
 * @brief Struct that contains the details of the filter result for a specific specification
 */
struct FilterDetails
{
    /**
     * @brief Set the Detection Result object
     * 
     * @param aFound true of the specification was found in the shape, false if not
     * @param aSurface the surface of the object
     * @param aXPosCenter the x coordinate of the center of the object
     * @param aYPosCenter the y coordinate of the center of the object
     */
    void setDetectionResult(bool aFound, int aSurface, int aXPosCenter, int aYPosCenter);
    /** True if the specification was found, false if not */
    bool found;
    /** The surface of the object (only set if the object was found) */
    int surface;
    /** The x position of the center of the object (only set if the object was found) */
    int xPosCenter;
    /** The y position of the center of the object (only set if the object was found) */
    int yPosCenter;
    /** The amount of clock ticks it took before the shape was detected */    
    std::clock_t clockTicks;
    /** The radius of the shape (if it's (half) a circle) */
    int radius;
    /** The vertices of the shape (or radius if its a circle) */
    std::vector<geometry_msgs::Point> shapeVertices;
};

/**
 * @brief The detector class handles all functionality concerning the feature-detecting process
 * It will to to recognize pre-defined shapes on a binary image (a captured image after feature-extracting)
 * 
 */
class Detector
{
public:
    /**
     * @brief Construct a new Detector object
     */
    Detector();

    /**
     * @brief Destroy the Detector object
     */
    virtual ~Detector();

    /**
     * @brief implementation of the cosine law (see inline comments in .cpp file)
     * 
     * @param a first width
     * @param b second width
     * @param c third width
     * @return double the angle
     */
    double cosineLaw(const double a, const double b, const double c) const;

    /**
     * @brief function that checks if a shape is a square based on surface, width and height
     * 
     * @param contours the contours detected
     * @param index the index of the contour to be detected
     * @param filterDetails the result (details) of the detection for a specific set of contours
     * @return true if the contours matches with the definition of a square
     * @return false if the contours don't match with the definition of a square
     */
    bool checkIfSquare(const std::vector<std::vector<cv::Point>>& contours, int index,
    FilterDetails& filterDetails) const;

    /**
     * @brief function that checks if a shape is a triangle based on surface and total of the three angles
     * 
     * @param contours the contours detected
     * @param index the index of the contour to be detected
     * @param filterDetails the result (details) of the detection for a specific set of contours
     * @return true if the contours matches with the definition of a square
     * @return false if the contours don't match with the definition of a triangle
     */
    bool checkIfTriangle(const std::vector<std::vector<cv::Point>>& contours, int index,
    FilterDetails& filterDetails) const;

    /**
     * @brief function that checks if a shape is a rectangle (a square is also a rectangle)
     * 
     * @param contours the contours detected
     * @param index the index of the contour to be detected
     * @param filterDetails the result (details) of the detection for a specific set of contours
     * @return true if the contours matches with the definition of a square
     * @return false if the contours don't match with the definition of a rectangle
     */
    bool checkIfRect(const std::vector<std::vector<cv::Point>>& contours, int index,
    FilterDetails& filterDetails) const;

    /**
     * @brief function that checks if a shape is a circle
     * 
     * @param contours the contours detected
     * @param index the index of the contour to be detected
     * @param filterDetails the result (details) of the detection for a specific set of contours
     * @return true if the contours matches with the definition of a square
     * @return false if the contours don't match with the definition of a circle
     */
    bool checkIfCircle(const std::vector<std::vector<cv::Point>>& contours, int index,
    FilterDetails& filterDetails) const;

    /**
     * @brief function that checks if a shape is half a circle
     * 
     * @param contours the contours detected
     * @param index the index of the contour to be detected
     * @param filterDetails the result (details) of the detection for a specific set of contours
     * @return true if the contours matches with the definition of a square
     * @return false if the contours don't match with the definition of half a circle
     */
    bool checkIfHalfCircle(const std::vector<std::vector<cv::Point>>& contours, int index,
    FilterDetails& filterDetails) const;

    /**
     * @brief function that detects a shape if it matches (enough) with a reference shape
     * 
     * @param contour the contour to be detected
     * @return std::optional<supportedShape> nullopt if the shape didn't match enough with any reference shape,
     * else the shape it matches the most with
     */
    std::optional<supportedShape> detectShape(const std::vector<cv::Point>& contour) const;
    
private:
    /** The minimum nr of pixel a shape should contain to be treated as a valid object */
    unsigned short minimumObjectSize = 550;

    /** 0.35 matches a 0.65 resemblances beteween a reference shape and detected shape to be recognized as a supported shape */
    double minResultFactor = 0.35;

    /** Map that contains the set of contours for the reference shapes */
    std::unordered_multimap<supportedShape, std::vector<cv::Point>> referenceContours;

    /**
     * @brief function that loads the reference contours from a set of images to a map
     * 
     * @return true if the images were successfully converted into a set of contours
     * @return false if the images could not be opened or could not be converted into a set of contours
     */
    bool loadReferenceContours();

    /**
     * @brief function that sorts a vector of points (THE SIZE OF THE VECTOR MUST BE 4)
     * from top left to bottom left
     * 
     * @param vertices 
     */
    void sortVertices(std::vector<cv::Point>& vertices) const;
};

#endif //DETECTION_HPP_