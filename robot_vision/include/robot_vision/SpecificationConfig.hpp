#ifndef SPECIFICATION_CONFIG_HPP
#define SPECIFICATION_CONFIG_HPP

#include <string>
#include <unordered_map>
#include <ros/package.h>
#include <robot_controller/MoveObject.h>

/**
 * @brief Struct that contains mapped values for a supported color
 */
struct colorRange
{
    /** Minimum hue (0-179) for a specific color */
    int minHue;
    /** Maximum hue (0-179) for a specific color */
    int maxHue;
    /** Minimum saturation (0-255) for a specific color */
    int minSaturation;
    /** Maximum saturation (0-255) for a specific color */
    int maxSaturation;
    /** Minimum value (0-255) for a specific color */
    int minValue;
    /** Maximum value (0-255) for a specific color */
    int maxValue;
};

/** Struct that contains the options to show filter results */
enum class DisplayMode: uint8_t
{
    toConsole,
    inImage,
    toRobot
};

/** This enum contains all colors that are supported (can be detected) */
enum class supportedColor: uint8_t
{
    red,
    green,
    blue,
    yellow,
    black,
    white
};

/** This enum contains all shapes that are supported (can be detected) */
enum class supportedShape: uint8_t
{
    circle = robot_controller::MoveObject::Request::CIRCLE,
    halfCircle = robot_controller::MoveObject::Request::HALFCIRCLE,
    square = robot_controller::MoveObject::Request::SQUARE,
    rectangle = robot_controller::MoveObject::Request::RECTANGLE,
    triangle = robot_controller::MoveObject::Request::TRIANGLE
};

/** This unordered map contains the string representation for each supported color */
const std::unordered_map<std::string, supportedColor> supportedColorMap =
{
    {"rood", supportedColor::red},
    {"groen", supportedColor::green},
    {"blauw", supportedColor::blue},
    {"geel", supportedColor::yellow},
    {"zwart", supportedColor::black},
    {"wit", supportedColor::white}
};

/** This unordered map contains the string representation for each supported shape */
const std::unordered_map<std::string, supportedShape> supportedShapeMap =
{
    {"cirkel", supportedShape::circle},
    {"halve cirkel", supportedShape::halfCircle},
    {"vierkant", supportedShape::square},
    {"rechthoek", supportedShape::rectangle},
    {"driehoek", supportedShape::triangle}
};

/** This unordered map contains paths to reference shapes for each supported shape */
const std::unordered_multimap<supportedShape, std::string> shapeReferencePaths = 
{
    {supportedShape::triangle, std::string(ros::package::getPath("robot_vision") + "/reference/triangle.jpg")},
    {supportedShape::square, std::string(ros::package::getPath("robot_vision") + "/reference/square.png")},
    {supportedShape::rectangle, std::string(ros::package::getPath("robot_vision") + "/reference/rectangle.png")},
    {supportedShape::circle, std::string(ros::package::getPath("robot_vision") + "/reference/circle.png")},
    {supportedShape::halfCircle, std::string(ros::package::getPath("robot_vision") + "/reference/half_circle.jpg")},
    {supportedShape::rectangle, std::string(ros::package::getPath("robot_vision") + "/reference/sndrectangle.jpg")},
    {supportedShape::rectangle, std::string(ros::package::getPath("robot_vision") + "/reference/trdrectangle.jpg")},

};

#endif //SPECIFICATION_CONFIG_HPP