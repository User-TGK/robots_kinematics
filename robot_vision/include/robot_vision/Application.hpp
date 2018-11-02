#ifndef APPLICATION_HPP_
#define APPLICATION_HPP_

#include <FilterExecutor.hpp>

#include <vector>
#include <iostream>
#include <atomic>

/**
 * @brief the super class for an application instance. The application can be either the interactive mode,
 * the batchmode and the calibration mode. For each deriving class the pure virtual run() function will be 
 * implemented and run in the main.
 * 
 */ 
class Application
{
public:
    /**
     * @brief default Application constructor
     * @param aDisplayMode the displaymode to be set
     * @param aName the windowname
     */
    Application(const DisplayMode& aDisplayMode, const std::string& aName);

    /**
     * @brief destructor
     */
    virtual ~Application();

    /**
     * @brief function that runs a specific instance of an application
     * pure virtual: the function gets implemented by its deriving classes
     */
    virtual void run() = 0;

    /**
     * @brief Get the New Frame object from the videocapture instance
     */
    void getNewFrame();

protected:
    /** Each application has a parser that converts a batchfile or string to a specification */
    Parser parser;

    /** An application contains a list with specifications to be executed */
    std::vector<Specification> specifications;

    /** The image used is based on a videocapture */
    cv::VideoCapture cap;

    /** The last frame captured from the camera */
    cv::Mat frame;

    /** The filtered frame to be displayed if a specification is given */
    cv::Mat filteredFrame;

    /** The filter executor instance that handles filtering based on specifications */
    FilterExecutor filterExecutor;

    /** The name of the cv::namedWindow that displays the captured frames */
    std::string windowName;

    /** The time untill the frame refreshes (in milliseconds) */
    int frameRefreshRate;

    /** Boolean that is true if the application should be active and false if inactive */
    std::atomic<bool> active;
};

#endif //APPLICATION_HPP_