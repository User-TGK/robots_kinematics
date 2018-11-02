#ifndef BATCHAPPLICATION_HPP_
#define BATCHAPPLICATION_HPP_

#include <Application.hpp>

/**
 * @brief Instance of an application. The batch application starts the batch mode in the run() function
 * that stays active until all specifications are handled. The application will display the result for a specification
 * a couple of times (itering as much as specified in framesPerSpec) before moving to the next specification.
 * 
 */
class BatchApplication: public Application
{
public:
    /**
     * @brief constructor of a batchapplication: here the specifications stored in a batch file
     * will be parsed into a list of specifications
     * 
     * @param aDisplayMode the displaymode to be set
     * @param batchConfigPath the path to the batchfile used in the batchmode
     * @param aName the name of the window
     */
    BatchApplication(const DisplayMode& aDisplayMode, const std::string& batchConfigPath, const std::string& aName);

    /**
     * @brief destructor of a batchapplication (will be called after the execution of the batch is finished)
     */
    virtual ~BatchApplication();

    /**
     * @brief function that runs the application: overrides the pure virtual run -function in Application.hpp
     */
    void run();

private:
    /** Number that describes the number of times a specification will be shown before moving to the next */
    unsigned short framesPerSpec;

    /** Number that contains the number of times a frame has been refreshed */
    unsigned short currentRepeat;
};

#endif //BATCHAPPLICATION_HPP_