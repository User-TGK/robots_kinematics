#ifndef INTERACTIVEAPPLICATION_HPP_
#define INTERACTIVEAPPLICATION_HPP_

#include <Application.hpp>

#include <thread>
#include <mutex>

/**
 * @brief Instance of an application. The interactive application starts the interactive mode in the run() function
 * that stays active until the user specifies the 'exit' command on the console. The application will
 * display the result for a specification until a new specification is given.
 * 
 */
class InteractiveApplication: public Application
{
public:
    /**
     * @brief constructor of the interactive application
     * 
     * @param aDisplayMode the displaymode to be set
     * @param aName
     */
    InteractiveApplication(const DisplayMode& aDisplayMode, const std::string& aName);

    /**
     * @brief destructor of the interactive application
     */
    virtual ~InteractiveApplication();

    /**
     * @brief function that runs the application: overrides the pure virtual run -function in Application.hpp
     */
    void run();

    /**
     * @brief function that reads and parses new specifications
     * @exception std::invalid_argument if an invalid specification was entered by the user
     */
    void readNewSpecifications();

private:
    /** Thread that reads console input (std::cin) for new specifications entered by the user */
    std::thread newSpecificationThread;

    /** Lock the queue and active instance when changing/reading its values */
    std::mutex m;
};

#endif //INTERACTIVEAPPLICATION_HPP_