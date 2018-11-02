#include <InteractiveApplication.hpp>
#include <BatchApplication.hpp>
#include <CalibrationMode.hpp>

#include <memory>
#include <stdexcept>
#include <ros/ros.h>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle n;
    std::shared_ptr<Application> app;

    if (argc == 1)
    {
        //Interactive application mode initiated
        app = std::make_shared<InteractiveApplication>(DisplayMode::toRobot, "InteractiveApp");
    }
    else if (argc == 2) 
    {
        try
        {
            //Batch application mode initiated
            app = std::make_shared<BatchApplication>(DisplayMode::toConsole, argv[1], "BatchApp");
        }
        catch(const std::invalid_argument& reason)
        {
            std::cout << "The application failed to start because of the following reason: " << reason.what() << std::endl;
            return 1;
        }
    }
    else if (argc == 3 && std::string(argv[1]) == std::string("calibrate"))
    {
        try
        {
            //Calibration mode initiated
            app = std::make_shared<CalibrationMode>(DisplayMode::toConsole, "CalibrateApp", "CalibrateWindow");
        }
        catch(const std::invalid_argument& reason)
        {
            std::cout << "The application failed to start because of the following reason: " << reason.what() << std::endl;
            return -1;
        }
    }
    else
    {
        std::cout << "*** APPLICATION START-UP INSTRUCTIONS ***" << std::endl;
        std::cout << "1) Batch mode: '$ ./RobotAlgorithm <path to batchfile>'" << std::endl;
        std::cout << "Example: ' $ ./RobotAlgorithm ../batch/batchfile.ini '" << std::endl;
        std::cout << "2) Interactive mode: '$ ./RobotAlgorithm'" << std::endl;
        std::cout << "3) Calibration mode: '$ ./RobotAlgorithm calibrate mode'" << std::endl;
        std::cout << "Mode to calibrate color ranges (HSV)" << std::endl;

        return -1;
    }
    try 
    { 
        app->run(); 
        ros::spinOnce();
    }
    catch (const std::runtime_error& error)
    {
        std::cout << error.what() << std::endl;
        return -1;
    }
    
    std::cout << "** Application finished because of one of the following reasons: **"  << std::endl
                  << "- All specifications were executed (batch)"                       << std::endl
                  << "- You specified the 'exit' command (interactive)"                 << std::endl 
                  << "- You saved the calibrated color ranges (calibration)"            << std::endl;
    return 0;
}