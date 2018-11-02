#include <InteractiveApplication.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

InteractiveApplication::InteractiveApplication(const DisplayMode& aDisplayMode, const std::string& aName):
    Application(aDisplayMode, aName)
{
    // Capturing of new specifications will be done on a seperate thread
    newSpecificationThread = std::thread(&InteractiveApplication::readNewSpecifications, this);
}

InteractiveApplication::~InteractiveApplication()
{
    newSpecificationThread.join();
}

template<typename T>
void pop_front(std::vector<T>& vec)
{
    assert(!vec.empty());
    vec.front() = std::move(vec.back());
    vec.pop_back();
}

void InteractiveApplication::run()
{
    std::cerr << "** Interactive app initiated **" << std::endl;
    Specification lastSpecification;

    bool calibrated = false;
    bool solvePnpSet = false;
    bool pixelPerMeterSet = false;

    // Webcam initialization time
    for (std::size_t i = 0; i < 25; ++i)
    {
        getNewFrame();
        cv::imshow(windowName, frame);
        cv::waitKey(frameRefreshRate);
    }

    // Keep the application running until the user specifies it should shutdown (or a fatal exception is thrown)
    while (active)
    {
        try {
          
            getNewFrame();

            if (!calibrated)
            {
                calibrated = filterExecutor.setCalibrationSquares(frame);
            }
            getNewFrame();
            if (calibrated && !solvePnpSet)
            {
                solvePnpSet = filterExecutor.calibrateCameraPos();
                
            }
            getNewFrame();
            if (calibrated && solvePnpSet && !pixelPerMeterSet)
            {
                pixelPerMeterSet = filterExecutor.setPixelPerMeter(frame);
            }
            
            m.lock();
            if (specifications.empty())
            {
                m.unlock();

                cv::imshow(windowName, frame);
                cv::waitKey(frameRefreshRate);
            }
            else if (!specifications.empty())
            {
                if (specifications.size() >= 2)
                {
                    // Delete the oldest specification if a new specification has been entered
                    pop_front(specifications);
                }
                bool found = false;

                lastSpecification = *specifications.begin();
                m.unlock();
                if (lastSpecification.exit)
                {
                    break;
                }

                for (std::size_t i = 0; i < 5; i++)
                {
                    std::vector<FilterDetails> result;
                    std::clock_t clockTicks;
                    filteredFrame = filterExecutor.filter(frame, lastSpecification, found, result, clockTicks);
                    if (found)
                    {
                        try
                        {
                            auto targetPos = filterExecutor.getTargetPos(frame);
                            filterExecutor.showResult(filteredFrame, lastSpecification, clockTicks, targetPos, result);
                        }
                        catch (const std::runtime_error& e)
                        {
                            std::cerr << e.what() << std::endl;
                        }
                        
                        break;
                    }
                    getNewFrame();
                }
                if (!found)
                {
                    std::cerr << "Object not found within 5 attempts. Try again, or move the object." << std::endl;
                }
                cv::imshow(windowName, filteredFrame);
                cv::waitKey(frameRefreshRate);

                if (specifications.size() == 1)
                {
                    pop_front(specifications);
                }
            }
        }
        catch (const std::runtime_error& error)
        {
            std::cerr << "Runtime error: " << error.what() << std::endl;    
            break;
        }
    }
}

void InteractiveApplication::readNewSpecifications()
{
    std::string input;
    Specification newSpecification;

    while (active)
    {
        getline(std::cin, input);

        if (input == "exit" || input == "Exit")
        {
            newSpecification.exit = true;

            m.lock();

            specifications.push_back(newSpecification);
            active = false;

            m.unlock();

            break;
        }
        try
        {
            m.lock();

            parser.tryParseSpecificationFromStr(input, newSpecification);
            specifications.push_back(newSpecification);

            m.unlock();
        }
        catch (const std::invalid_argument& reason)
        {
            m.unlock();
            std::cout << "WARNING: " << reason.what() << std::endl;
        }
    }
}