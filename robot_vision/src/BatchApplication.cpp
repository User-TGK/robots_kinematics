#include <BatchApplication.hpp>

BatchApplication::BatchApplication(const DisplayMode& aDisplayMode, const std::string& batchConfigPath, const std::string& aName):
    Application(aDisplayMode, aName), framesPerSpec(15), currentRepeat(0)
{
    parser.tryParseBatchConfig(batchConfigPath, specifications);
}

BatchApplication::~BatchApplication()
{
}

void BatchApplication::run()
{
    std::cout << "** Batch app initiated **" << std::endl;
    std::cout << "** " << specifications.size() << " specificatie(s) gevonden die worden gerund door het algoritme. **" << std::endl;
    
    // Webcam initialization time
    for (std::size_t i = currentRepeat; i < 25; ++i)
    {
        getNewFrame();
        cv::imshow(windowName, frame);
        cv::waitKey(frameRefreshRate);
    }
    for (const auto& spec : specifications)
    {
        currentRepeat = 0;
        for (currentRepeat = 0; currentRepeat < framesPerSpec; ++currentRepeat)
        {
            try 
            {
                bool found = false;
                std::vector<FilterDetails> result;
                std::clock_t clockTicks;
                getNewFrame();
                filteredFrame = filterExecutor.filter(frame, spec, found, result, clockTicks);

                cv::imshow(windowName, filteredFrame);
                cv::waitKey(frameRefreshRate);
            }
            catch (const std::runtime_error& error)
            {
                std::cout << "Runtime error: " << error.what() << std::endl;
                break;
            }
        }
    }
}