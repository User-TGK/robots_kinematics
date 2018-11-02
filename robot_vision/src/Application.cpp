#include <Application.hpp>

Application::Application(const DisplayMode& aDisplayMode, const std::string& aName):
    filterExecutor(aDisplayMode), windowName(aName), frameRefreshRate(20), active(true)
{
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);

    cap = cv::VideoCapture(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
}

Application::~Application()
{
    frame.release();
    filteredFrame.release();
    
    cv::destroyAllWindows();
}

void Application::getNewFrame()
{
    if (cap.isOpened())
    {
        cap.read(frame);

        if (frame.empty())
        {
            throw std::runtime_error("CAMERA NOT ATTATCHED OR COULD NOT CAPTURE A NEW FRAME.");
        }
    }
}