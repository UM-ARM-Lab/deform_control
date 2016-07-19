#include "simulation/recording.h"
#include <boost/filesystem.hpp>
#include "utils/my_exceptions.h"
#include "utils/my_assert.h"
#include <smmap_experiment_params/ros_params.hpp>

using namespace std;

namespace fs = boost::filesystem;

bool yesOrNo(const string& message)
{
    while (true)
    {
        cout << message << " (y/n): ";
        char yn;
        cin >> yn;
        if (yn == 'y')
        {
            return true;
        }
        else if (yn == 'n')
        {
            return false;
        }
    }
}

void askToResetDir(fs::path p)
{
    if (fs::exists(p))
    {
        bool consent = yesOrNo(p.string() + "already exists. Delete it?");
        if (consent)
        {
            cout << "deleting " << p.string() << endl;
            fs::remove_all(p);
        }
        else throw IOError();
    }
    ENSURE(fs::create_directory(p));
}

void resetDir(fs::path p)
{
    if (fs::exists(p))
    {
        fs::remove_all(p);
    }
    ENSURE(fs::create_directory(p));
}

ScreenRecorder::ScreenRecorder(osgViewer::Viewer& viewer, const bool screenshots_enabled, ros::NodeHandle& nh)
    : m_screenshotsEnabled(screenshots_enabled)
    , m_viewer(viewer)
{
    if (m_screenshotsEnabled)
    {
        const std::string screenshot_dir = smmap::GetScreenshotFolder(nh);
        std::cout << "Clearing screenshot folder " << screenshot_dir << std::endl;
        resetDir(screenshot_dir.c_str());
        m_captureOperation = std::make_shared<osgViewer::ScreenCaptureHandler::WriteToFile>(screenshot_dir + "img", "jpg", osgViewer::ScreenCaptureHandler::WriteToFile::SEQUENTIAL_NUMBER);
        m_captureHandler = std::make_shared<osgViewer::ScreenCaptureHandler>(m_captureOperation.get());
        m_viewer.addEventHandler(m_captureHandler.get());
    }
    else
    {
        std::cout << "Screenshots disabled\n";
    }
}

void ScreenRecorder::snapshot()
{
    if (m_screenshotsEnabled)
    {
        std::cout << "Taking screenshot\n";
        m_captureHandler->captureNextFrame(m_viewer);
    }
}
