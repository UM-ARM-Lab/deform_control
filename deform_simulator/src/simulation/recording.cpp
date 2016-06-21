#include "simulation/recording.h"
#include <boost/filesystem.hpp>
#include "utils/my_exceptions.h"
#include "utils/my_assert.h"

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


ScreenRecorder::ScreenRecorder(osgViewer::Viewer& viewer)
    : m_viewer(viewer)
{
    askToResetDir("screenshots");
    m_captureOperation = std::make_shared<osgViewer::ScreenCaptureHandler::WriteToFile>("screenshots/img", "jpg", osgViewer::ScreenCaptureHandler::WriteToFile::SEQUENTIAL_NUMBER);
    m_captureHandler = std::make_shared<osgViewer::ScreenCaptureHandler>(m_captureOperation.get());
    m_viewer.addEventHandler(m_captureHandler.get());
}

void ScreenRecorder::snapshot()
{
    cout << "taking snapshot!" << endl;
    m_captureHandler->captureNextFrame(m_viewer);
}
