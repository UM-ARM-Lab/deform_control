#pragma once
#include "utils/config.h"
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <memory>

class ScreenRecorder
{
    public:
        ScreenRecorder(osgViewer::Viewer& viewer);
        ~ScreenRecorder();

        void snapshot(); //call this BEFORE scene's step() method

    private:
        osgViewer::Viewer& m_viewer;
        std::shared_ptr<osgViewer::ScreenCaptureHandler::CaptureOperation> m_captureOperation;
        std::shared_ptr<osgViewer::ScreenCaptureHandler> m_captureHandler;
};
