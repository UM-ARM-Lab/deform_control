#pragma once
#include "utils/config.h"
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <memory>
#include <ros/ros.h>

class ScreenRecorder
{
    public:
        ScreenRecorder(osgViewer::Viewer& viewer, const bool screenshots_enabled, ros::NodeHandle& nh);

        void snapshot(); //call this BEFORE scene's step() method

    private:
        const bool m_screenshotsEnabled;
        osgViewer::Viewer& m_viewer;
        std::shared_ptr<osgViewer::ScreenCaptureHandler::CaptureOperation> m_captureOperation;
        std::shared_ptr<osgViewer::ScreenCaptureHandler> m_captureHandler;
};
