#ifndef CUSTOM_KEY_HANDLER_H
#define CUSTOM_KEY_HANDLER_H

#include "custom_scene.h"

class CustomKeyHandler : public osgGA::GUIEventHandler
{
    public:
        CustomKeyHandler(CustomScene &scene_);

        bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);

    private:
        CustomScene &scene;
};

#endif
