#ifndef CUSTOM_KEY_HANDLER_H
#define CUSTOM_KEY_HANDLER_H

#include "colab_cloth.h"
#include "colab_cloth_custom_scene.h"

class CustomKeyHandler : public osgGA::GUIEventHandler
{
    public:
        CustomKeyHandler(ColabClothCustomScene &scene_)
            : scene(scene_)
        {}

        bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);

    private:
        ColabClothCustomScene &scene;
};

#endif
