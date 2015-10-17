#ifndef CUSTOM_SCENE_H
#define CUSTOM_SCENE_H

#include "simulation/environment.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/rope.h"

#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"

#include "gripper_kinematic_object.h"

class CustomScene : public Scene
{
    public:
        enum DeformableType
        {
            ROPE,
            CLOTH
        };

        enum TaskType
        {
            COVERAGE,
            COLAB_FOLDING
        };

        CustomScene(  DeformableType deformable_type, TaskType task_type  );
        void run(  bool syncTime = false  );

    private:
        // Construction functions
        void makeRopeWorld();
        void makeClothWorld();

        // Plotting functions
        void initializePloting();
        void drawAxes();

        DeformableType deformable_type_;
        TaskType task_type_;

        // objects shared by multiple deformable types
        std::map< std::string, PlotAxes::Ptr > gripper_axes_;
        std::map< std::string, GripperKinematicObject::Ptr > grippers_;
        std::vector< GripperKinematicObject::Ptr > auto_grippers_;

        // rope world objects
        boost::shared_ptr<CapsuleRope> rope_;
        CylinderStaticObject::Ptr cylinder_;
        BoxObject::Ptr table_;

        // cloth world objects

        // coverage task objects
        std::vector<btVector3> cover_points_;
};

#endif
