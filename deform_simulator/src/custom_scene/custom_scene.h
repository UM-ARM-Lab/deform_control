#ifndef CUSTOM_SCENE_H
#define CUSTOM_SCENE_H

#include "simulation/environment.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/rope.h"

#include "simulation/config_bullet.h"
#include "simulation/config_scene.h"
#include "simulation/config_viewer.h"

#include "gripper_kinematic_object.h"

#include <ros/ros.h>

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

        CustomScene( DeformableType deformable_type, TaskType task_type, ros::NodeHandle& nh );
        void run( bool syncTime = false );

    private:
        ////////////////////////////////////////////////////////////////////////
        // Construction functions
        ////////////////////////////////////////////////////////////////////////

        void makeTable( const float half_side_length, const bool set_cover_points = false );
        void makeCylinder( const bool set_cover_points = false );
        void makeRope();
        void makeCloth();

        void makeRopeWorld();
        void makeClothWorld();
        void findClothCornerNodes();

        ////////////////////////////////////////////////////////////////////////
        // Plotting functions
        ////////////////////////////////////////////////////////////////////////

        void initializePloting();
        void drawAxes();

        PlotPoints::Ptr plot_points_;
        PlotLines::Ptr plot_lines_;

        ////////////////////////////////////////////////////////////////////////
        // Task Variables TODO to be moved into a CustomSceneConfig file
        ////////////////////////////////////////////////////////////////////////

        DeformableType deformable_type_;
        TaskType task_type_;

        ////////////////////////////////////////////////////////////////////////
        // Grippers
        ////////////////////////////////////////////////////////////////////////

        std::map< std::string, PlotAxes::Ptr > gripper_axes_;
        std::map< std::string, GripperKinematicObject::Ptr > grippers_;
        std::vector< GripperKinematicObject::Ptr > auto_grippers_;

        ////////////////////////////////////////////////////////////////////////
        // Shared world objects
        ////////////////////////////////////////////////////////////////////////

        static constexpr float TABLE_X = 0; // METERS
        static constexpr float TABLE_Y = 0; // METERS
        static constexpr float TABLE_Z = 0.7; // METERS
        static constexpr float TABLE_THICKNESS = 0.05; // METERS
        BoxObject::Ptr table_;

        ////////////////////////////////////////////////////////////////////////
        // Rope world objects
        ////////////////////////////////////////////////////////////////////////

        static constexpr float ROPE_SEGMENT_LENGTH = 0.025; // METERS
        static constexpr float ROPE_RADIUS = 0.01; // METERS
        static constexpr int ROPE_NUM_LINKS = 50;
        static constexpr float ROPE_GRIPPER_APPERTURE = 0.03; // METERS
        static constexpr float ROPE_TABLE_HALF_SIDE_LENGTH = 1.5; // METERS
        static constexpr float ROPE_CYLINDER_RADIUS = 0.15; // METERS
        static constexpr float ROPE_CYLINDER_HEIGHT = 0.3; // METERS
        CylinderStaticObject::Ptr cylinder_;
        boost::shared_ptr<CapsuleRope> rope_;

        ////////////////////////////////////////////////////////////////////////
        // Cloth world objects
        ////////////////////////////////////////////////////////////////////////

        static constexpr float CLOTH_HALF_SIDE_LENGTH = 0.25; // METERS
        static constexpr float CLOTH_X = TABLE_X + CLOTH_HALF_SIDE_LENGTH; // METERS
        static constexpr float CLOTH_Y = TABLE_Y; // METERS
        static constexpr float CLOTH_Z = TABLE_Z + 0.1; // METERS
        static constexpr int CLOTH_DIVS = 45;
        static constexpr float CLOTH_GRIPPER_APPERTURE = 0.1; // METERS
        static constexpr float CLOTH_TABLE_HALF_SIDE_LENGTH = 0.2; // METERS
        BulletSoftObject::Ptr cloth_;
        std::vector< int > cloth_corner_node_indices_;

        ////////////////////////////////////////////////////////////////////////
        // Coverage task objects
        ////////////////////////////////////////////////////////////////////////

        std::vector<btVector3> cover_points_;

        ////////////////////////////////////////////////////////////////////////
        // ROS Objects and Helpers
        ////////////////////////////////////////////////////////////////////////

        ros::NodeHandle nh_;
};

#endif
