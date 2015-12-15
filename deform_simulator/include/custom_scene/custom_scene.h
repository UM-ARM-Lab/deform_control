#ifndef CUSTOM_SCENE_H
#define CUSTOM_SCENE_H

#include <boost/thread/mutex.hpp>

#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

#include "simulation/environment.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/rope.h"

#include "simulation/config_bullet.h"
#include "simulation/config_scene.h"
#include "simulation/config_viewer.h"

#include "gripper_kinematic_object.h"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <smmap/task.h>
#include <smmap_msgs/messages.h>

class CustomScene : public Scene
{
    public:
        CustomScene( ros::NodeHandle& nh, smmap::DeformableType deformable_type, smmap::TaskType task_type );

        ////////////////////////////////////////////////////////////////////////
        // Main function that makes things happen
        ////////////////////////////////////////////////////////////////////////

        void run( bool syncTime = false );

    private:
        ////////////////////////////////////////////////////////////////////////
        // Construction helper functions
        ////////////////////////////////////////////////////////////////////////

        void makeTable( const float half_side_length, const bool set_cover_points = false );
        void makeCylinder( const bool set_cover_points = false );
        void makeRope();
        void makeCloth();

        void makeRopeWorld();
        void makeClothWorld();
        void findClothCornerNodes();

        ////////////////////////////////////////////////////////////////////////
        // Main loop helper functions
        ////////////////////////////////////////////////////////////////////////

        void moveGrippers();
        void publishSimulatorFbk();

        ////////////////////////////////////////////////////////////////////////
        // Internal helper functions
        ////////////////////////////////////////////////////////////////////////

        std::vector< btVector3 > getDeformableObjectNodes();
        btPointCollector collisionHelper( const std::string& gripper_name );

        ////////////////////////////////////////////////////////////////////////
        // ROS Callbacks
        ////////////////////////////////////////////////////////////////////////

        bool cmdGripperTrajCallback(
                smmap_msgs::CmdGrippersTrajectory::Request& req,
                smmap_msgs::CmdGrippersTrajectory::Response& res );
        bool getGripperNamesCallback(
                smmap_msgs::GetGripperNames::Request& req,
                smmap_msgs::GetGripperNames::Response& res );
        bool getGripperAttachedNodeIndicesCallback(
                smmap_msgs::GetGripperAttachedNodeIndices::Request& req,
                smmap_msgs::GetGripperAttachedNodeIndices::Response& res );
        bool getGripperPoseCallback(
                smmap_msgs::GetGripperPose::Request& req,
                smmap_msgs::GetGripperPose::Response& res );
        bool getCoverPointsCallback(
                smmap_msgs::GetPointSet::Request& req,
                smmap_msgs::GetPointSet::Response& res );
        bool getMirrorLineCallback(
                smmap_msgs::GetMirrorLine::Request& req,
                smmap_msgs::GetMirrorLine::Response& res );
        bool getObjectInitialConfigurationCallback(
                smmap_msgs::GetPointSet::Request& req,
                smmap_msgs::GetPointSet::Response& res );

        void visualizationMarkerCallback( visualization_msgs::Marker marker );
        void visualizationMarkerArrayCallback( visualization_msgs::MarkerArray marker_array );

        ////////////////////////////////////////////////////////////////////
        // ROS Objects and Helpers
        ////////////////////////////////////////////////////////////////////

        // Our internal version of ros::spin()
        static void spin( double loop_rate );

        ////////////////////////////////////////////////////////////////////////
        // Pre-step Callbacks
        ////////////////////////////////////////////////////////////////////////

        void drawAxes();

        PlotPoints::Ptr plot_points_;
        PlotLines::Ptr plot_lines_;

        std::map< std::string,  PlotLines::Ptr > visualization_line_markers_;
        std::map< std::string,  PlotSpheres::Ptr > visualization_sphere_markers_;

        ////////////////////////////////////////////////////////////////////////
        // Post-step Callbacks
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // Task Variables TODO to be moved into a CustomSceneConfig file
        ////////////////////////////////////////////////////////////////////////

        smmap::DeformableType deformable_type_;
        smmap::TaskType task_type_;

        ////////////////////////////////////////////////////////////////////////
        // Grippers
        ////////////////////////////////////////////////////////////////////////

        std::map< std::string, PlotAxes::Ptr > gripper_axes_;
        std::map< std::string, GripperKinematicObject::Ptr > grippers_;
        std::vector< std::string > auto_grippers_;
        std::vector< std::string > manual_grippers_;

        ////////////////////////////////////////////////////////////////////////
        // Shared world objects
        ////////////////////////////////////////////////////////////////////////

        static constexpr float TABLE_X = 0.0f; // METERS
        static constexpr float TABLE_Y = 0.0f; // METERS
        static constexpr float TABLE_Z = 0.7f; // METERS
        static constexpr float TABLE_THICKNESS = 0.05f; // METERS
        BoxObject::Ptr table_;

        ////////////////////////////////////////////////////////////////////////
        // Rope world objects
        ////////////////////////////////////////////////////////////////////////

        static constexpr float ROPE_SEGMENT_LENGTH = 0.025f; // METERS
        static constexpr float ROPE_RADIUS = 0.01f; // METERS
        static constexpr int ROPE_NUM_LINKS = 50;
		// TODO: why did Dmitry's code use 0.5f here?
        static constexpr float ROPE_GRIPPER_APPERTURE = 0.03f; // METERS
        static constexpr float ROPE_TABLE_HALF_SIDE_LENGTH = 1.5f; // METERS
        static constexpr float ROPE_CYLINDER_RADIUS = 0.15f; // METERS
        static constexpr float ROPE_CYLINDER_HEIGHT = 0.3f; // METERS
        CylinderStaticObject::Ptr cylinder_;
        boost::shared_ptr< CapsuleRope > rope_;

        ////////////////////////////////////////////////////////////////////////
        // Cloth world objects
        ////////////////////////////////////////////////////////////////////////

        static constexpr float CLOTH_HALF_SIDE_LENGTH = 0.25f; // METERS
        static constexpr float CLOTH_X = TABLE_X + CLOTH_HALF_SIDE_LENGTH; // METERS
        static constexpr float CLOTH_Y = TABLE_Y; // METERS
        static constexpr float CLOTH_Z = TABLE_Z + 0.1f; // METERS
        static constexpr int CLOTH_DIVS = 45;
        static constexpr float CLOTH_GRIPPER_APPERTURE = 0.1f; // METERS
        static constexpr float CLOTH_TABLE_HALF_SIDE_LENGTH = 0.2f; // METERS
        BulletSoftObject::Ptr cloth_;
        std::vector< int > cloth_corner_node_indices_;

        ////////////////////////////////////////////////////////////////////////
        // Task specific objects
        ////////////////////////////////////////////////////////////////////////

        std::vector< btVector3 > cover_points_;
        smmap_msgs::GetMirrorLine::Response mirror_line_data_;

        ////////////////////////////////////////////////////////////////////////
        // ROS Objects and Helpers
        ////////////////////////////////////////////////////////////////////////

        ros::NodeHandle nh_;

        // global input mutex
        boost::mutex input_mtx_;

        ros::ServiceServer cmd_grippers_traj_srv_;
        smmap_msgs::CmdGrippersTrajectory::Request next_gripper_traj_;
        smmap_msgs::CmdGrippersTrajectory::Request curr_gripper_traj_;
        size_t gripper_traj_index_;
        bool new_gripper_traj_ready_;

        ros::Publisher simulator_fbk_pub_;

        ros::ServiceServer gripper_names_srv_;
        ros::ServiceServer gripper_attached_node_indices_srv_;
        ros::ServiceServer gripper_pose_srv_;
        ros::ServiceServer cover_points_srv_;
        ros::ServiceServer mirror_line_srv_;
        ros::ServiceServer object_initial_configuration_srv_;
        std::vector< geometry_msgs::Point > object_initial_configuration_;

        ros::Subscriber visualization_marker_sub_;
        ros::Subscriber visualization_marker_array_sub_;

        ////////////////////////////////////////////////////////////////////////
        // Key Handler for our Custom Scene
        ////////////////////////////////////////////////////////////////////////

        class CustomKeyHandler : public osgGA::GUIEventHandler
        {
            public:
                CustomKeyHandler( CustomScene &scene );

                bool handle( const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& );

            private:
                CustomScene& scene_;

                // stores the current gripper motion type
                GripperKinematicObject::Ptr current_gripper_;
                bool translate_gripper_;
                bool rotate_gripper_;

                // used to track how much to drag by
                bool start_dragging_;
                float mouse_last_x_;
                float mouse_last_y_;

                GripperKinematicObject::Ptr getGripper( size_t gripper_num );
        };
};

#endif
