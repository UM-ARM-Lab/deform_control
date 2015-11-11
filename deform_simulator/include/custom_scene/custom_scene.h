#ifndef CUSTOM_SCENE_H
#define CUSTOM_SCENE_H

#include <boost/thread/mutex.hpp>

#include "simulation/environment.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/rope.h"

#include "simulation/config_bullet.h"
#include "simulation/config_scene.h"
#include "simulation/config_viewer.h"

#include "gripper_kinematic_object.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// This pragma is here because the ROS message generator has an extra ';' on one
// line of code, and we can't push this off to be a system include
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "deform_simulator/SimulatorFbkStamped.h"
#include "deform_simulator/GetGripperAttachedNodeIndices.h"
#include "deform_simulator/GetGripperPose.h"
// This pragma is here because the service call has an empty request
// (or response) message thus the allocator that it is passed never gets used
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "deform_simulator/GetGripperNames.h"
#include "deform_simulator/GetPointSet.h"
#include "deform_simulator/CmdGrippersTrajectory.h"
#include "deform_simulator/SetVisualizationMarker.h"
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop

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

        CustomScene( DeformableType deformable_type, TaskType task_type,
                ros::NodeHandle& nh,
                const std::string& cmd_gripper_traj_topic = "cmd_gripper_traj",
                const std::string& simulator_fbk_topic = "simulator_fbk",
                const std::string& get_gripper_names_topic = "get_gripper_names",
                const std::string& get_gripper_attached_node_indices_topic = "get_gripper_attached_node_indices",
                const std::string& get_gripper_pose_topic = "get_gripper_pose",
                const std::string& get_object_initial_configuration_topic = "get_object_initial_configuration",
                const std::string& get_cover_points_topic = "get_cover_points",
                const std::string& set_visualization_marker_topic = "set_visualization_marker" );

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

        ////////////////////////////////////////////////////////////////////////
        // ROS Callbacks
        ////////////////////////////////////////////////////////////////////////

        bool cmdGripperTrajCallback(
                deform_simulator::CmdGrippersTrajectory::Request& req,
                deform_simulator::CmdGrippersTrajectory::Response& res );
        bool getGripperNamesCallback(
                deform_simulator::GetGripperNames::Request& req,
                deform_simulator::GetGripperNames::Response& res );
        bool getGripperAttachedNodeIndicesCallback(
                deform_simulator::GetGripperAttachedNodeIndices::Request& req,
                deform_simulator::GetGripperAttachedNodeIndices::Response& res );
        bool getGripperPoseCallback(
                deform_simulator::GetGripperPose::Request& req,
                deform_simulator::GetGripperPose::Response& res );
        bool getCoverPointsCallback(
                deform_simulator::GetPointSet::Request& req,
                deform_simulator::GetPointSet::Response& res );
        bool getObjectInitialConfigurationCallback(
                deform_simulator::GetPointSet::Request& req,
                deform_simulator::GetPointSet::Response& res );
        bool setVisualizationCallback(
                deform_simulator::SetVisualizationMarker::Request& req,
                deform_simulator::SetVisualizationMarker::Response& res );

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

        static constexpr float TABLE_X = 0.0; // METERS
        static constexpr float TABLE_Y = 0.0; // METERS
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
        boost::shared_ptr< CapsuleRope > rope_;

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

        std::vector< btVector3 > cover_points_;

        ////////////////////////////////////////////////////////////////////////
        // ROS Objects and Helpers
        ////////////////////////////////////////////////////////////////////////

        ros::NodeHandle nh_;

        // global input mutex
        boost::mutex input_mtx_;

        ros::ServiceServer cmd_grippers_traj_srv_;
        deform_simulator::CmdGrippersTrajectory::Request next_gripper_traj_;
        deform_simulator::CmdGrippersTrajectory::Request curr_gripper_traj_;
        size_t gripper_traj_index_;
        bool new_gripper_traj_ready_;

        ros::Publisher simulator_fbk_pub_;

        ros::ServiceServer gripper_names_srv_;
        ros::ServiceServer gripper_attached_node_indices_srv_;
        ros::ServiceServer gripper_pose_srv_;
        ros::ServiceServer cover_points_srv_;
        ros::ServiceServer object_initial_configuration_srv_;
        ros::ServiceServer set_visualization_marker_srv_;
        std::vector< geometry_msgs::Point > object_initial_configuration_;
};

#endif
