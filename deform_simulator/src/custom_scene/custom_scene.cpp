#include "custom_scene/custom_scene.h"

#include <limits>
#include <string>
#include <boost/thread.hpp>
#include <ros/callback_queue.h>
#include <smmap/ros_params.hpp>

#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>

#include <BulletSoftBody/btSoftBodyHelpers.h>

#include <bullet_helpers/bullet_internal_conversions.hpp>
#include <bullet_helpers/bullet_ros_conversions.hpp>
#include <bullet_helpers/bullet_math_helpers.hpp>
#include <bullet_helpers/bullet_pretty_print.hpp>

#include "utils/util.h"

using namespace BulletHelpers;

// TODO: find a way to not need to do this
// Making CLANG happy...
constexpr float CustomScene::TABLE_X;
constexpr float CustomScene::TABLE_Y;
constexpr float CustomScene::TABLE_Z;
constexpr float CustomScene::CLOTH_X;
constexpr float CustomScene::CLOTH_Y;
constexpr float CustomScene::CLOTH_Z;

////////////////////////////////////////////////////////////////////////////////
// Constructor and Destructor
////////////////////////////////////////////////////////////////////////////////

CustomScene::CustomScene( ros::NodeHandle& nh,
        smmap::DeformableType deformable_type, smmap::TaskType task_type)
    : plot_points_( new PlotPoints( 0.1f*METERS ) )
    , plot_lines_( new PlotLines( 0.25f*METERS ) )
    , deformable_type_( deformable_type )
    , task_type_( task_type )
    , nh_( nh )
    , cmd_grippers_traj_as_( nh, smmap::GetCommandGripperTrajTopic( nh ), false )
    , cmd_grippers_traj_goal_( nullptr )
    , num_timesteps_to_execute_per_gripper_cmd_( 4 )
{
    ROS_INFO( "Building the world" );
    // Build the world
    // TODO: make this setable/resetable via a ROS service call
    switch ( deformable_type_ )
    {
        case smmap::DeformableType::ROPE:
            makeRopeWorld();
            break;

        case smmap::DeformableType::CLOTH:
            makeClothWorld();
            break;

        default:
        {
            throw new std::invalid_argument( "Unknown deformable object type" );
        }
    };

    // Store the initial configuration as it will be needed by other libraries
    // TODO: find a better way to do this that exposes less internals
    object_initial_configuration_ = toRosPointVector( getDeformableObjectNodes(), METERS );

    ROS_INFO( "Creating subscribers and publishers" );
    // Publish to the feedback channel
    simulator_fbk_pub_ = nh_.advertise< smmap_msgs::SimulatorFeedback >(
            smmap::GetSimulatorFeedbackTopic( nh_ ), 20 );

    ROS_INFO( "Creating services" );
    // Create a service to let others know the internal gripper names
    gripper_names_srv_ = nh_.advertiseService(
            smmap::GetGripperNamesTopic( nh_ ), &CustomScene::getGripperNamesCallback, this );

    // Create a service to let others know what nodes the grippers are attached too
    gripper_attached_node_indices_srv_ = nh_.advertiseService(
            smmap::GetGripperAttachedNodeIndicesTopic( nh_ ), &CustomScene::getGripperAttachedNodeIndicesCallback, this );

    // Create a service to let others know the current gripper pose
    gripper_pose_srv_ = nh_.advertiseService(
            smmap::GetGripperPoseTopic( nh_ ), &CustomScene::getGripperPoseCallback, this);

    // Create a service to let others know the current gripper pose
    gripper_collision_check_srv_ = nh_.advertiseService(
            smmap::GetGripperCollisionCheckTopic( nh_ ), &CustomScene::gripperCollisionCheckCallback, this);

    // Create a service to let others know the cover points
    cover_points_srv_ = nh_.advertiseService(
            smmap::GetCoverPointsTopic( nh_ ), &CustomScene::getCoverPointsCallback, this );

    // Create a service to let others know the mirror line data
    mirror_line_srv_ = nh_.advertiseService(
            smmap::GetMirrorLineTopic( nh_ ), &CustomScene::getMirrorLineCallback, this );

    // Create a service to let others know the object initial configuration
    object_initial_configuration_srv_ = nh_.advertiseService(
            smmap::GetObjectInitialConfigurationTopic( nh_ ), &CustomScene::getObjectInitialConfigurationCallback, this );

    // Create a subscriber to take visualization instructions
    visualization_marker_sub_ = nh_.subscribe(
            smmap::GetVisualizationMarkerTopic( nh_ ), 20, &CustomScene::visualizationMarkerCallback, this );

    // Create a subscriber to take visualization instructions
    visualization_marker_array_sub_ = nh_.subscribe(
            smmap::GetVisualizationMarkerArrayTopic( nh_ ), 20, &CustomScene::visualizationMarkerArrayCallback, this );
}

////////////////////////////////////////////////////////////////////////////////
// Main function that makes things happen
////////////////////////////////////////////////////////////////////////////////

void CustomScene::run( bool syncTime )
{
    // Note that viewer cleans up this memory
    viewer.addEventHandler( new CustomKeyHandler( *this ) );

    // When the viewer closes, shutdown ROS
    addVoidCallback( osgGA::GUIEventAdapter::EventType::CLOSE_WINDOW,
            boost::bind( &ros::shutdown ) );

    addPreStepCallback( boost::bind( &CustomScene::drawAxes, this ) );

    // if syncTime is set, the simulator blocks until the real time elapsed
    // matches the simulator time elapsed, or something, it's not clear
    setSyncTime( syncTime );
    startViewer();

    // Let the object settle before anything else happens
    stepFor( BulletConfig::dt, 2 );

    base_sim_time_ = simTime;

    // Create a service to let others know the object current configuration
    object_current_configuration_srv_ = nh_.advertiseService(
            smmap::GetObjectCurrentConfigurationTopic( nh_ ), &CustomScene::getObjectCurrentConfigurationCallback, this );

    // Startup the action server
    cmd_grippers_traj_as_.start();

    // TODO: remove this hardcoded spin rate
    boost::thread spin_thread( boost::bind( &CustomScene::spin, 1000 ) );

    ROS_INFO( "Simulation ready." );

    // Run the simulation
    while ( ros::ok() )
    {
        // Check if we've been asked to follow a trajectory
        if ( cmd_grippers_traj_as_.isNewGoalAvailable() )
        {
            // If we already have a trajectory, premept the current one with the results so far
            if ( cmd_grippers_traj_as_.isActive() )
            {
                cmd_grippers_traj_as_.setPreempted( cmd_grippers_traj_result_ );
            }

            cmd_grippers_traj_goal_ = cmd_grippers_traj_as_.acceptNewGoal();
            cmd_grippers_traj_result_.sim_state_trajectory.clear();
            cmd_grippers_traj_result_.sim_state_trajectory.reserve( cmd_grippers_traj_goal_->trajectory.size() );
            cmd_grippers_traj_next_index_ = 0;
        }

        // If the current goal (new or not) has been preempted, send our current results and clear the goal
        if ( cmd_grippers_traj_as_.isPreemptRequested() )
        {
            cmd_grippers_traj_as_.setPreempted( cmd_grippers_traj_result_ );
            cmd_grippers_traj_goal_ = nullptr; // stictly speaking, this shouldn't be needed
        }

        // If we have not reached the end of the current gripper trajectory, execute the next step
        if ( cmd_grippers_traj_as_.isActive() )
        {
            // Advance the sim time and record the sim state
            boost::mutex::scoped_lock lock ( sim_mutex_ );
            moveGrippers();

            for ( size_t filter_ind = 0; filter_ind < num_timesteps_to_execute_per_gripper_cmd_; filter_ind++ )
            {
                step( BulletConfig::dt );
            }

            smmap_msgs::SimulatorFeedback msg = createSimulatorFbk();
            lock.unlock();

            // publish the message
            simulator_fbk_pub_.publish( msg );

            // Deal with the action server parts of feedback
            smmap_msgs::CmdGrippersTrajectoryFeedback as_feedback;
            as_feedback.sim_state = msg;
            cmd_grippers_traj_as_.publishFeedback( as_feedback );

            cmd_grippers_traj_result_.sim_state_trajectory.push_back( msg );

            if ( cmd_grippers_traj_next_index_ == cmd_grippers_traj_goal_->trajectory.size() )
            {
                cmd_grippers_traj_as_.setSucceeded( cmd_grippers_traj_result_ );
            }
        }
        else
        {
            boost::mutex::scoped_lock lock ( sim_mutex_ );
            step( 0 );
            lock.unlock();
            usleep( (__useconds_t)(BulletConfig::dt * 1e6) );
        }

        // TODO: should we send feedback on the generic FB channel regardless of time advancing?
    }

    // clean up the extra thread we started
    spin_thread.join();
}

////////////////////////////////////////////////////////////////////////////////
// Construction helper functions
////////////////////////////////////////////////////////////////////////////////

void CustomScene::makeTable( const float half_side_length, const bool set_cover_points )
{
    // table parameters
    const btVector3 table_surface_position = btVector3( TABLE_X, TABLE_Y, TABLE_Z ) * METERS;
    const btVector3 table_half_extents = btVector3( half_side_length, half_side_length, TABLE_THICKNESS*METERS/2 );

    // create the table
    table_ = BoxObject::Ptr( new BoxObject( 0, table_half_extents,
                btTransform( btQuaternion( 0, 0, 0, 1 ),
                    table_surface_position - btVector3( 0, 0, TABLE_THICKNESS*METERS/2 ) ) ) );
    table_->setColor( 0.4f, 0.4f, 0.4f, 1.0f );
    // TODO why was this not set for the rope and only for the cloth?
    table_->rigidBody->setFriction(1);

    env->add( table_ );

    // if we are doing a table coverage task, create the table coverage points
    if ( set_cover_points )
    {
        const float stepsize = 0.0125f*METERS;
        btTransform table_tf = table_->rigidBody->getCenterOfMassTransform();

        std::vector< btVector3 > cloth_coverage_lines;
        for(float y = -table_->halfExtents.y(); y <= table_->halfExtents.y(); y += stepsize)
        {
            // Add a coverage line to the visualization
            cloth_coverage_lines.push_back(
                    table_tf * btVector3 ( -table_->halfExtents.x(), y, table_->halfExtents.z() ) );

            cloth_coverage_lines.push_back(
                    table_tf * btVector3 ( +table_->halfExtents.x(), y, table_->halfExtents.z() ) );

            // Add many coverage points along the coverage line
            for(float x = -table_->halfExtents.x(); x <= table_->halfExtents.x(); x += stepsize)
            {
                cover_points_.push_back( table_tf * btVector3( x, y, table_->halfExtents.z() ) );
            }
        }
        ROS_INFO_STREAM( "Number of cover points: " << cover_points_.size() );

        std::vector<btVector4> cloth_coverage_color( cloth_coverage_lines.size(), btVector4( 1, 0, 0, 1 ) );
        plot_lines_->setPoints( cloth_coverage_lines, cloth_coverage_color );
        env->add( plot_lines_ );
    }
}

void CustomScene::makeCylinder( const bool set_cover_points )
{
    // cylinder parameters
    // NOTE: this currently has part of the cylinder inside the table
    const btVector3 cylinder_com_origin =
        table_->rigidBody->getCenterOfMassTransform().getOrigin() +
        //btVector3( 0, 0, TABLE_THICKNESS*METERS/2 ) +
        btVector3( 0, ROPE_CYLINDER_RADIUS*METERS*5/3, ROPE_CYLINDER_HEIGHT*METERS/2 );

    // create a cylinder
    cylinder_ = CylinderStaticObject::Ptr( new CylinderStaticObject(
                0, ROPE_CYLINDER_RADIUS*METERS, ROPE_CYLINDER_HEIGHT*METERS,
                btTransform( btQuaternion( 0, 0, 0, 1 ), cylinder_com_origin ) ) );
    cylinder_->setColor( 179.0f/255.0f, 176.0f/255.0f, 160.0f/255.0f, 1.0f );

    // add the cylinder to the world
    env->add( cylinder_ );

    if ( set_cover_points )
    {
        // find the points that we want to cover with a rope

        // consider 21 points around the cylinder
        for( float theta = 0; theta < 2.0f * M_PI; theta += 0.3f )
        // NOTE: this 0.3 ought to be 2*M_PI/21=0.299199... however that chops off the last value, probably due to rounding
        {
            // 31 points per theta
            for( float h = 0; h < ROPE_CYLINDER_HEIGHT*METERS; h += ROPE_CYLINDER_HEIGHT*METERS/30.0f )
            {
                cover_points_.push_back(
                        cylinder_com_origin
                        + btVector3( (ROPE_CYLINDER_RADIUS*METERS + rope_->radius/2)*std::cos( theta ),
                                     (ROPE_CYLINDER_RADIUS*METERS + rope_->radius/2)*std::sin( theta ),
                                     h - ROPE_CYLINDER_HEIGHT*METERS/2 ) );
            }
        }
        ROS_INFO_STREAM( "Number of cover points: " << cover_points_.size() ) ;

        std::vector<btVector4> rope_coverage_color( cover_points_.size(), btVector4( 1, 0, 0, 1 ) );
        plot_points_->setPoints( cover_points_, rope_coverage_color );
        env->add( plot_points_ );
    }
}

void CustomScene::makeRope()
{
    // find the needed table parameters
    const btVector3 table_surface_position = btVector3( TABLE_X, TABLE_Y, TABLE_Z ) * METERS;

    // make the rope
    std::vector<btVector3> control_points( ROPE_NUM_LINKS );
    for ( int n = 0; n < ROPE_NUM_LINKS; n++ )
    {
        // TODO: get rid of this random "- 20"
        control_points[(size_t)n] = table_surface_position +
//            btVector3( ((float)n - (float)(ROPE_NUM_LINKS - 1)/2)*ROPE_SEGMENT_LENGTH, 0, 5*ROPE_RADIUS ) * METERS;
            btVector3( (float)(n - 20)*ROPE_SEGMENT_LENGTH, 0, 5*ROPE_RADIUS ) * METERS;

    }
    rope_.reset( new CapsuleRope( control_points, ROPE_RADIUS*METERS ) );

    // color the rope
    std::vector< BulletObject::Ptr > children = rope_->getChildren();
    for ( size_t j = 0; j < children.size(); j++ )
    {
        children[j]->setColor( 0.15f, 0.65f, 0.15f, 1.0f );
    }

    // add the table and rope to the world
    env->add( rope_ );
}

void CustomScene::makeCloth()
{
    // cloth parameters
    const btVector3 cloth_center = btVector3( CLOTH_X, CLOTH_Y, CLOTH_Z )*METERS;

    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        env->bullet->softBodyWorldInfo,
        cloth_center + btVector3( -CLOTH_HALF_SIDE_LENGTH, -CLOTH_HALF_SIDE_LENGTH, 0) * METERS,
        cloth_center + btVector3( +CLOTH_HALF_SIDE_LENGTH, -CLOTH_HALF_SIDE_LENGTH, 0) * METERS,
        cloth_center + btVector3( -CLOTH_HALF_SIDE_LENGTH, +CLOTH_HALF_SIDE_LENGTH, 0) * METERS,
        cloth_center + btVector3( +CLOTH_HALF_SIDE_LENGTH, +CLOTH_HALF_SIDE_LENGTH, 0) * METERS,
        CLOTH_DIVS, CLOTH_DIVS,
        0, true);

    psb->m_cfg.piterations = 10;//2;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
        | btSoftBody::fCollision::CL_RS; //  | btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = 1.0;
    psb->getCollisionShape()->setMargin( 0.05f );
    btSoftBody::Material *pm = psb->appendMaterial();
    // commented out as there are no self collisions
    //pm->m_kLST = 0.2;//0.1; //makes it rubbery (handles self collisions better)
    psb->m_cfg.kDP = 0.05f;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    // TODO: why is setTotalMass being called twice?
    psb->setTotalMass(1, true);
    psb->generateClusters(0);
    psb->setTotalMass(0.1f);

    // commented out as there are no self collisions
/*    psb->generateClusters(500);

    for (int i = 0; i < psb->m_clusters.size(); ++i) {
        psb->m_clusters[i]->m_selfCollisionImpulseFactor = 0.1;
    }
*/

    cloth_.reset( new BulletSoftObject( psb ) );
    // note that we need to add the cloth to the environment before setting the
    // color, otherwise we get a segfault
    env->add( cloth_ );
    cloth_->setColor( 0.15f, 0.65f, 0.15f, 1.0f );

    findClothCornerNodes();
}

void CustomScene::makeRopeWorld()
{
    // Here we assume that we are already working with a rope object
    switch ( task_type_ )
    {
        case smmap::TaskType::COVERAGE:
        {
            const bool table_set_cover_points = false;
            const bool cylinder_set_cover_points = true;
            makeTable( ROPE_TABLE_HALF_SIDE_LENGTH*METERS, table_set_cover_points );
            makeRope();
            makeCylinder( cylinder_set_cover_points );

            // add a single auto gripper to the world
            grippers_["gripper"] = GripperKinematicObject::Ptr(
                    new GripperKinematicObject( "gripper", ROPE_GRIPPER_APPERTURE*METERS ) );
            grippers_["gripper"]->setWorldTransform(
                    rope_->children[0]->rigidBody->getCenterOfMassTransform() );
            grippers_["gripper"]->rigidGrab( rope_->children[0]->rigidBody.get(), 0, env );

            auto_grippers_.push_back( "gripper" );

            break;
        }
        default:
        {
            throw new std::invalid_argument( "Unknown task type for a ROPE object" );
        }
    }

    for ( auto& gripper: grippers_ )
    {
        gripper_axes_[gripper.first] = PlotAxes::Ptr( new PlotAxes() );

        // Add the gripper and it's axis to the world
        env->add( gripper.second );
        env->add( gripper_axes_[gripper.first] );
    }

    // Add a gripper that is in the same state as used for the rope experiments
    collision_check_gripper_ = GripperKinematicObject::Ptr(
                new GripperKinematicObject( "collision_check_gripper",
                                            ROPE_GRIPPER_APPERTURE*METERS,
                                            btVector4( 0, 0, 0, 0 ) ) );
    collision_check_gripper_->setWorldTransform( btTransform() );
    env->add( collision_check_gripper_ );
}

void CustomScene::makeClothWorld()
{
    switch ( task_type_ )
    {
        case smmap::TaskType::COVERAGE:
        {
            const bool set_cover_points = true;
            makeTable( CLOTH_TABLE_HALF_SIDE_LENGTH*METERS, set_cover_points );
            makeCloth();

            /*
             * add 2 auto grippers to the world
             */

            btVector3 gripper_half_extents;

            // auto gripper0
            grippers_["auto_gripper0"] = GripperKinematicObject::Ptr(
                    new GripperKinematicObject( "auto_gripper0", CLOTH_GRIPPER_APPERTURE*METERS ) );
            gripper_half_extents = grippers_["auto_gripper0"]->getHalfExtents();
            grippers_["auto_gripper0"]->setWorldTransform(
                    btTransform( btQuaternion( 0, 0, 0, 1 ),
                                 cloth_->softBody->m_nodes[cloth_corner_node_indices_[0]].m_x
                                 + btVector3( gripper_half_extents.x(), gripper_half_extents.y(), 0 ) ) );

            auto_grippers_.push_back( "auto_gripper0" );

            // auto gripper1
            grippers_["auto_gripper1"] = GripperKinematicObject::Ptr(
                    new GripperKinematicObject( "auto_gripper1", CLOTH_GRIPPER_APPERTURE*METERS ) );
            gripper_half_extents = grippers_["auto_gripper1"]->getHalfExtents();
            grippers_["auto_gripper1"]->setWorldTransform(
                    btTransform( btQuaternion( 0, 0, 0, 1 ),
                                 cloth_->softBody->m_nodes[cloth_corner_node_indices_[1]].m_x
                                 + btVector3( gripper_half_extents.x(), -gripper_half_extents.y(), 0 ) ) );

            auto_grippers_.push_back( "auto_gripper1" );

            break;
        }
        case smmap::TaskType::COLAB_FOLDING:
        {
            makeCloth();

            /*
             * add 2 auto grippers to the world
             */

            btVector3 gripper_half_extents;

            // auto gripper0
            grippers_["auto_gripper0"] = GripperKinematicObject::Ptr(
                    new GripperKinematicObject( "auto_gripper0", CLOTH_GRIPPER_APPERTURE*METERS ) );
            gripper_half_extents = grippers_["auto_gripper0"]->getHalfExtents();
            grippers_["auto_gripper0"]->setWorldTransform(
                    btTransform( btQuaternion( 0, 0, 0, 1 ),
                                 cloth_->softBody->m_nodes[cloth_corner_node_indices_[0]].m_x
                                 + btVector3( gripper_half_extents.x(), gripper_half_extents.y(), 0 ) ) );

            auto_grippers_.push_back( "auto_gripper0" );

            // auto gripper1
            grippers_["auto_gripper1"] = GripperKinematicObject::Ptr(
                    new GripperKinematicObject( "auto_gripper1", CLOTH_GRIPPER_APPERTURE*METERS ) );
            gripper_half_extents = grippers_["auto_gripper1"]->getHalfExtents();
            grippers_["auto_gripper1"]->setWorldTransform(
                    btTransform( btQuaternion( 0, 0, 0, 1 ),
                                 cloth_->softBody->m_nodes[cloth_corner_node_indices_[1]].m_x
                                 + btVector3( gripper_half_extents.x(), -gripper_half_extents.y(), 0 ) ) );

            auto_grippers_.push_back( "auto_gripper1" );

            /*
             * add 2 manual grippers to the world
             */

            // manual gripper0
            grippers_["manual_gripper0"] = GripperKinematicObject::Ptr(
                    new GripperKinematicObject( "manual_gripper0", CLOTH_GRIPPER_APPERTURE*METERS,
                                                btVector4( 0.0f, 0.0f, 0.6f, 0.9f ) ) );
            gripper_half_extents = grippers_["manual_gripper0"]->getHalfExtents();
            grippers_["manual_gripper0"]->setWorldTransform(
                    btTransform( btQuaternion( 0, 0, 0, 1 ),
                                 cloth_->softBody->m_nodes[cloth_corner_node_indices_[2]].m_x
                                 + btVector3( -gripper_half_extents.x(), gripper_half_extents.y(), 0 ) ) );

            manual_grippers_.push_back( "manual_gripper0" );
            manual_grippers_paths_.push_back( smmap::ManualGripperPath( grippers_["manual_gripper0"], &smmap::gripperPath0 ) );

            // manual gripper1
            grippers_["manual_gripper1"] = GripperKinematicObject::Ptr(
                    new GripperKinematicObject( "manual_gripper1", CLOTH_GRIPPER_APPERTURE*METERS,
                                                btVector4( 0.0f, 0.0f, 0.6f, 0.9f )  ) );
            gripper_half_extents = grippers_["manual_gripper1"]->getHalfExtents();
            grippers_["manual_gripper1"]->setWorldTransform(
                    btTransform( btQuaternion( 0, 0, 0, 1 ),
                                 cloth_->softBody->m_nodes[cloth_corner_node_indices_[3]].m_x
                                 + btVector3( -gripper_half_extents.x(), -gripper_half_extents.y(), 0 ) ) );

            manual_grippers_.push_back( "manual_gripper1" );
            manual_grippers_paths_.push_back( smmap::ManualGripperPath( grippers_["manual_gripper1"], &smmap::gripperPath1 ) );

            break;
        }
        default:
        {
            throw new std::invalid_argument( "Unknown task type for a CLOTH object" );
        }
    }

    for ( auto& gripper: grippers_ )
    {
        // Grip the cloth
        gripper.second->toggleOpen();
        gripper.second->toggleAttach( cloth_->softBody.get() );

        gripper_axes_[gripper.first] = PlotAxes::Ptr( new PlotAxes() );

        // Add the gripper and it's axis to the world
        env->add( gripper.second );
        env->add( gripper_axes_[gripper.first] );

        // Add a callback in case this gripper gets step_openclose activated on it
        // This is a state machine whose input comes from CustomKeyHandler
        addPreStepCallback( boost::bind( &GripperKinematicObject::step_openclose, gripper.second, cloth_->softBody.get() ) );
    }

    // Add a gripper that is in the same state as used for the cloth experiments
    collision_check_gripper_ = GripperKinematicObject::Ptr(
                new GripperKinematicObject( "collision_check_gripper",
                                            CLOTH_GRIPPER_APPERTURE*METERS,
                                            btVector4( 0, 0, 0, 0 ) ) );
    collision_check_gripper_->setWorldTransform( btTransform() );
    collision_check_gripper_->toggleOpen();
    env->add( collision_check_gripper_ );
}

/**
 * Sets cloth_corner_node_indices_ to the extremal points of the mesh
 *
 * cloth_corner_node_indices_[0] = minx_miny
 * cloth_corner_node_indices_[1] = minx_maxy
 * cloth_corner_node_indices_[2] = maxx_miny
 * cloth_corner_node_indices_[3] = maxx_maxy
 */
void CustomScene::findClothCornerNodes()
{
    cloth_corner_node_indices_.resize(4, 0);
    // Setup defaults for doing the max and min operations inside of the loop
    std::vector< btVector3 > corner_node_positions( 4 );

    // min_x, min_y
    corner_node_positions[0] = btVector3(
            std::numeric_limits< btScalar >::infinity(),
            std::numeric_limits< btScalar >::infinity(),
            0 );

    // min_x, max_y
    corner_node_positions[1] = btVector3(
            std::numeric_limits< btScalar >::infinity(),
            -std::numeric_limits< btScalar >::infinity(),
            0 );

    // max_x, min_y
    corner_node_positions[2] = btVector3(
            -std::numeric_limits< btScalar >::infinity(),
            std::numeric_limits< btScalar >::infinity(),
            0 );

    // max_x, max_y
    corner_node_positions[3] = btVector3(
            -std::numeric_limits< btScalar >::infinity(),
            -std::numeric_limits< btScalar >::infinity(),
            0 );

    btSoftBody::tNodeArray cloth_nodes = cloth_->softBody->m_nodes;

    // Itterate through the nodes in the cloth, finding the extremal points
    for ( int ind = 0; ind < cloth_nodes.size(); ind++ )
    {
        if ( cloth_nodes[ind].m_x.x() <= corner_node_positions[0].x() &&
                cloth_nodes[ind].m_x.y() <= corner_node_positions[0].y() )
        {
            cloth_corner_node_indices_[0] = ind;
            corner_node_positions[0] = cloth_nodes[ind].m_x;
        }
        else if ( cloth_nodes[ind].m_x.x() <= corner_node_positions[1].x() &&
                cloth_nodes[ind].m_x.y() >= corner_node_positions[1].y() )
        {
            cloth_corner_node_indices_[1] = ind;
            corner_node_positions[1] = cloth_nodes[ind].m_x;
        }
        else if ( cloth_nodes[ind].m_x.x() >= corner_node_positions[2].x() &&
                cloth_nodes[ind].m_x.y() <= corner_node_positions[2].y() )
        {
            cloth_corner_node_indices_[2] = ind;
            corner_node_positions[2] = cloth_nodes[ind].m_x;
        }
        else if ( cloth_nodes[ind].m_x.x() >= corner_node_positions[3].x() &&
                cloth_nodes[ind].m_x.y() >= corner_node_positions[3].y() )
        {
            cloth_corner_node_indices_[3] = ind;
            corner_node_positions[3] = cloth_nodes[ind].m_x;
        }
    }

    // Create a mirror line if we are doing colaborative folding
    if ( task_type_ == smmap::TaskType::COLAB_FOLDING )
    {
        mirror_line_data_.min_y = corner_node_positions[0].y() / METERS;
        mirror_line_data_.max_y = corner_node_positions[3].y() / METERS;
        mirror_line_data_.mid_x = ( corner_node_positions[0].x() +
                (corner_node_positions[3].x() - corner_node_positions[0].x() ) / 2 )  / METERS;

        std::vector< btVector3 > mirror_line_points;
        mirror_line_points.push_back( btVector3( (float)mirror_line_data_.mid_x, (float)mirror_line_data_.min_y, 0.8f) * METERS );
        mirror_line_points.push_back( btVector3( (float)mirror_line_data_.mid_x, (float)mirror_line_data_.max_y, 0.8f) * METERS );
        std::vector< btVector4 > mirror_line_colors;
        mirror_line_colors.push_back( btVector4(1,0,0,1) );

        PlotLines::Ptr line_strip( new PlotLines( 0.1f * METERS ) );
        line_strip->setPoints( mirror_line_points,
                               mirror_line_colors );
        visualization_line_markers_["mirror_line"] = line_strip;

        env->add( line_strip );
    }
}

////////////////////////////////////////////////////////////////////////////////
// Main loop helper functions
////////////////////////////////////////////////////////////////////////////////

void CustomScene::moveGrippers()
{
    // Given that we are moving the grippers, we know that cmd_grippers_traj_goal_
    // is valid, and cmd_grippers_traj_index_ is less than the length of the
    // trajectories we are following
    // TODO: remove these asserts once I've confirmed that I haven't made any coding errors
    assert( cmd_grippers_traj_goal_ != nullptr );
    assert( cmd_grippers_traj_goal_->trajectory.size() > 0 );
    assert( cmd_grippers_traj_next_index_ < cmd_grippers_traj_goal_->trajectory.size() );

    // TODO check for valid gripper names (and length of names vector)
    for ( size_t gripper_ind = 0; gripper_ind < cmd_grippers_traj_goal_->gripper_names.size(); gripper_ind++ )
    {
        GripperKinematicObject::Ptr gripper = grippers_.at( cmd_grippers_traj_goal_->gripper_names[gripper_ind] );

        btTransform tf = toBulletTransform(
                    cmd_grippers_traj_goal_->trajectory[cmd_grippers_traj_next_index_].pose[gripper_ind], METERS );

        // prevent the gripper from going into table for the rope coverage task
        if ( deformable_type_ == smmap::DeformableType::ROPE &&
             task_type_ == smmap::TaskType::COVERAGE )
        {
            if ( tf.getOrigin().z() - gripper->getGripperRadius() < TABLE_Z * METERS )
            {
                std::cerr << "Moving gripper to ";
                tf.setOrigin( btVector3( tf.getOrigin().x(), tf.getOrigin().y(), TABLE_Z * METERS + gripper->getGripperRadius() ) );
                std::cerr << tf.getOrigin().z() << std::endl;
            }
        }

        gripper->setWorldTransform( tf );
    }
    cmd_grippers_traj_next_index_++;

    // Advance the manual grippers on their paths
    for ( auto& path: manual_grippers_paths_ )
    {
        path.advanceGripper();
    }
}

smmap_msgs::SimulatorFeedback CustomScene::createSimulatorFbk()
{
    assert( num_timesteps_to_execute_per_gripper_cmd_ > 0 );

    smmap_msgs::SimulatorFeedback msg;

    // fill out the object configuration data
    msg.object_configuration = toRosPointVector( getDeformableObjectNodes(), METERS );

    // fill out the gripper data
    for ( const std::string &gripper_name: auto_grippers_ )
    {
        msg.gripper_names.push_back( gripper_name );
        msg.gripper_poses.push_back( toRosPose( grippers_[gripper_name]->getWorldTransform(), METERS ) );

        btPointCollector collision_result = collisionHelper( grippers_[gripper_name] );

        if ( collision_result.m_hasResult )
        {
            msg.gripper_distance_to_obstacle.push_back( collision_result.m_distance/METERS );
        }
        else
        {
            msg.gripper_distance_to_obstacle.push_back( std::numeric_limits< double >::infinity() );
        }

        msg.obstacle_surface_normal.push_back( toRosVector3( collision_result.m_normalOnBInWorld, 1 ) );
        msg.gripper_nearest_point_to_obstacle.push_back( toRosPoint(
                    collision_result.m_pointInWorld
                    + collision_result.m_normalOnBInWorld * collision_result.m_distance, METERS ) );
    }

    // update the sim_time
    msg.sim_time = ( simTime - base_sim_time_ ) / (double)num_timesteps_to_execute_per_gripper_cmd_;

    return msg;
}

////////////////////////////////////////////////////////////////////////////////
// Internal helper functions
////////////////////////////////////////////////////////////////////////////////

std::vector< btVector3 > CustomScene::getDeformableObjectNodes() const
{
    std::vector< btVector3 > nodes;

    switch ( deformable_type_ )
    {
        case smmap::DeformableType::ROPE:
            nodes = rope_->getNodes();
            break;

        case smmap::DeformableType::CLOTH:
            nodes = nodeArrayToNodePosVector( cloth_->softBody->m_nodes );
            break;
    }

    return nodes;
}

/**
 * @brief Invoke bullet's collision detector to find the points on the gripper
 * and cylinder/table that are nearest each other.
 *
 * @param gripper_name The name of the gripper to check for collision
 *
 * @return The result of the collision check
 */
btPointCollector CustomScene::collisionHelper(
        const GripperKinematicObject::Ptr& gripper )
{
    assert( gripper );

    BulletObject::Ptr obj = nullptr;
    // Note that gjkOutput initializes to hasResult = false and m_distance = BT_LARGE_FLOAT
    btPointCollector gjkOutput_min;

    if ( task_type_ == smmap::TaskType::COVERAGE )
    {
        if ( deformable_type_ == smmap::DeformableType::ROPE )
        {
            obj = cylinder_;
        }
        else if ( deformable_type_ == smmap::DeformableType::CLOTH )
        {
            obj = table_;
        }
        else
        {
            // This error should never happen, as elsewhere in the code we throw
            // exceptions
            ROS_ERROR_NAMED( "collision_detection", "Unknown deformable + coverage combination!" );
        }
    }

    // find the distance to the object
    if ( obj )
    {
        for ( size_t gripper_child_ind = 0; gripper_child_ind < gripper->getChildren().size(); gripper_child_ind++ )
        {
            // TODO: how much (if any) of this should be static/class members?
            btGjkEpaPenetrationDepthSolver epaSolver;
            btVoronoiSimplexSolver sGjkSimplexSolver;
            btPointCollector gjkOutput;

            btGjkPairDetector convexConvex( dynamic_cast< btBoxShape* >( gripper->getChildren()[gripper_child_ind]->collisionShape.get() ),
                    dynamic_cast< btConvexShape* >(obj->collisionShape.get()), &sGjkSimplexSolver, &epaSolver );

            btGjkPairDetector::ClosestPointInput input;

            gripper->children[gripper_child_ind]->motionState->getWorldTransform( input.m_transformA );
            obj->motionState->getWorldTransform( input.m_transformB );
            input.m_maximumDistanceSquared = btScalar(BT_LARGE_FLOAT);
            convexConvex.getClosestPoints( input, gjkOutput, nullptr );

            if ( gjkOutput.m_distance < gjkOutput_min.m_distance )
            {
                gjkOutput_min = gjkOutput;
            }
        }
    }

    return gjkOutput_min;
}

////////////////////////////////////////////////////////////////////////////////
// ROS Callbacks
////////////////////////////////////////////////////////////////////////////////

bool CustomScene::getGripperNamesCallback(
        smmap_msgs::GetGripperNames::Request& req,
        smmap_msgs::GetGripperNames::Response& res )
{
    (void)req;
    res.names = auto_grippers_;
    return true;
}

bool CustomScene::getGripperAttachedNodeIndicesCallback(
        smmap_msgs::GetGripperAttachedNodeIndices::Request& req,
        smmap_msgs::GetGripperAttachedNodeIndices::Response& res )
{
    GripperKinematicObject::Ptr gripper = grippers_.at( req.name );
    res.indices = gripper->getAttachedNodeIndices();
    return true;
}

bool CustomScene::getGripperPoseCallback(
        smmap_msgs::GetGripperPose::Request& req,
        smmap_msgs::GetGripperPose::Response& res )
{
    GripperKinematicObject::Ptr gripper = grippers_.at( req.name );
    res.pose = toRosPose( gripper->getWorldTransform(), METERS );
    return true;
}

bool CustomScene::gripperCollisionCheckCallback(
        smmap_msgs::GetGripperCollisionReport::Request& req,
        smmap_msgs::GetGripperCollisionReport::Response& res )
{
    size_t num_checks = req.pose.size();

    res.gripper_distance_to_obstacle.resize( num_checks );
    res.gripper_nearest_point_to_obstacle.resize( num_checks );
    res.obstacle_surface_normal.resize( num_checks );

    for ( size_t pose_ind = 0; pose_ind < num_checks; pose_ind++ )
    {
        collision_check_gripper_->setWorldTransform( toBulletTransform( req.pose[pose_ind], METERS ) );
        btPointCollector collision_result = collisionHelper( collision_check_gripper_ );

        if ( collision_result.m_hasResult )
        {
            res.gripper_distance_to_obstacle[pose_ind] = collision_result.m_distance/METERS;
        }
        else
        {
            res.gripper_distance_to_obstacle[pose_ind] = std::numeric_limits< double >::infinity();
        }

        res.obstacle_surface_normal[pose_ind] = toRosVector3( collision_result.m_normalOnBInWorld, 1 );
        res.gripper_nearest_point_to_obstacle[pose_ind] = toRosPoint(
                    collision_result.m_pointInWorld
                    + collision_result.m_normalOnBInWorld * collision_result.m_distance, METERS );
    }

    return true;
}

bool CustomScene::getCoverPointsCallback(
        smmap_msgs::GetPointSet::Request& req,
        smmap_msgs::GetPointSet::Response& res )
{
    (void)req;
    res.points = toRosPointVector( cover_points_, METERS );
    return true;
}

bool CustomScene::getMirrorLineCallback(
        smmap_msgs::GetMirrorLine::Request& req,
        smmap_msgs::GetMirrorLine::Response& res )
{
    (void)req;
    if ( task_type_ == smmap::TaskType::COLAB_FOLDING &&
         deformable_type_ == smmap::DeformableType::CLOTH )
    {
        res = mirror_line_data_;
        return true;
    }
    else
    {
        res.mid_x = std::numeric_limits< double >::infinity();
        res.min_y = std::numeric_limits< double >::infinity();
        res.max_y = std::numeric_limits< double >::infinity();
        return false;
    }
}

bool CustomScene::getObjectInitialConfigurationCallback(
        smmap_msgs::GetPointSet::Request& req,
        smmap_msgs::GetPointSet::Response& res )
{
    (void)req;
    res.points = object_initial_configuration_;
    return true;
}

bool CustomScene::getObjectCurrentConfigurationCallback(
        smmap_msgs::GetPointSet::Request& req,
        smmap_msgs::GetPointSet::Response& res )
{
    (void)req;
    res.points = toRosPointVector( getDeformableObjectNodes(), METERS );
    return true;
}

// TODO: be able to delete markers and have a timeout
void CustomScene::visualizationMarkerCallback(
        visualization_msgs::Marker marker )
{
    std::string id = marker.ns + std::to_string( marker.id );

    // TODO: make this mutex not quite so "global" around this switch
    boost::mutex::scoped_lock lock( sim_mutex_ );

    switch ( marker.type )
    {
        case visualization_msgs::Marker::POINTS:
        {
            if ( visualization_point_markers_.count(id) == 0 )
            {
                PlotPoints::Ptr points( new PlotPoints() );
                points->setPoints( toOsgRefVec3Array( marker.points, METERS ),
                                   toOsgRefVec4Array( marker.colors ) );
                visualization_point_markers_[id] = points;

                env->add( points );
            }
            else
            {
                PlotPoints::Ptr points = visualization_point_markers_[id];
                points->setPoints( toOsgRefVec3Array( marker.points, METERS ),
                                   toOsgRefVec4Array( marker.colors ) );
            }
            break;
        }
        case visualization_msgs::Marker::SPHERE:
        {
            if ( visualization_sphere_markers_.count(id) == 0 )
            {
                PlotSpheres::Ptr spheres( new PlotSpheres() );
                spheres->plot( toOsgRefVec3Array( marker.points, METERS ),
                               toOsgRefVec4Array( marker.colors ),
                               std::vector< float >( marker.points.size(), (float)marker.scale.x * METERS ) );
                visualization_sphere_markers_[id] = spheres;

                env->add( spheres );
            }
            else
            {
                PlotSpheres::Ptr spheres = visualization_sphere_markers_[id];
                spheres->plot( toOsgRefVec3Array( marker.points, METERS ),
                               toOsgRefVec4Array( marker.colors ),
                               std::vector< float >( marker.points.size(), (float)marker.scale.x * METERS ) );
            }
            break;
        }
        case visualization_msgs::Marker::LINE_STRIP:
        {
            convertLineStripToLineList( marker );
        }
        case visualization_msgs::Marker::LINE_LIST:
        {
            // if the object is new, add it
            if ( visualization_line_markers_.count( id ) == 0 )
            {
                PlotLines::Ptr line_strip( new PlotLines( (float)marker.scale.x * METERS ) );
                line_strip->setPoints( toBulletPointVector( marker.points, METERS ),
                                       toBulletColorArray( marker.colors ));
                visualization_line_markers_[id] = line_strip;

                env->add( line_strip );
            }
            else
            {
                PlotLines::Ptr line_strip = visualization_line_markers_[id];
                line_strip->setPoints( toBulletPointVector( marker.points, METERS ),
                                       toBulletColorArray( marker.colors ));
            }
            break;
        }
        default:
        {
            ROS_ERROR_STREAM_NAMED( "visualization",
                    "Marker type " << marker.type << " not implemented " << id );
        }
    }
}

void CustomScene::visualizationMarkerArrayCallback(
        visualization_msgs::MarkerArray marker_array )
{
    for ( visualization_msgs::Marker marker: marker_array.markers )
    {
        visualizationMarkerCallback( marker );
    }
}

////////////////////////////////////////////////////////////////////////////////
// ROS Objects and Helpers
////////////////////////////////////////////////////////////////////////////////

void CustomScene::spin( double loop_rate )
{
    ros::NodeHandle ph("~");
    ROS_INFO( "Starting feedback spinner" );
    while ( ros::ok() )
    {
        ros::getGlobalCallbackQueue()->callAvailable( ros::WallDuration( loop_rate ) );
    }
}

////////////////////////////////////////////////////////////////////////////////
// Pre-step Callbacks
////////////////////////////////////////////////////////////////////////////////

void CustomScene::drawAxes()
{
    for ( auto& axes: gripper_axes_ )
    {
        axes.second->setup( grippers_[axes.first]->getWorldTransform(), 1 );
    }
}

////////////////////////////////////////////////////////////////////////////////
// Post-step Callbacks
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////
// Key Handler for our Custom Scene
////////////////////////////////////////////////////////////////////////

CustomScene::CustomKeyHandler::CustomKeyHandler( CustomScene &scene )
    : scene_( scene )
    , current_gripper_( nullptr )
    , translate_gripper_( false )
    , rotate_gripper_( false )
{}

bool CustomScene::CustomKeyHandler::handle( const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter & aa )
{
    (void)aa;
    bool suppress_default_handler = false;

    switch ( ea.getEventType() )
    {
        case osgGA::GUIEventAdapter::KEYDOWN:
        {
            switch ( ea.getKey() )
            {
                // Gripper translate/rotate selection keybinds
                {
                    case '1':
                    {
                        current_gripper_ = getGripper( 0 );
                        translate_gripper_ = true;
                        rotate_gripper_ = false;
                        break;
                    }
                    case 'q':
                    {
                        current_gripper_ = getGripper( 0 );
                        translate_gripper_ = false;
                        rotate_gripper_ = true;
                        break;
                    }

                    case '2':
                    {
                        current_gripper_ = getGripper( 1 );
                        translate_gripper_ = true;
                        rotate_gripper_ = false;
                        break;
                    }
                    case 'w':
                    {
                        current_gripper_ = getGripper( 1 );
                        translate_gripper_ = false;
                        rotate_gripper_ = true;
                        break;
                    }

                    case '3':
                    {
                        current_gripper_ = getGripper( 2 );
                        translate_gripper_ = true;
                        rotate_gripper_ = false;
                        break;
                    }
                    case 'e':
                    {
                        current_gripper_ = getGripper( 2 );
                        translate_gripper_ = false;
                        rotate_gripper_ = true;
                        break;
                    }

                    case '4':
                    {
                        current_gripper_ = getGripper( 3 );
                        translate_gripper_ = true;
                        rotate_gripper_ = false;
                        break;
                    }
                    case 'r':
                    {
                        current_gripper_ = getGripper( 3 );
                        translate_gripper_ = false;
                        rotate_gripper_ = true;
                        break;
                    }
                }

/*
                case '[':
                {
                    scene.left_gripper1->toggle();
                    scene.left_gripper1->toggleattach( scene.clothPtr->softBody.get() );
                    if( scene.num_auto_grippers > 1 )
                    {
                        scene.left_gripper2->toggle();
                        scene.left_gripper2->toggleattach( scene.clothPtr->softBody.get() );
                    }
                    break;
                }

                case ']':
                {
                    if( scene.num_auto_grippers > 1 )
                        scene.corner_number_ += 2;
                    else
                        scene.corner_number_ += 1;
                    if( scene.corner_number_ > 3 )
                        scene.corner_number_ = 0;
                    scene.left_gripper1->setWorldTransform( btTransform( btQuaternion( 0,0,0,1 ),
                        scene.clothPtr->softBody->m_nodes[scene.corner_grasp_point_inds[scene.corner_number_]].m_x ) );
                    if( scene.num_auto_grippers > 1 )
                        scene.left_gripper2->setWorldTransform( btTransform( btQuaternion( 0,0,0,1 ),
                            scene.clothPtr->softBody->m_nodes[scene.corner_grasp_point_inds[scene.corner_number_+1]].m_x ) );
                    break;
                }
*/

/*
                case 's':
                    scene.left_gripper2->toggle();
                    break;

                case 'z':
                    scene.left_gripper1->toggleattach( scene.clothPtr->softBody.get() );
                    break;

                case 'x':
                    scene.left_gripper2->toggleattach( scene.clothPtr->softBody.get() );
                    break;


                case 'c':
                    scene.regraspWithOneGripper( scene.left_gripper1,scene.left_gripper2 );
                    break;

                case 'v':
                    scene.regraspWithOneGripper( scene.right_gripper1,scene.right_gripper2 );
                    break;
*/
/*                case 'f':
                    // scene.regraspWithOneGripper( scene.right_gripper1,scene.left_gripper1 );
                    // scene.left_gripper1->setWorldTransform( btTransform( btQuaternion( 0,0,0,1 ), btVector3( 0,0,100 ) ) );
                    scene.testRelease( scene.left_gripper1 );
                    break;

                case 't':
                    scene.testRelease2( scene.left_gripper2 );
                    break;scene.inputState.startDragging = true;

                case 'y':
                    scene.testRegrasp2( scene.left_gripper2 );
                    break;

                case 'g':
                    // scene.regraspWithOneGripper( scene.right_gripper2,scene.left_gripper2 );
                    // scene.left_gripper2->setWorldTransform( btTransform( btQuaternion( 0,0,0,1 ), btVector3( 0,0,110 ) ) );
                    scene.testRegrasp( scene.left_gripper1 );
                    break;

                case 'k':
                    std::cout << "try to adjust first gripper" << std::endl;
                    scene.testAdjust( scene.left_gripper1 );
                    break;

                case 'l':
                    // std::cout << "try to adjust second gripper" << std::endl;
                    scene.switchTarget();
                    break;
*/
                // case 'k':
                //     scene.right_gripper1->setWorldTransform( btTransform( btQuaternion( btVector3( 1,0,0 ),-0.2 )*
                //         scene.right_gripper1->getWorldTransform().getRotation(),
                //         scene.right_gripper1->getWorldTransform().getOrigin() ) );
                //     break;

                // case ',':
                //     scene.right_gripper1->setWorldTransform( btTransform( btQuaternion( btVector3( 1,0,0 ),0.2 )*
                //         scene.right_gripper1->getWorldTransform().getRotation(),
                //         scene.right_gripper1->getWorldTransform().getOrigin() ) );
                //     break;


                // case 'l':
                //     scene.right_gripper2->setWorldTransform( btTransform( btQuaternion( btVector3( 1,0,0 ),0.2 )*
                //         scene.right_gripper2->getWorldTransform().getRotation(),
                //         scene.right_gripper2->getWorldTransform().getOrigin() ) );
                //     break;

                //case '.':
                //    scene.right_gripper2->setWorldTransform( btTransform( btQuaternion( btVector3( 1,0,0 ),-0.2 )*
                //        scene.right_gripper2->getWorldTransform().getRotation(),
                //        scene.right_gripper2->getWorldTransform().getOrigin() ) );
                //    break;


                // case 'y':
                //     scene.right_gripper1->setWorldTransform( btTransform( btQuaternion( btVector3( 0,1,0 ),-0.2 )*
                //         scene.right_gripper1->getWorldTransform().getRotation(),
                //         scene.right_gripper1->getWorldTransform().getOrigin() ) );
                //     break;


                //case 'u':
                //    scene.right_gripper2->setWorldTransform( btTransform( btQuaternion( btVector3( 0,1,0 ),-0.2 )*
                //        scene.right_gripper2->getWorldTransform().getRotation(),
                //        scene.right_gripper2->getWorldTransform().getOrigin() ) );
                //    break;

/*
                case 'i':
                    scene.left_gripper2->setWorldTransform( btTransform( btQuaternion( 0,0,0,1 ),
                        scene.clothPtr->softBody->m_nodes[scene.robot_mid_point_ind].m_x ) );
                    break;

                case 'o':
                    scene.right_gripper2->setWorldTransform( btTransform( btQuaternion( 0,0,0,1 ),
                        scene.clothPtr->softBody->m_nodes[scene.user_mid_point_ind].m_x ) );
                    break;


                case 'b':
                    if( scene.right_gripper2->bOpen )
                        scene.right_gripper2->state = GripperState_CLOSING;
                    else
                        scene.right_gripper2->state = GripperState_OPENING;

                    break;

                case 'n':
                    if( scene.left_gripper2->bOpen )
                        scene.left_gripper2->state = GripperState_CLOSING;
                    else
                        scene.left_gripper2->state = GripperState_OPENING;

                    break;

                case 'j':
                {
                    /// by bFirstTrackingIteration
                    scene.getDeformableObjectNodes( scene.prev_node_pos );
                    scene.bTracking = !scene.bTracking;
                    if( scene.bTracking )
                    {
                        scene.bFirstTrackingIteration = true;
                        scene.itrnumber = 0;
                    }
                    if( !scene.bTracking )
                    {
                        scene.plot_points->setPoints( std::vector<btVector3> (), std::vector<btVector4> () );
                    }

                    break;
                }
*/
            }
            break;
        }

        case osgGA::GUIEventAdapter::KEYUP:
        {
            switch ( ea.getKey() )
            {
                case '1':
                case 'q':
                case '2':
                case 'w':
                case '3':
                case 'e':
                case '4':
                case 'r':
                {
                    current_gripper_ = nullptr;
                    translate_gripper_ = false;
                    rotate_gripper_ = false;
                }
            }
            break;
        }

        case osgGA::GUIEventAdapter::PUSH:
        {
            start_dragging_ = true;
            break;
        }

        case osgGA::GUIEventAdapter::DRAG:
        {
            // drag the active manipulator in the plane of view
            if (  ( ea.getButtonMask() & ea.LEFT_MOUSE_BUTTON ) && current_gripper_ != nullptr )
            {
                // if we've just started moving, reset our internal position trackers
                if ( start_dragging_ )
                {
                    mouse_last_x_ = ea.getXnormalized();
                    mouse_last_y_ = ea.getYnormalized();
                    start_dragging_ = false;
                }
                float dx = mouse_last_x_ - ea.getXnormalized();
                float dy = ea.getYnormalized() - mouse_last_y_;

                mouse_last_x_ = ea.getXnormalized();
                mouse_last_y_ = ea.getYnormalized();

                // get our current view
                osg::Vec3d osgCenter, osgEye, osgUp;
                scene_.manip->getTransformation( osgCenter, osgEye, osgUp );
                btVector3 from( util::toBtVector( osgEye ) );
                btVector3 to( util::toBtVector( osgCenter ) );
                btVector3 up( util::toBtVector( osgUp ) ); up.normalize();

                // compute basis vectors for the plane of view
                // ( the plane normal to the ray from the camera to the center of the scene )
                btVector3 camera_normal_vector = ( to - from ).normalized();
                btVector3 camera_y_axis = ( up - ( up.dot( camera_normal_vector ) )*camera_normal_vector ).normalized(); //TODO: FIXME: is this necessary with osg?
                btVector3 camera_x_axis = camera_normal_vector.cross( camera_y_axis );
                btVector3 drag_vector = SceneConfig::mouseDragScale * ( dx*camera_x_axis + dy*camera_y_axis );

                btTransform current_transform = current_gripper_->getWorldTransform();
                btTransform next_transform( current_transform );

                if ( translate_gripper_ )
                {
                    // if moving the manip, just set the origin appropriately
                    next_transform.setOrigin( drag_vector + current_transform.getOrigin() );
                }
                else if ( rotate_gripper_ )
                {
                    // if we're rotating, the axis is perpendicular to the
                    // direction the mouse is dragging
                    btVector3 axis = camera_normal_vector.cross( drag_vector );
                    btScalar angle = drag_vector.length();
                    btQuaternion rot( axis, angle );
                    // we must ensure that we never get a bad rotation quaternion
                    // due to really small ( effectively zero ) mouse movements
                    // this is the easiest way to do this:
                    if ( rot.length() > 0.99f && rot.length() < 1.01f )
                    {
                        next_transform.setRotation( rot * current_transform.getRotation() );
                    }
                }

                // We don't need to lock here, as this is called inside of draw()
                // at which point we've already locked
                current_gripper_->setWorldTransform( next_transform );
                suppress_default_handler = true;
            }
            break;
        }

        default:
        {
            break;
        }
    }
    return suppress_default_handler;
}


GripperKinematicObject::Ptr CustomScene::CustomKeyHandler::getGripper( size_t gripper_num )
{
    if ( gripper_num < scene_.auto_grippers_.size() )
    {
        return scene_.grippers_[scene_.auto_grippers_[gripper_num]];
    }
    else if ( gripper_num - scene_.auto_grippers_.size() < scene_.manual_grippers_.size() )
    {
        gripper_num = gripper_num - scene_.auto_grippers_.size();
        return scene_.grippers_[scene_.manual_grippers_[gripper_num]];
    }
    else
    {
        std::cerr << "Invalid gripper number: " << gripper_num << std::endl;
        std::cerr << "Existing auto grippers:   " << PrettyPrint::PrettyPrint( scene_.auto_grippers_ ) << std::endl;
        std::cerr << "Existing manual grippers: " << PrettyPrint::PrettyPrint( scene_.manual_grippers_ ) << std::endl;
        return nullptr;
    }
}
