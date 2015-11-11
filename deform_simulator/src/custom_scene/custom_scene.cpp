#include "custom_scene/custom_scene.h"

#include <limits>
#include <string>
#include <boost/thread.hpp>
#include <ros/callback_queue.h>

#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <bullet_helpers/bullet_internal_conversions.hpp>
#include <bullet_helpers/bullet_ros_conversions.hpp>

#include "bullet_helpers/bullet_pretty_print.hpp"

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

CustomScene::CustomScene(CustomScene::DeformableType deformable_type,
        CustomScene::TaskType task_type,
        ros::NodeHandle& nh,
        const std::string& cmd_gripper_traj_topic,
        const std::string& simulator_fbk_topic,
        const std::string& get_gripper_names_topic,
        const std::string& get_gripper_attached_node_indices_topic,
        const std::string& get_gripper_pose_topic,
        const std::string& get_object_initial_configuration_topic,
        const std::string& get_cover_points_topic,
        const std::string& set_visualization_marker_topic )
    : plot_points_( new PlotPoints( 0.1*METERS ) )
    , plot_lines_( new PlotLines( 0.25*METERS ) )
    , deformable_type_( deformable_type )
    , task_type_( task_type )
    , nh_( nh )
{
    ROS_INFO_NAMED( "custom_scene", "Buiding the world" );
    // Build the world
    // TODO: make this setable/resetable via a ROS service call
    switch ( deformable_type_ )
    {
        case ROPE:
            makeRopeWorld();
            break;

        case CLOTH:
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

    ROS_INFO_NAMED( "custom_scene", "Creating subscribers and publishers" );
    // Publish to the feedback channel
    simulator_fbk_pub_ = nh_.advertise< deform_simulator::SimulatorFbkStamped >(
            simulator_fbk_topic, 20 );

    ROS_INFO_NAMED( "custom_scene", "Creating services" );
    // Create a service to let others know the internal gripper names
    gripper_names_srv_ = nh_.advertiseService(
            get_gripper_names_topic, &CustomScene::getGripperNamesCallback, this );

    // Create a service to let others know what nodes the grippers are attached too
    gripper_attached_node_indices_srv_ = nh_.advertiseService(
            get_gripper_attached_node_indices_topic, &CustomScene::getGripperAttachedNodeIndicesCallback, this );

    // Create a service to let others know the current gripper pose
    gripper_pose_srv_ = nh_.advertiseService(
            get_gripper_pose_topic, &CustomScene::getGripperPoseCallback, this);

    // Create a service to let others know the cover points
    cover_points_srv_ = nh_.advertiseService(
            get_cover_points_topic, &CustomScene::getCoverPointsCallback, this );

    // Create a service to let others know the object initial configuration
    object_initial_configuration_srv_ = nh_.advertiseService(
            get_object_initial_configuration_topic, &CustomScene::getObjectInitialConfigurationCallback, this );

    // Create a service to take gripper trajectories
    cmd_grippers_traj_srv_ = nh_.advertiseService(
            cmd_gripper_traj_topic, &CustomScene::cmdGripperTrajCallback, this );
    gripper_traj_index_ = 0;
    new_gripper_traj_ready_ = false;

    // Create a service to take visualization instructions
    set_visualization_marker_srv_ = nh_.advertiseService(
            set_visualization_marker_topic, &CustomScene::setVisualizationCallback, this );

    ROS_INFO_NAMED( "custom_scene", "Simulation ready." );
}

////////////////////////////////////////////////////////////////////////////////
// Main function that makes things happen
////////////////////////////////////////////////////////////////////////////////

void CustomScene::run( bool syncTime )
{
    //viewer.addEventHandler( new CustomKeyHandler( *this ) );

    // When the viewer closes, shutdown ROS
    addVoidCallback( osgGA::GUIEventAdapter::EventType::CLOSE_WINDOW,
            boost::bind( &ros::shutdown ) );

    addPreStepCallback( boost::bind( &CustomScene::drawAxes, this ) );

    // TODO: remove this hardcoded spin rate
    boost::thread spin_thread( boost::bind( &CustomScene::spin, 1000 ) );

    // if syncTime is set, the simulator blocks until the real time elapsed
    // matches the simulator time elapsed
    setSyncTime( syncTime );
    startViewer();

    // Run the simulation
    while ( ros::ok() )
    {
        // if we have some desired trajectories, execute them
        boost::mutex::scoped_lock lock( input_mtx_ );
        if ( new_gripper_traj_ready_ )
        {
            curr_gripper_traj_ = next_gripper_traj_;
            gripper_traj_index_ = 0;
            new_gripper_traj_ready_ = false;
        }
        lock.unlock();

        // If we have not reached the end of the current gripper trajectory, execute the next step
        if ( curr_gripper_traj_.trajectories.size() > 0 &&
             gripper_traj_index_ < curr_gripper_traj_.trajectories[0].pose.size() )
        {
            moveGrippers();

            step( BulletConfig::dt );

            publishSimulatorFbk();
        }
        else
        {
            // TODO: replace this with something that redraws/allows user input
            step( 0 );
        }
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
    // TODO why was this not set for the rope and only for the cloth?
    table_->rigidBody->setFriction(1);

    env->add( table_ );

    // if we are doing a table coverage task, create the table coverage points
    if ( set_cover_points )
    {
        const float stepsize = 0.0125*METERS;
        btTransform table_tf = table_->rigidBody->getCenterOfMassTransform();

        std::vector< btVector3 > cloth_coverage_lines;
        for(float y = -table_->halfExtents.y(); y < table_->halfExtents.y(); y += stepsize)
        {
            // Add a coverage line to the visualization
            cloth_coverage_lines.push_back(
                    table_tf * btVector3 ( -table_->halfExtents.x(), y, table_->halfExtents.z() ) );

            cloth_coverage_lines.push_back(
                    table_tf * btVector3 ( +table_->halfExtents.x(), y, table_->halfExtents.z() ) );

            // Add many coverage points along the coverage line
            for(float x = -table_->halfExtents.x(); x < table_->halfExtents.x(); x += stepsize)
            {
                cover_points_.push_back( table_tf * btVector3( x, y, table_->halfExtents.z() ) );
            }
        }
        std::cout << "Number of cover points: " << cover_points_.size() << std::endl;

        std::vector<btVector4> cloth_coverage_color( cloth_coverage_lines.size(), btVector4( 1, 0, 1, 1 ) );
        plot_lines_->setPoints( cloth_coverage_lines, cloth_coverage_color );
        env->add( plot_lines_ );
    }
}

void CustomScene::makeCylinder( const bool set_cover_points )
{
    // find the needed table parameters
    const btVector3 table_surface_position = btVector3( TABLE_X, TABLE_Y, TABLE_Z ) * METERS;

    // cylinder parameters
    const btVector3 cylinder_com_origin =
        table_surface_position +
        btVector3( 0, ROPE_CYLINDER_RADIUS*METERS*5/3, ROPE_CYLINDER_HEIGHT*METERS/2 );

    // create a cylinder
    cylinder_ = CylinderStaticObject::Ptr( new CylinderStaticObject(
                0, ROPE_CYLINDER_RADIUS*METERS, ROPE_CYLINDER_HEIGHT*METERS,
                btTransform( btQuaternion( 0, 0, 0, 1 ), cylinder_com_origin ) ) );
    cylinder_->setColor( 179.0/255.0, 176.0/255.0, 160.0/255.0, 0.25 );

    // add the cylinder to the world
    env->add( cylinder_ );

    if ( set_cover_points )
    {
        // find the points that we want to cover with a rope
        for( float theta = 0; theta < 2 * M_PI; theta += 0.3 )
        {
            for( float h = 0; h < ROPE_CYLINDER_HEIGHT*METERS; h += 0.2 )
            {
                cover_points_.push_back(
                        cylinder_com_origin
                        + btVector3( (ROPE_CYLINDER_RADIUS*METERS + rope_->radius/2)*cos( theta ),
                                     (ROPE_CYLINDER_RADIUS*METERS + rope_->radius/2)*sin( theta ),
                                     h - ROPE_CYLINDER_HEIGHT*METERS/2 ) );

            }
        }
        std::cout << "Number of cover points: " << cover_points_.size() << std::endl;

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
        control_points[n] = table_surface_position +
            btVector3( ((float)n - (float)(ROPE_NUM_LINKS - 1)/2)*ROPE_SEGMENT_LENGTH, 0, 5*ROPE_RADIUS ) * METERS;
    }
    rope_.reset( new CapsuleRope( control_points, ROPE_RADIUS*METERS ) );

    // color the rope
    std::vector< BulletObject::Ptr > children = rope_->getChildren();
    for ( size_t j = 0; j < children.size(); j++ )
    {
        children[j]->setColor( 0.15, 0.65, 0.15, 1.0 );
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
    psb->getCollisionShape()->setMargin( 0.05 );
    btSoftBody::Material *pm = psb->appendMaterial();
    // commented out as there are no self collisions
    //pm->m_kLST = 0.2;//0.1; //makes it rubbery (handles self collisions better)
    psb->m_cfg.kDP = 0.05;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    // TODO why are these both being called?
    psb->setTotalMass(1, true);
    psb->setTotalMass(0.1);
    psb->generateClusters(0);

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
    cloth_->setColor( 0.15, 0.65, 0.15, 1.0 );

    findClothCornerNodes();
}

void CustomScene::makeRopeWorld()
{
    // Here we assume that we are already working with a rope object
    switch ( task_type_ )
    {
        case COVERAGE:
        {
            const bool table_set_cover_points = false;
            const bool cylinder_set_cover_points = true;
            makeTable( ROPE_TABLE_HALF_SIDE_LENGTH*METERS, table_set_cover_points );
            makeRope();
            makeCylinder( cylinder_set_cover_points );

            // add a single auto gripper to the world
            grippers_["gripper"] = GripperKinematicObject::Ptr(
                    new GripperKinematicObject( ROPE_GRIPPER_APPERTURE*METERS ) );
            grippers_["gripper"]->setWorldTransform(
                    rope_->children[0]->rigidBody->getCenterOfMassTransform() );
            grippers_["gripper"]->rigidGrab( rope_->children[0]->rigidBody.get(), 0, env );

            auto_grippers_.push_back( grippers_["gripper"] );
            gripper_axes_["gripper"] = PlotAxes::Ptr( new PlotAxes() );

            env->add( grippers_["gripper"] );
            env->add( gripper_axes_["gripper"] );

            break;
        }
        default:
        {
            throw new std::invalid_argument( "Unknown task type for a ROPE object" );
        }
    }

    std::vector<btVector3> node_pos( rope_->getNodes() );
}

void CustomScene::makeClothWorld()
{
    switch ( task_type_ )
    {
        case COVERAGE:
        {
            const bool set_cover_points = true;
            makeTable( CLOTH_TABLE_HALF_SIDE_LENGTH*METERS, set_cover_points );
            makeCloth();

            /*
             * add 2 auto grippers to the world
             */

            // auto gripper0
            grippers_["auto_gripper0"] = GripperKinematicObject::Ptr(
                    new GripperKinematicObject( CLOTH_GRIPPER_APPERTURE*METERS ) );
            grippers_["auto_gripper0"]->setWorldTransform(
                    btTransform( btQuaternion( 0, 0, 0, 1 ),
                    cloth_->softBody->m_nodes[cloth_corner_node_indices_[0]].m_x ) );
            grippers_["auto_gripper0"]->toggleOpen();
            grippers_["auto_gripper0"]->toggleAttach( cloth_->softBody.get() );

            auto_grippers_.push_back( grippers_["auto_gripper0"] );
            gripper_axes_["auto_gripper0"] = PlotAxes::Ptr( new PlotAxes() );

            env->add( grippers_["auto_gripper0"] );
            env->add( gripper_axes_["auto_gripper0"] );

            // auto gripper1
            grippers_["auto_gripper1"] = GripperKinematicObject::Ptr(
                    new GripperKinematicObject( CLOTH_GRIPPER_APPERTURE*METERS ) );
            grippers_["auto_gripper1"]->setWorldTransform(
                    btTransform( btQuaternion( 0, 0, 0, 1 ),
                    cloth_->softBody->m_nodes[cloth_corner_node_indices_[1]].m_x ) );
            grippers_["auto_gripper1"]->toggleOpen();
            grippers_["auto_gripper1"]->toggleAttach( cloth_->softBody.get() );

            auto_grippers_.push_back( grippers_["auto_gripper1"] );
            gripper_axes_["auto_gripper1"] = PlotAxes::Ptr( new PlotAxes() );

            env->add( grippers_["auto_gripper1"] );
            env->add( gripper_axes_["auto_gripper1"] );

            break;
        }
        case COLAB_FOLDING:
        {
            makeCloth();

            /*
             * add 2 auto grippers to the world
             */

            // auto gripper0
            grippers_["auto_gripper0"] = GripperKinematicObject::Ptr(
                    new GripperKinematicObject( CLOTH_GRIPPER_APPERTURE*METERS ) );
            grippers_["auto_gripper0"]->setWorldTransform(
                    btTransform( btQuaternion( 0, 0, 0, 1 ),
                    cloth_->softBody->m_nodes[cloth_corner_node_indices_[0]].m_x ) );
            grippers_["auto_gripper0"]->toggleOpen();
            grippers_["auto_gripper0"]->toggleAttach( cloth_->softBody.get() );

            auto_grippers_.push_back( grippers_["auto_gripper0"] );
            gripper_axes_["auto_gripper0"] = PlotAxes::Ptr( new PlotAxes() );

            env->add( grippers_["auto_gripper0"] );
            env->add( gripper_axes_["auto_gripper0"] );

            // auto gripper1
            grippers_["auto_gripper1"] = GripperKinematicObject::Ptr(
                    new GripperKinematicObject( CLOTH_GRIPPER_APPERTURE*METERS ) );
            grippers_["auto_gripper1"]->setWorldTransform(
                    btTransform( btQuaternion( 0, 0, 0, 1 ),
                    cloth_->softBody->m_nodes[cloth_corner_node_indices_[1]].m_x ) );
            grippers_["auto_gripper1"]->toggleOpen();
            grippers_["auto_gripper1"]->toggleAttach( cloth_->softBody.get() );

            auto_grippers_.push_back( grippers_["auto_gripper1"] );
            gripper_axes_["auto_gripper1"] = PlotAxes::Ptr( new PlotAxes() );

            env->add( grippers_["auto_gripper1"] );
            env->add( gripper_axes_["auto_gripper1"] );

            break;
        }
        default:
        {
            throw new std::invalid_argument( "Unknown task type for a CLOTH object" );
        }
    }
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
    corner_node_positions[0] = btVector3(
            std::numeric_limits< btScalar >::infinity(),
            std::numeric_limits< btScalar >::infinity(),
            0 );
    corner_node_positions[1] = btVector3(
            std::numeric_limits< btScalar >::infinity(),
            -std::numeric_limits< btScalar >::infinity(),
            0 );
    corner_node_positions[2] = btVector3(
            -std::numeric_limits< btScalar >::infinity(),
            std::numeric_limits< btScalar >::infinity(),
            0 );
    corner_node_positions[3] = btVector3(
            -std::numeric_limits< btScalar >::infinity(),
            -std::numeric_limits< btScalar >::infinity(),
            0 );

    btSoftBody::tNodeArray cloth_nodes = cloth_->softBody->m_nodes;

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

    std::cout << "Cloth corner node positions\n";
    for ( int ind: cloth_corner_node_indices_ )
    {
        std::cout
            << "\tx: " << cloth_nodes[ind].m_x.x()/METERS << " "
            << "\ty: " << cloth_nodes[ind].m_x.y()/METERS << " "
            << "\tz: " << cloth_nodes[ind].m_x.z()/METERS << " "
            << std::endl;
    }
}

////////////////////////////////////////////////////////////////////////////////
// Main loop helper functions
////////////////////////////////////////////////////////////////////////////////

void CustomScene::moveGrippers()
{
    // TODO check for valid gripper names (and length of names vector)
    for ( size_t ind = 0; ind < curr_gripper_traj_.gripper_names.size(); ind++ )
    {
        btTransform tf = toBulletTransform(
                    curr_gripper_traj_.trajectories[ind].pose[gripper_traj_index_], METERS );

        grippers_[curr_gripper_traj_.gripper_names[ind]]->setWorldTransform( tf );
    }
    gripper_traj_index_++;
}

void CustomScene::publishSimulatorFbk()
{
    // TODO: don't rebuild this every time
    deform_simulator::SimulatorFbkStamped msg;

    // fill out the gripper data
    for ( auto &gripper: grippers_ )
    {
        msg.gripper_names.push_back( gripper.first );
        msg.gripper_poses.push_back( toRosPose( gripper.second->getWorldTransform(), METERS ) );
    }

    // fill out the object configuration data
    msg.object_configuration = toRosPointVector( getDeformableObjectNodes(), METERS );

    // update the sim_time
    // TODO: is this actually correct?
    msg.sim_time = viewer.getFrameStamp()->getSimulationTime();

    // publish the message
    msg.header.stamp = ros::Time::now();
    simulator_fbk_pub_.publish( msg );
}

////////////////////////////////////////////////////////////////////////////////
// Internal helper functions
////////////////////////////////////////////////////////////////////////////////

std::vector< btVector3 > CustomScene::getDeformableObjectNodes()
{
    std::vector< btVector3 > nodes;

    switch ( deformable_type_ )
    {
        case ROPE:
            nodes = rope_->getNodes();
            break;

        case CLOTH:
            nodes = nodeArrayToNodePosVector( cloth_->softBody->m_nodes );
            break;
    }

    return nodes;
}

////////////////////////////////////////////////////////////////////////////////
// ROS Callbacks
////////////////////////////////////////////////////////////////////////////////

bool CustomScene::cmdGripperTrajCallback(
        deform_simulator::CmdGrippersTrajectory::Request& req,
        deform_simulator::CmdGrippersTrajectory::Response& res)
{
    (void)res;
    boost::mutex::scoped_lock lock( input_mtx_ );

    assert( req.trajectories.size() == req.gripper_names.size() );

    next_gripper_traj_ = req;
    new_gripper_traj_ready_ = true;

    return true;
}

bool CustomScene::getGripperNamesCallback(
        deform_simulator::GetGripperNames::Request& req,
        deform_simulator::GetGripperNames::Response& res )
{
    (void)req;
    for ( auto& gripper: grippers_ )
    {
        res.names.push_back( gripper.first );
    }
    return true;
}

bool CustomScene::getGripperAttachedNodeIndicesCallback(
        deform_simulator::GetGripperAttachedNodeIndices::Request& req,
        deform_simulator::GetGripperAttachedNodeIndices::Response& res )
{
    // TODO: error check input
    GripperKinematicObject::Ptr gripper = grippers_[req.name];
    res.indices = gripper->getAttachedNodeIndices();
    return true;
}

bool CustomScene::getGripperPoseCallback(
        deform_simulator::GetGripperPose::Request& req,
        deform_simulator::GetGripperPose::Response& res )
{
    // TODO: error check input
    GripperKinematicObject::Ptr gripper = grippers_[req.name];
    res.pose = toRosPose( gripper->getWorldTransform(), METERS );
    return true;
}

bool CustomScene::getCoverPointsCallback(
        deform_simulator::GetPointSet::Request& req,
        deform_simulator::GetPointSet::Response& res )
{
    (void)req;
    res.points = toRosPointVector( cover_points_, METERS );
    return true;
}

bool CustomScene::getObjectInitialConfigurationCallback(
        deform_simulator::GetPointSet::Request& req,
        deform_simulator::GetPointSet::Response& res )
{
    (void)req;
    res.points = object_initial_configuration_;
    return true;
}

bool CustomScene::setVisualizationCallback(
        deform_simulator::SetVisualizationMarker::Request& req,
        deform_simulator::SetVisualizationMarker::Response& res )
{
    // TODO: be able to delete markers

    //std::map< std::string, std::vector< EnvironmentObject::Ptr > > visualizaton_markers_;
    (void)res;

    std::string id = req.marker.ns + std::to_string( req.marker.id );

    switch ( req.marker.type )
    {
        case visualization_msgs::Marker::SPHERE:
        {
            if ( visualization_sphere_markers_.count(id) == 0 )
            {
                ROS_INFO_STREAM( "Creating new Marker::SPHERE" );
                PlotSpheres::Ptr spheres( new PlotSpheres() );
                spheres->plot( toOsgRefVec3Array( req.marker.points, METERS ),
                               toOsgRefVec4Array( req.marker.colors ),
                               std::vector< float >( req.marker.points.size(), req.marker.scale.x * METERS ) );
                visualization_sphere_markers_[id] = spheres;

                env->add( spheres );
            }
            else
            {
                PlotSpheres::Ptr spheres = visualization_sphere_markers_[id];
                spheres->plot( toOsgRefVec3Array( req.marker.points, METERS ),
                               toOsgRefVec4Array( req.marker.colors ),
                               std::vector< float >( req.marker.points.size(), req.marker.scale.x * METERS ) );
            }
            break;
        }
        case visualization_msgs::Marker::LINE_STRIP:
        {
            convertLineStripToLineList( req.marker );
        }
        case visualization_msgs::Marker::LINE_LIST:
        {
            // if the object is new, add it
            if ( visualization_line_markers_.count( id ) == 0 )
            {
                ROS_INFO_STREAM( "Creating new Marker::LINE_LIST" );
                PlotLines::Ptr line_strip( new PlotLines( req.marker.scale.x * METERS ) );
                line_strip->setPoints( toBulletPointVector( req.marker.points, METERS ),
                                       toBulletColorArray( req.marker.colors ));
                visualization_line_markers_[id] = line_strip;

                env->add( line_strip );
            }
            else
            {
                PlotLines::Ptr line_strip = visualization_line_markers_[id];
                line_strip->setPoints( toBulletPointVector( req.marker.points, METERS ),
                                       toBulletColorArray( req.marker.colors ));
            }
            break;
        }
        default:
        {
            ROS_ERROR_STREAM_NAMED( "custom_scene_visualization",
                    "Marker type " << req.marker.type << " not implemented" );
            return false;
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////
// ROS Objects and Helpers
////////////////////////////////////////////////////////////////////////////////

void CustomScene::spin( double loop_rate )
{
    ros::NodeHandle ph("~");
    ROS_INFO_NAMED( "custom_scene" , "Starting feedback spinner" );
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
