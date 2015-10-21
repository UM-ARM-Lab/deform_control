#include "custom_scene.h"

#include <limits>
#include <thread>

#include <BulletSoftBody/btSoftBodyHelpers.h>

////////////////////////////////////////////////////////////////////////////////
// Constructor and Destructor
////////////////////////////////////////////////////////////////////////////////

CustomScene::CustomScene(
        CustomScene::DeformableType deformable_type,
        CustomScene::TaskType task_type,
        ros::NodeHandle& nh )
    : plot_points_( new PlotPoints( 0.1*METERS ) )
    , plot_lines_( new PlotLines( 0.25*METERS ) )
    , deformable_type_( deformable_type )
    , task_type_( task_type )
    , nh_( nh )
{
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
}

////////////////////////////////////////////////////////////////////////////////
// Initialization helper functions
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
    cylinder_->setColor( 179.0/255.0, 176.0/255.0, 160.0/255.0, 1 );

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
        cloth_center + btVector3( -CLOTH_HALF_SIDE_LENGTH, -CLOTH_HALF_SIDE_LENGTH, 0)*METERS,
        cloth_center + btVector3( +CLOTH_HALF_SIDE_LENGTH, -CLOTH_HALF_SIDE_LENGTH, 0)*METERS,
        cloth_center + btVector3( -CLOTH_HALF_SIDE_LENGTH, +CLOTH_HALF_SIDE_LENGTH, 0)*METERS,
        cloth_center + btVector3( +CLOTH_HALF_SIDE_LENGTH, +CLOTH_HALF_SIDE_LENGTH, 0)*METERS,
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
// Plotting helpers
////////////////////////////////////////////////////////////////////////////////

void CustomScene::initializePloting()
{
    /*
    plot_points_.reset( new PlotPoints( 5 ) );
    env->add( plot_points_ );

    plot_lines_.reset( new PlotLines( 2 ) );
    plot_lines_->setPoints( std::vector<btVector3> (), std::vector<btVector4> () );
    env->add( plot_lines_ );
    */
}

void CustomScene::drawAxes()
{
    for ( auto& axes: gripper_axes_ )
    {
        axes.second->setup( grippers_[axes.first]->getWorldTransform(), 1 );
    }
}

////////////////////////////////////////////////////////////////////////////////
// Main function that makes things happen
////////////////////////////////////////////////////////////////////////////////
void CustomScene::run( bool syncTime )
{
    //initializePloting();

    //viewer.addEventHandler( new CustomKeyHandler( *this ) );

    // When the viewer closes, shutdown ROS
    addVoidCallback( osgGA::GUIEventAdapter::EventType::CLOSE_WINDOW,
            boost::bind( ros::shutdown ) );

    //addPreStepCallback( std::bind( &CustomScene::getDesiredGripperTrajectory, this ) );
    //addPreStepCallback( std::bind( &CustomScene::doJTracking, this ) );
    addPreStepCallback( std::bind( &CustomScene::drawAxes, this ) );

    // if syncTime is set, the simulator blocks until the real time elapsed
    // matches the simulator time elapsed
    setSyncTime( syncTime );
    startViewer();
    stepFor( BulletConfig::dt, 2 );

    // Start listening for ROS messages
    std::thread spin_thread( boost::bind( ros::spin ) );

    // Run the simulation
    startFixedTimestepLoop( BulletConfig::dt );

    // clean up the extra thread we started
    spin_thread.join();
}