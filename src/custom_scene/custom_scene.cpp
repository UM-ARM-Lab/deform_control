#include "custom_scene.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor and Destructor
////////////////////////////////////////////////////////////////////////////////

CustomScene::CustomScene(
        CustomScene::DeformableType deformable_type,
        CustomScene::TaskType task_type )
    : deformable_type_( deformable_type )
    , task_type_( task_type )
{
    btVector4 color( 0.6,0.6,0.6,1 );
    btVector4 color2( 0, 0, 1, 1 );

    // Create all the grippers. makeClothWorld and makeRopeWorld will add them to
    // the environment if they are needed.
    grippers_["left_gripper1"] = GripperKinematicObject::Ptr( new GripperKinematicObject( color ) );
    grippers_["right_gripper1"] = GripperKinematicObject::Ptr( new GripperKinematicObject( color2 ) );
    grippers_["left_gripper2"] = GripperKinematicObject::Ptr( new GripperKinematicObject( color ) );
    grippers_["right_gripper2"] = GripperKinematicObject::Ptr( new GripperKinematicObject( color2 ) );

    switch ( deformable_type_ )
    {
        case ROPE:
            makeRopeWorld();
            break;

        case CLOTH:
//            makeClothWorld();
            break;

        default:
            throw new std::invalid_argument( "Unknown deformable object type" );
    };
}

////////////////////////////////////////////////////////////////////////////////
// Initialization helper functions
////////////////////////////////////////////////////////////////////////////////

void CustomScene::makeRopeWorld()
{
    // table parameters
    const float table_x = 0*METERS;
    const float table_y = 0*METERS;
    const float table_z = .7*METERS;
    const btVector3 table_surface_position = btVector3( table_x, table_y, table_z );

    const float table_width = 1.5*METERS;
    const float table_length = 1.5*METERS;
    const float table_thickness = .05*METERS;
    const btVector3 table_half_extents = btVector3( table_width, table_length, table_thickness )/2;

    // rope parameters
    const float rope_radius = .01*METERS;
    const float segment_len = .025*METERS;
    const int num_links = 50;

    // create the table
    table_ = BoxObject::Ptr( new BoxObject( 0, table_half_extents,
                btTransform( btQuaternion( 0, 0, 0, 1 ),
                    table_surface_position - btVector3( 0, 0, table_thickness/2 ) ) ) );

    // create the rope
    // TODO: remove this scope once it's clear that I don't need it anymore
    {
        std::vector<btVector3> control_points( num_links );
        for ( int n = 0; n < num_links; n++ )
        {
            control_points[n] = btVector3(
                    ((float)n - (float)(num_links - 1)/2)*segment_len,
                    table_y,
                    table_z + 5*rope_radius );
        }
        rope_.reset( new CapsuleRope( control_points, rope_radius ) );

        std::vector<BulletObject::Ptr> children = rope_->getChildren();
        for ( size_t j = 0; j < children.size(); j++ )
        {
            children[j]->setColor( 0.15, 0.65, 0.15, 1.0 );
        }
    }

    // add the table and rope to the world
    env->add( table_ );
    env->add( rope_ );

    // Here we assume that we are already working with a rope object
    switch ( task_type_ )
    {
        case COVERAGE:
        {
            // create a cylinder
            const float cylinder_radius = 0.15*METERS;
            const float cylinder_height = 0.3*METERS;
            const btVector3 cylinder_com_origin =
                table_surface_position + btVector3( 0, 0.25*METERS, cylinder_height/2 );

            cylinder_ = CylinderStaticObject::Ptr( new CylinderStaticObject(
                        0, cylinder_radius, cylinder_height,
                        btTransform( btQuaternion( 0, 0, 0, 1 ), cylinder_com_origin ) ) );
            cylinder_->setColor( 179.0/255.0, 176.0/255.0, 160.0/255.0, 1 );

            // add the cylinder to the world
            env->add( cylinder_ );

            // add a single auto gripper to the world
            grippers_["left_gripper1"]->setWorldTransform(
                    rope_->children[0]->rigidBody->getCenterOfMassTransform() );
            grippers_["left_gripper1"]->rigidGrab( rope_->children[0]->rigidBody.get(), 0, env );

            auto_grippers_.push_back( grippers_["left_gripper1"] );
            gripper_axes_["left_gripper1"] = PlotAxes::Ptr( new PlotAxes() );

            env->add( grippers_["left_gripper1"] );
            env->add( gripper_axes_["left_gripper1"] );

            // find the points that we want to cover with a rope
            for( float theta = 0; theta < 2 * M_PI; theta += 0.3 )
            {
                for( float h = 0; h < cylinder_height; h += 0.2 )
                {
                    cover_points_.push_back(
                            cylinder_com_origin
                            + btVector3( cylinder_radius*cos( theta ) + rope_radius/2,
                                         cylinder_radius*sin( theta ) + rope_radius/2,
                                         h - cylinder_height/2 ) );
                }
            }
            std::cout << "num cover points " << cover_points_.size() << std::endl;

            break;
        }
        default:
            throw new std::invalid_argument( "Unknown deformable object type" );
    }

    std::vector<btVector3> node_pos( rope_->getNodes() );
    std::cout << "rope node length " << ( node_pos[0]-node_pos[node_pos.size()-1] ).length() << std::endl;

}

////////////////////////////////////////////////////////////////////////////////
// Plotting helpers
////////////////////////////////////////////////////////////////////////////////

void CustomScene::initializePloting()
{
/*    plot_points.reset( new PlotPoints( 5 ) );
    env->add( plot_points );

    rot_lines.reset( new PlotLines( 2 ) );
    rot_lines->setPoints( std::vector<btVector3> (), std::vector<btVector4> () );
    env->add( rot_lines );
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
    initializePloting();

    //viewer.addEventHandler( new CustomKeyHandler( *this ) );

    //addPreStepCallback( std::bind( &CustomScene::getDesiredGripperTrajectory, this ) );
    //addPreStepCallback( std::bind( &CustomScene::doJTracking, this ) );
    addPreStepCallback( std::bind( &CustomScene::drawAxes, this ) );

    // if syncTime is set, the simulator blocks until the real time elapsed
    // matches the simulator time elapsed
    setSyncTime( syncTime );
    startViewer();
    stepFor( BulletConfig::dt, 2 );

    startFixedTimestepLoop( BulletConfig::dt );
}
