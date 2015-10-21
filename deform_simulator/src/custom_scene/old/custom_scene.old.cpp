#include "custom_scene.h"
#include "custom_key_handler.h"
#include "vector_field.h"

#include "utils/conversions.h"
#include "utils/helper_functions.h"

#include <functional>
#include <limits>
#include <stdexcept>

#include <BulletSoftBody/btSoftBodyHelpers.h>

CustomScene::CustomScene(
        CustomScene::DeformableType deformable_type,
        CustomScene::TaskType task_type )
    : corner_number_( 0 )
    , deformable_type_( deformable_type )
    , task_type_( task_type )
{
    inputState.transGrabber0 = inputState.rotateGrabber0 =
            inputState.transGrabber1 = inputState.rotateGrabber1 =
            inputState.transGrabber2 = inputState.rotateGrabber2 =
            inputState.transGrabber3 = inputState.rotateGrabber3 =
            inputState.startDragging = false;

    btVector4 color( 0.6,0.6,0.6,1 );
    btVector4 color2( 0, 0, 1, 1 );

    left_gripper1.reset( new GripperKinematicObject( color2 ) );
    left_gripper1->setWorldTransform( btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0, -10, 0 ) ) );
    env->add( left_gripper1 );

    right_gripper1.reset( new GripperKinematicObject( color ) );
    right_gripper1->setWorldTransform( btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0, 10, 0 ) ) );
    env->add( right_gripper1 );

    left_gripper2.reset( new GripperKinematicObject( color2 ) );
    left_gripper2->setWorldTransform( btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0, 20, 0 ) ) );
    env->add( left_gripper2 );

    right_gripper2.reset( new GripperKinematicObject( color ) );
    right_gripper2->setWorldTransform( btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0, -20, 0 ) ) );
    env->add( right_gripper2 );

    num_auto_grippers = 2;

    switch ( deformable_type_ )
    {
        case ROPE:
            makeRopeWorld();
            break;

        case CLOTH:
            makeClothWorld();
            left_gripper1->apperture = 0.2;
            left_gripper2->apperture = 0.2;
            right_gripper1->apperture = 0.2;
            right_gripper2->apperture = 0.2;

            left_gripper1->toggle();
            left_gripper2->toggle();
            break;

        default:
            throw new std::invalid_argument( "Unknown deformable object type" );
    };
}

void CustomScene::getDeformableObjectNodes( std::vector<btVector3>& vnodes )
{
    switch ( deformable_type_ )
    {
        case ROPE:
            vnodes = rope_ptr->getNodes();
            break;

        case CLOTH:
            nodeArrayToNodePosVector( clothPtr->softBody->m_nodes, vnodes );
            break;

        default:
            throw new std::invalid_argument( "Unknown deformable object type" );
    }
}

int CustomScene::getNumDeformableObjectNodes()
{
    switch ( deformable_type_ )
    {
        case ROPE:
            return rope_ptr->getNodes().size();
            break;

        case CLOTH:
            return clothPtr->softBody->m_nodes.size();
            break;

        default:
            throw new std::invalid_argument( "Unknown deformable object type" );
    }
}

void CustomScene::regraspWithOneGripper( GripperKinematicObject::Ptr gripper_to_attach, GripperKinematicObject::Ptr gripper_to_detach )
{
    gripper_to_attach->toggleattach( clothPtr->softBody.get() );
    gripper_to_detach->toggleattach( clothPtr->softBody.get() );
    gripper_to_detach->toggle();

    float apperture = gripper_to_attach->apperture;
    gripper_to_attach->apperture = 0.1;
    gripper_to_attach->toggle();
    gripper_to_attach->apperture = apperture;

    gripper_to_attach->toggleattach( clothPtr->softBody.get() );
    gripper_to_attach->toggle();

    //gripper_to_detach->setWorldTransform( btTransform( btQuaternion( 0,0,0,1 ),btVector3( 100,5,0 ) ) );
}

void CustomScene::drawAxes()
{
    if( !!left_axes1 )
        left_axes1->setup( left_gripper1->getWorldTransform(),1 );
    if( !!left_axes2 )
        left_axes2->setup( left_gripper2->getWorldTransform(),1 );
}

void CustomScene::computeDeformableObjectDistanceMatrix( const std::vector<btVector3>& node_pos, Eigen::MatrixXf& distance_matrix )
{
    distance_matrix = Eigen::MatrixXf( node_pos.size(), node_pos.size() );
    for( size_t i = 0; i < node_pos.size(); i++ )
    {
        for( size_t j = i; j < node_pos.size(); j++ )
        {
            distance_matrix( i,j ) = ( node_pos[i]-node_pos[j] ).length();
            distance_matrix( j,i ) = distance_matrix( i,j );
        }
    }
}

void CustomScene::initializePloting()
{
    plot_points.reset( new PlotPoints( 5 ) );
    env->add( plot_points );

    rot_lines.reset( new PlotLines( 2 ) );
    rot_lines->setPoints( std::vector<btVector3> (), std::vector<btVector4> () );
    env->add( rot_lines );
}

void CustomScene::makeRopeWorld()
{
    float rope_radius = .01;
    float segment_len = .025;
    const float table_height = .7;
    const float table_thickness = .05;
    int nLinks = 50;

    vector<btVector3> ctrlPts;
    for ( int i=0; i< nLinks; i++ )
    {
        ctrlPts.push_back( METERS*btVector3( .5 + segment_len*i, 0, table_height + 5*rope_radius ) );
    }

    table = BoxObject::Ptr( new BoxObject( 0, METERS*btVector3( .75, .75, table_thickness/2 ),
                btTransform( btQuaternion( 0, 0, 0, 1 ), METERS*btVector3( 1, 0, table_height-table_thickness/2 ) ) ) );

    rope_ptr.reset( new CapsuleRope( ctrlPts,.01*METERS ) );

    env->add( rope_ptr );
    env->add( table );

    vector<BulletObject::Ptr> children =  rope_ptr->getChildren();
    for ( size_t j=0; j<children.size(); j++ )
    {
      //children[j]->setColor( 1,0,0,1 );
        children[j]->setColor( 0.15,0.65,0.15,1.0 );
    }

    // Here we assume that we are already working with a rope object
    switch ( task_type_ )
    {
        case COVERAGE:
        {
            //cylinder
            float radius = 3;
            float height = 6;
            num_auto_grippers = 1;
            cylinder = CylinderStaticObject::Ptr( new CylinderStaticObject( 0,
                        radius, height,
                        btTransform( btQuaternion( 0, 0, 0, 1 ),
                            table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3( 0, 5, height/2 ) ) ) );
            cylinder->setColor( 179.0/255.0,176.0/255.0,160.0/255.0,1 );
            env->add( cylinder );

            for( float theta = 0; theta < 2*3.1415; theta += 0.3 )
            {
                for( float h = 0; h < height; h += 0.2 )
                    cover_points.push_back( cylinder->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3( radius*cos( theta )+.01*METERS/2,radius*sin( theta )+.01*METERS/2,h-height/2 ) );
            }
            cout << "num cover points " << cover_points.size() << endl;

            left_gripper2-> setWorldTransform( btTransform( btQuaternion( 0,0,0,1 ), btVector3( 0,0, 100 ) ) );
            right_gripper1->setWorldTransform( btTransform( btQuaternion( 0,0,0,1 ), btVector3( 0,0, 100 ) ) );
            right_gripper2->setWorldTransform( btTransform( btQuaternion( 0,0,0,1 ), btVector3( 0,0, 100 ) ) );
            break;
        }
        default:
            throw new std::invalid_argument( "Unknown deformable object type" );
    }

    std::vector<int> gripper_closestobjectnodeind( num_auto_grippers );

    std::vector<btVector3> node_pos( rope_ptr->getNodes() );

    gripper_node_distance_map.resize( num_auto_grippers );
    GripperKinematicObject::Ptr cur_gripper;
    int childindex = 0;
    for( int i = 0; i < num_auto_grippers; i++ )
    {

        if( i == 0 )
        {
            childindex = 0;
            gripper_closestobjectnodeind[i] = 0;
            cur_gripper = left_gripper1;
        }
        if( i == 1 )
        {
            childindex = children.size()-1;
            gripper_closestobjectnodeind[i] = children.size()-1;
            cur_gripper = left_gripper2;
        }

        cur_gripper->setWorldTransform( rope_ptr->children[childindex]->rigidBody->getCenterOfMassTransform() );
        cur_gripper->rigidGrab( rope_ptr->children[childindex]->rigidBody.get(),gripper_closestobjectnodeind[i],env );



        gripper_node_distance_map[i].resize( node_pos.size() );

        for( size_t j = 0; j < node_pos.size(); j++ )
        {
            gripper_node_distance_map[i][j] = ( node_pos[gripper_closestobjectnodeind[i]]-node_pos[j] ).length();
        }
    }

    computeDeformableObjectDistanceMatrix( node_pos,deformableobject_distance_matrix );

    cout << "rope node length " << ( node_pos[0]-node_pos[node_pos.size()-1] ).length() << endl;


    left_axes1.reset( new PlotAxes() );
    env->add( left_axes1 );
}

void CustomScene::makeClothWorld()
{
#ifdef DO_COVERAGE
    const float table_height = .7;
    const float table_thickness = .05;
    num_auto_grippers = 2;

    btTransform Tm_table( btQuaternion( 0, 0, 0, 1 ), METERS*btVector3( 0.5, 0, table_height-table_thickness/2 ) );
    table = BoxObject::Ptr( new BoxObject( 0, METERS*btVector3( .2, .2, table_thickness/2 ), Tm_table ) );
    table->rigidBody->setFriction( 1 );
    env->add( table );

    float stepsize = 0.25;
    for( float x = -table->halfExtents[0]; x < table->halfExtents[0]; x+=stepsize )
    {
        for( float y = -table->halfExtents[1]; y < table->halfExtents[1]; y+=stepsize )
        {
            cover_points.push_back( Tm_table*btVector3( x,y,table->halfExtents[2] ) );
        }
    }
    cout << "num cover points " << cover_points.size() << endl;
#endif

    BulletSoftObject::Ptr cloth(
            createCloth( METERS*0.25, METERS*btVector3( 0.7, 0, table_height+0.01 ) ) );

    btSoftBody* psb = cloth->softBody.get();
    clothPtr = cloth;
    psb->setTotalMass( 0.1 );

    addPreStepCallback( std::bind( &GripperKinematicObject::step_openclose, this->right_gripper2,psb ) );
    addPreStepCallback( std::bind( &GripperKinematicObject::step_openclose, this->left_gripper2,psb ) );

    //table->setColor( 0.8,0.2,0.2,1.0 );

    env->add( cloth );
    cloth->setColor( 0.15,0.65,0.15,1.0 );
    //left_mover.reset( new RigidMover( table, table->rigidBody->getCenterOfMassPosition(), env->bullet->dynamicsWorld ) );

    //btVector3 pos( 0,0,0 );
    //grab_left.reset( new Grab( psb, &psb->m_nodes[0], left_grabber->rigidBody.get() ) );
    //grab_left.reset( new Grab( psb, 0, left_grabber->rigidBody.get() ) );

    //psb->m_cfg.kAHR = 1;
    //psb->appendAnchor( 0,left_grabber->rigidBody.get() );
    //psb->appendAnchor( 1,left_grabber->rigidBody.get() );
    //psb->appendAnchor( 2,left_grabber->rigidBody.get() );


    double min_x = 100;
    double max_x = -100;
    double min_y = 100;
    double max_y = -100;

    //std::vector<float> node_x( psb->m_nodes.size() );
    //std::vector<float> node_y( psb->m_nodes.size() );
    std::vector<btVector3> node_pos( psb->m_nodes.size() );
    corner_ind = std::vector<int>( 4,-1 );
    std::vector<btVector3> corner_pnts( 4 );

    corner_pnts[0] = btVector3( 100,100,0 );
    corner_pnts[1] = btVector3( 100,-100,0 );
    corner_pnts[2] = btVector3( -100,100,0 );
    corner_pnts[3] = btVector3( -100,-100,0 );

    for( int i = 0; i < psb->m_nodes.size(); i++ )
    {
        node_pos[i] = psb->m_nodes[i].m_x;

        if( node_pos[i][0] <= corner_pnts[0][0] && node_pos[i][1] <= corner_pnts[0][1] )
        {
            corner_ind[0] = i;
            corner_pnts[0] = node_pos[i];
        }

        if( node_pos[i][0] <= corner_pnts[1][0] && node_pos[i][1] >= corner_pnts[1][1] )
        {
            corner_ind[1] = i;
            corner_pnts[1] = node_pos[i];
        }

        if( node_pos[i][0] >= corner_pnts[2][0] && node_pos[i][1] <= corner_pnts[2][1] )
        {
            corner_ind[2] = i;
            corner_pnts[2] = node_pos[i];
        }

        if( node_pos[i][0] >= corner_pnts[3][0] && node_pos[i][1] >= corner_pnts[3][1] )
        {
            corner_ind[3] = i;
            corner_pnts[3] = node_pos[i];
        }

    }

    max_x = corner_pnts[3][0];
    max_y = corner_pnts[3][1];
    min_x = corner_pnts[0][0];
    min_y = corner_pnts[0][1];


    //btTransform tm_left = btTransform( btQuaternion( 0,    0.9877,    0.1564 ,   0 ), psb->m_nodes[min_x_ind].m_x+btVector3( 0,0,2 ) );
    //btTransform tm_right = btTransform( btQuaternion( 0,    0.9877,    0.1564 ,   0 ), psb->m_nodes[max_x_ind].m_x+btVector3( 0,0,2 ) );
    //left_grabber->motionState->setKinematicPos( tm_left );
    //right_grabber->motionState->setKinematicPos( tm_right );

    //psb->appendAnchor( min_x_ind,left_grabber->rigidBody.get() );
    //psb->appendAnchor( max_x_ind,right_grabber->rigidBody.get() );


    btTransform tm_left1 = btTransform( btQuaternion( 0,    0,    0 ,   1 ), corner_pnts[0] +
        btVector3( left_gripper1->children[0]->halfExtents[0],left_gripper1->children[0]->halfExtents[1],0 ) );
    left_gripper1->setWorldTransform( tm_left1 );
    //left_gripper1->toggle();


    btTransform tm_right1 = btTransform( btQuaternion( 0,    0,    0 ,   1 ), corner_pnts[2] +
        btVector3( -right_gripper1->children[0]->halfExtents[0],right_gripper1->children[0]->halfExtents[1],0 ) );
    right_gripper1->setWorldTransform( tm_right1 );

    btTransform tm_left2 = btTransform( btQuaternion( 0,    0,    0 ,   1 ), corner_pnts[1] +
        btVector3( left_gripper2->children[0]->halfExtents[0],-left_gripper2->children[0]->halfExtents[1],0 ) );
    left_gripper2->setWorldTransform( tm_left2 );
    //left_gripper1->toggle();


    btTransform tm_right2 = btTransform( btQuaternion( 0,    0,    0 ,   1 ), corner_pnts[3] +
        btVector3( -right_gripper2->children[0]->halfExtents[0],-right_gripper2->children[0]->halfExtents[1],0 ) );
    right_gripper2->setWorldTransform( tm_right2 );

    gripper_node_distance_map.resize( num_auto_grippers );

    for( int i = 0; i < num_auto_grippers; i++ )
    {
        gripper_node_distance_map[i].resize( node_pos.size() );

        for( size_t j = 0; j < node_pos.size(); j++ )
        {
            gripper_node_distance_map[i][j] = ( corner_pnts[i]-node_pos[j] ).length();
        }
    }

#ifdef DO_COVERAGE
//    right_gripper1->setWorldTransform( tm_left2 );
//    left_gripper2->setWorldTransform( tm_right1 );

    corner_grasp_point_inds.resize( 4 );

    for( int i = 0; i < 4; i++ )
    {

        btVector3 tm_origin;
        float closest_dist = 100000;
        corner_grasp_point_inds[i] = -1;
        if( i == 0 )
            tm_origin = tm_left1.getOrigin();
        else if( i ==1 )
            tm_origin = tm_right1.getOrigin();
        else if( i ==2 )
            tm_origin = tm_left2.getOrigin();
        else
            tm_origin = tm_right2.getOrigin();

        for( size_t j = 0; j < node_pos.size(); j++ )
        {
            float dist = ( node_pos[j]-tm_origin ).length();
            if( dist < closest_dist )
            {
                closest_dist = dist;
                corner_grasp_point_inds[i] = j;
            }
        }
    }
    //make an intuitive order
    int temp = corner_grasp_point_inds[1];
    corner_grasp_point_inds[1] = corner_grasp_point_inds[2];
    corner_grasp_point_inds[2] = temp;
#endif


    computeDeformableObjectDistanceMatrix( node_pos,deformableobject_distance_matrix );
    //cout << deformableobject_distance_matrix<< endl;

    //mirror about centerline along y direction;
    //centerline defined by 2 points
    float mid_x = ( max_x + min_x )/2;



    //plotting

    std::vector<btVector3> plotpoints;
    std::vector<btVector4> plotcols;
    plotpoints.push_back( btVector3( mid_x,min_y,node_pos[0][2] ) );
    plotpoints.push_back( btVector3( mid_x,max_y,node_pos[0][2] ) );
    plotcols.push_back( btVector4( 1,0,0,1 ) );

#ifndef DO_COVERAGE
    PlotLines::Ptr lines;
    lines.reset( new PlotLines( 2 ) );
    lines->setPoints( plotpoints,plotcols );
    env->add( lines );
#endif

    left_center_point.reset( new PlotPoints( 10 ) );

    btTransform left_tm = left_gripper1->getWorldTransform();
    std::vector<btVector3> poinsfsefts2;
    //points2.push_back( left_tm.getOrigin() );
    //points2.push_back( left_tm.getOrigin() );
    std::vector<btVector4> plotcols2;
    plotcols2.push_back( btVector4( 1,0,0,1 ) );
    //plotcols2.push_back( btVector4( 1,0,0,1 ) );

    poinsfsefts2.push_back( btVector3( mid_x,min_y,node_pos[0][2] ) );
    //poinsfsefts2[0] = left_tm.getOrigin();
    //plotcols.push_back( btVector4( 1,0,0,1 ) );

    std::vector<btVector3> plotpoints2;
    plotpoints2.push_back( left_tm.getOrigin() );
    //plotpoints2.push_back( btVector3( mid_x,max_y,node_pos[0][2] ) );


    env->add( left_center_point );
    //left_center_point->setPoints( plotpoints2 );


    left_axes1.reset( new PlotAxes() );
    left_axes1->setup( tm_left1,1 );
    env->add( left_axes1 );

    left_axes2.reset( new PlotAxes() );
    left_axes2->setup( tm_left2,1 );
    env->add( left_axes2 );

//    drag_line.reset( new PlotLines( 2 ) );
//    env->add( drag_line );

    left_gripper1->toggle();
    left_gripper1->toggleattach( clothPtr->softBody.get() );

    if( num_auto_grippers == 2 )
    {
        left_gripper2->toggle();
        left_gripper2->toggleattach( clothPtr->softBody.get() );
    }
#ifndef DO_COVERAGE
    right_gripper1->toggle();
    right_gripper1->toggleattach( clothPtr->softBody.get() );

    right_gripper2->toggle();
    right_gripper2->toggleattach( clothPtr->softBody.get() );
#else
    right_gripper1->setWorldTransform( btTransform( btQuaternion( 0,0,0,1 ), btVector3( 0,0,100 ) ) );
    right_gripper2->setWorldTransform( btTransform( btQuaternion( 0,0,0,1 ), btVector3( 0,0,100 ) ) );

#endif
}

BulletSoftObject::Ptr CustomScene::createCloth( btScalar half_side_length, const btVector3 &center )
{
    const int divs = 45;

    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        env->bullet->softBodyWorldInfo,
        center + btVector3( -half_side_length,-half_side_length,0 ),
        center + btVector3( +half_side_length,-half_side_length,0 ),
        center + btVector3( -half_side_length,+half_side_length,0 ),
        center + btVector3( +half_side_length,+half_side_length,0 ),
        divs, divs,
        0, true );

    psb->m_cfg.piterations = 10;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
        | btSoftBody::fCollision::CL_RS      ;//  | btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = 1.0;
    psb->getCollisionShape()->setMargin( 0.05 );
    btSoftBody::Material *pm = psb->appendMaterial();
    //pm->m_kLST = 0.2; //makes it rubbery ( handles self collisions better )
    psb->m_cfg.kDP = 0.05;
    psb->generateBendingConstraints( 2, pm );
    psb->randomizeConstraints();
    psb->setTotalMass( 1, true );
    psb->generateClusters( 0 );
    // the more clusters, the better the simulation, but it's slower ( and perhaps not stable )
/*    psb->generateClusters( 500 );

    for ( size_t i = 0; i < psb->m_clusters.size(); ++i )
    {
        psb->m_clusters[i]->m_selfCollisionImpulseFactor = 0.1;
    }*/

    return BulletSoftObject::Ptr( new BulletSoftObject( psb ) );
}

void CustomScene::run( bool syncTime )
{
    viewer.addEventHandler( new CustomKeyHandler( *this ) );

    //addPreStepCallback( std::bind( &CustomScene::getDesiredGripperTrajectory, this ) );
    //    addPreStepCallback( std::bind( &CustomScene::doJTracking, this ) );
    addPreStepCallback( std::bind( &CustomScene::drawAxes, this ) );


    initializePloting();

    // if syncTime is set, the simulator blocks until the real time elapsed
    // matches the simulator time elapsed
    setSyncTime( syncTime );
    startViewer();
    stepFor( BulletConfig::dt, 2 );

    startFixedTimestepLoop( BulletConfig::dt );
}
