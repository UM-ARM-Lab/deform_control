#include "gripper_kinematic_object.h"

#include "utils/conversions.h"

GripperKinematicObject::GripperKinematicObject( btVector4 color, int apperture_input )
    : apperture( apperture_input )
    , bOpen ( true )
    , bAttached( false )
    , halfextents( btVector3( 0.3, 0.3, 0.1 ) )
    , closed_gap( 0.1 )
{
    BoxObject::Ptr top_jaw( new BoxObject( 0, halfextents, btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0, 0, apperture/2 ) ),true ) );
    top_jaw->setColor( color[0],color[1],color[2],color[3] );
    top_jaw->motionState->getWorldTransform( cur_tm );

    BoxObject::Ptr bottom_jaw( new BoxObject( 0, halfextents, btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0,0,-apperture/2 ) ),true ) );
    bottom_jaw->setColor( color[0],color[1],color[2],color[3] );
    cur_tm.setOrigin( cur_tm.getOrigin() - btVector3( 0,0,-apperture/2 ) );

    children.push_back( top_jaw );
    children.push_back( bottom_jaw );
}

void GripperKinematicObject::applyTransform( btTransform tm )
{
    setWorldTransform( getWorldTransform()*tm );
}

void GripperKinematicObject::translate( btVector3 transvec )
{
    btTransform tm = getWorldTransform();
    tm.setOrigin( tm.getOrigin() + transvec );
    setWorldTransform( tm );
}

void GripperKinematicObject::toggleattach( btSoftBody * psb, double radius )
{
    if( bAttached )
    {
        btAlignedObjectArray<btSoftBody::Anchor> newanchors;
        for( int i = 0; i < psb->m_anchors.size(); i++ )
        {
            if( psb->m_anchors[i].m_body != children[0]->rigidBody.get() && psb->m_anchors[i].m_body != children[1]->rigidBody.get() )
                newanchors.push_back( psb->m_anchors[i] );
        }
        releaseAllAnchors( psb );
        for( int i = 0; i < newanchors.size(); i++ )
        {
            psb->m_anchors.push_back( newanchors[i] );
        }
        vattached_node_inds.clear();
    }
    else
    {
        if ( radius > 0 )
        {
            std::cout << "use radius contact?????" << std::endl;
            if( radius == 0 )
                radius = halfextents[0];
            btTransform top_tm;
            children[0]->motionState->getWorldTransform( top_tm );
            btTransform bottom_tm;
            children[1]->motionState->getWorldTransform( bottom_tm );
            int closest_body = -1;
            for( int j = 0; j < psb->m_nodes.size(); j++ )
            {
                if( ( psb->m_nodes[j].m_x - cur_tm.getOrigin() ).length() < radius )
                {
                    if( ( psb->m_nodes[j].m_x - top_tm.getOrigin() ).length() < ( psb->m_nodes[j].m_x - bottom_tm.getOrigin() ).length() )
                        closest_body = 0;
                    else
                        closest_body = 1;

                    vattached_node_inds.push_back( j );
                    appendAnchor( psb, &psb->m_nodes[j], children[closest_body]->rigidBody.get() );
                    std::cout << "\tappending anchor, closest ind: " << j << "\n";

                }
            }
        }
        else
        {
            std::vector<btVector3> nodeposvec;
            nodeArrayToNodePosVector( psb->m_nodes, nodeposvec );

            for( int k = 0; k < 2; k++ )
            {
                BoxObject::Ptr part = children[k];

                btRigidBody* rigidBody = part->rigidBody.get();
                btSoftBody::tRContactArray rcontacts;
                getContactPointsWith( psb, rigidBody, rcontacts );
                std::cout << "got " << rcontacts.size() << " contacts\n";

                //if no contacts, return without toggling bAttached
                if( rcontacts.size() == 0 )
                    continue;
                for ( int i = 0; i < rcontacts.size(); ++i ) {
                    //const btSoftBody::RContact &c = rcontacts[i];
                    btSoftBody::Node *node = rcontacts[i].m_node;
                    //btRigidBody* colLink = c.m_cti.m_colObj;
                    //if ( !colLink ) continue;
                    const btVector3 &contactPt = node->m_x;
                    //if ( onInnerSide( contactPt, left ) ) {
                        unsigned int closest_ind = 0;
                        bool closest_found = false;
                        for( unsigned int n = 0; !closest_found && n < nodeposvec.size(); n++ )
                        {
                            if( ( contactPt - nodeposvec[n] ).length() < 0.0001 )
                            {
                                closest_ind = n;
                                closest_found = true;
                            }
                        }
                        assert( closest_found );

                        vattached_node_inds.push_back( closest_ind );
                        appendAnchor( psb, node, rigidBody );
                        std::cout << "\tappending anchor, closest ind: "<< closest_ind << "\n";
                }
            }
        }
    }

    bAttached = !bAttached;
}

// Fills in the rcontacs array with contact information between psb and pco
void GripperKinematicObject::getContactPointsWith( btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts )
{
    // custom contact checking adapted from btSoftBody.cpp and btSoftBodyInternals.h
    struct Custom_CollideSDF_RS : btDbvt::ICollide {
        Custom_CollideSDF_RS( btSoftBody::tRContactArray &rcontacts_ ) : rcontacts( rcontacts_ ) { }

        void Process( const btDbvtNode* leaf ) {
            btSoftBody::Node* node=( btSoftBody::Node* )leaf->data;
            DoNode( *node );
        }

        void DoNode( btSoftBody::Node& n ) {
            const btScalar m=n.m_im>0?dynmargin:stamargin;
            btSoftBody::RContact c;
            if ( !n.m_battach && psb->checkContact( m_colObj1,n.m_x,m,c.m_cti ) ) {
                const btScalar  ima=n.m_im;
                const btScalar  imb= m_rigidBody? m_rigidBody->getInvMass() : 0.f;
                const btScalar  ms=ima+imb;
                if( ms>0 ) {
                    // there's a lot of extra information we don't need to compute
                    // since we just want to find the contact points
                    c.m_node        =       &n;

                    rcontacts.push_back( c );

                }
            }
        }
        btSoftBody*             psb;
        btCollisionObject*      m_colObj1;
        btRigidBody*    m_rigidBody;
        btScalar                dynmargin;
        btScalar                stamargin;
        btSoftBody::tRContactArray &rcontacts;
    };

    Custom_CollideSDF_RS  docollide( rcontacts );
    btRigidBody*            prb1=btRigidBody::upcast( pco );
    btTransform     wtr=pco->getWorldTransform();

    const btTransform       ctr=pco->getWorldTransform();
    const btScalar          timemargin=( wtr.getOrigin()-ctr.getOrigin() ).length();
    const btScalar          basemargin=psb->getCollisionShape()->getMargin();
    btVector3                       mins;
    btVector3                       maxs;
    ATTRIBUTE_ALIGNED16( btDbvtVolume )               volume;
    pco->getCollisionShape()->getAabb(      pco->getWorldTransform(),
            mins,
            maxs );
    volume=btDbvtVolume::FromMM( mins,maxs );
    volume.Expand( btVector3( basemargin,basemargin,basemargin ) );
    docollide.psb           =       psb;
    docollide.m_colObj1 = pco;
    docollide.m_rigidBody = prb1;

    docollide.dynmargin     =       basemargin+timemargin;
    docollide.stamargin     =       basemargin;
    psb->m_ndbvt.collideTV( psb->m_ndbvt.m_root,volume,docollide );
}

// adapted from btSoftBody.cpp ( btSoftBody::appendAnchor )
void GripperKinematicObject::appendAnchor( btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence )
{
    btSoftBody::Anchor a;
    a.m_node = node;
    a.m_body = body;
    a.m_local = body->getWorldTransform().inverse()*a.m_node->m_x;
    a.m_node->m_battach = true;
    a.m_influence = influence;
    psb->m_anchors.push_back( a );
}

void GripperKinematicObject::setWorldTransform( btTransform tm )
{
    btTransform top_tm = tm;
    btTransform bottom_tm = tm;

    btTransform top_offset;

    children[0]->motionState->getWorldTransform( top_offset );
    top_offset = cur_tm.inverse()*top_offset;

    top_tm.setOrigin( top_tm.getOrigin() + top_tm.getBasis().getColumn( 2 )*( top_offset.getOrigin()[2] ) );
    bottom_tm.setOrigin( bottom_tm.getOrigin() - bottom_tm.getBasis().getColumn( 2 )*( top_offset.getOrigin()[2] ) );

    children[0]->motionState->setKinematicPos( top_tm );
    children[1]->motionState->setKinematicPos( bottom_tm );

    cur_tm = tm;
}

void GripperKinematicObject::toggle()
{
    btTransform top_tm;    btTransform bottom_tm;
    children[0]->motionState->getWorldTransform( top_tm );
    children[1]->motionState->getWorldTransform( bottom_tm );

    if( bOpen )
    {
        //btTransform top_offset = cur_tm.inverse()*top_tm;
        //float close_length = ( 1+closed_gap )*top_offset.getOrigin()[2] - children[0]->halfExtents[2];
        //float close_length = ( apperture/2 - children[0]->halfExtents[2] + closed_gap/2 );
        top_tm.setOrigin( cur_tm.getOrigin() + cur_tm.getBasis().getColumn( 2 )*( children[0]->halfExtents[2] + closed_gap/2 ) );
        bottom_tm.setOrigin( cur_tm.getOrigin() - cur_tm.getBasis().getColumn( 2 )*( children[1]->halfExtents[2] + closed_gap/2 ) );
    }
    else
    {
        top_tm.setOrigin( cur_tm.getOrigin() - cur_tm.getBasis().getColumn( 2 )*( apperture/2 ) );
        bottom_tm.setOrigin( cur_tm.getOrigin() + cur_tm.getBasis().getColumn( 2 )*( apperture/2 ) );
    }

    children[0]->motionState->setKinematicPos( top_tm );
    children[1]->motionState->setKinematicPos( bottom_tm );

    bOpen = !bOpen;
}

EnvironmentObject::Ptr GripperKinematicObject::copy( Fork &f ) const
{
    Ptr o( new GripperKinematicObject() );
    internalCopy( o, f );
    return o;
}

void GripperKinematicObject::internalCopy( GripperKinematicObject::Ptr o, Fork &f ) const
{
    o->apperture = apperture;
    o->cur_tm = cur_tm;
    o->bOpen = bOpen;
    o->state = state;
    o->closed_gap = closed_gap;
    o->vattached_node_inds = vattached_node_inds;
    o->halfextents = halfextents;
    o->bAttached = bAttached;

    o->children.clear();
    o->children.reserve( children.size() );
    ChildVector::const_iterator i;
    for ( i = children.begin(); i != children.end(); ++i ) {
        if ( *i )
            o->children.push_back( boost::static_pointer_cast<BoxObject> ( ( *i )->copy( f ) ) );
        else
            o->children.push_back( BoxObject::Ptr() );
    }
}

void GripperKinematicObject::step_openclose( btSoftBody * psb )
{
    if ( state == GripperState_DONE ) return;

    if( state == GripperState_OPENING && bAttached )
        toggleattach( psb );


    btTransform top_tm;
    btTransform bottom_tm;
    children[0]->motionState->getWorldTransform( top_tm );
    children[1]->motionState->getWorldTransform( bottom_tm );

    double step_size = 0.005;
    if( state == GripperState_CLOSING )
    {
        top_tm.setOrigin( top_tm.getOrigin() + step_size*top_tm.getBasis().getColumn( 2 ) );
        bottom_tm.setOrigin( bottom_tm.getOrigin() - step_size*bottom_tm.getBasis().getColumn( 2 ) );
    }
    else if( state == GripperState_OPENING )
    {
        top_tm.setOrigin( top_tm.getOrigin() - step_size*top_tm.getBasis().getColumn( 2 ) );
        bottom_tm.setOrigin( bottom_tm.getOrigin() + step_size*bottom_tm.getBasis().getColumn( 2 ) );
    }

    children[0]->motionState->setKinematicPos( top_tm );
    children[1]->motionState->setKinematicPos( bottom_tm );

//        if( state == GripperState_CLOSING && !bAttached )
//            toggleattach( psb, 0.5 );

    double cur_gap_length = ( top_tm.getOrigin() - bottom_tm.getOrigin() ).length();
    if( state == GripperState_CLOSING && cur_gap_length <= ( closed_gap + 2*halfextents[2] ) )
    {
        state = GripperState_DONE;
        bOpen = false;
        if( !bAttached )
            toggleattach( psb );

    }
    if( state == GripperState_OPENING && cur_gap_length >= apperture )
    {
        state = GripperState_DONE;
        bOpen = true;

    }

//        float frac = fracElapsed();
//        vals[0] = ( 1.f - frac )*startVal + frac*endVal;
//        manip->robot->setDOFValues( indices, vals );

//        if ( vals[0] == CLOSED_VAL ) {
//            attach( true );
//            attach( false );
//        }
}

void GripperKinematicObject::rigidGrab( btRigidBody* prb, int objectnodeind, Environment::Ptr env_ptr )
{
    btTransform top_tm;
    children[0]->motionState->getWorldTransform( top_tm );

    cnt.reset( new btGeneric6DofConstraint( *( children[0]->rigidBody.get() ),
        *prb,top_tm.inverse()*cur_tm,btTransform( btQuaternion( 0,0,0,1 ),btVector3( 0,0,0 ) ),true ) );
    cnt->setLinearLowerLimit( btVector3( 0,0,0 ) );
    cnt->setLinearUpperLimit( btVector3( 0,0,0 ) );
    cnt->setAngularLowerLimit( btVector3( 0,0,0 ) );
    cnt->setAngularUpperLimit( btVector3( 0,0,0 ) );
    env_ptr->bullet->dynamicsWorld->addConstraint( cnt.get() );

    vattached_node_inds.clear();
    vattached_node_inds.push_back( objectnodeind );

}
