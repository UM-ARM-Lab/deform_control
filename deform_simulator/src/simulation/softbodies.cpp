#include "simulation/softbodies.h"
#include "utils/util.h"

#include <osg/LightModel>
#include <osg/BlendFunc>
#include <BulletSoftBody/btSoftBodyInternals.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyData.h>
#include <osgbCollision/Utils.h>
#include <osgUtil/SmoothingVisitor>

#define COPY_ARRAY(a, b) { (a).resize((b).size()); for (int z = 0; z < (b).size(); ++z) (a)[z] = (b)[z]; }

// there is no btSoftBody::appendFace that takes Node pointers, so here it is
static void btSoftBody_appendFace(btSoftBody *psb, btSoftBody::Node *node0, btSoftBody::Node *node1,
        btSoftBody::Node *node2, btSoftBody::Material *mat=0) {
    btAssert(node0!=node1);
    btAssert(node1!=node2);
    btAssert(node2!=node0);
    if (node0==node1)
            return;
    if (node1==node2)
            return;
    if (node2==node0)
            return;

    psb->appendFace(-1, mat);
    btSoftBody::Face &f = psb->m_faces[psb->m_faces.size()-1];
    f.m_n[0] = node0;
    f.m_n[1] = node1;
    f.m_n[2] = node2;
    f.m_ra = AreaOf(f.m_n[0]->m_x, f.m_n[1]->m_x, f.m_n[2]->m_x);
    psb->m_bUpdateRtCst = true;
}

void BulletSoftObject::init() {
    getEnvironment()->bullet->dynamicsWorld->addSoftBody(softBody.get());

    trivertices = new osg::Vec3Array;
    trinormals = new osg::Vec3Array;
    trigeom = new osg::Geometry;
    trigeom->setDataVariance(osg::Object::DYNAMIC);
    trigeom->setUseDisplayList(false);
    trigeom->setUseVertexBufferObjects(true);
    trigeom->setVertexArray(trivertices.get());
    trigeom->setNormalArray(trinormals.get());
    trigeom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    quadvertices = new osg::Vec3Array;
    quadnormals = new osg::Vec3Array;
    quadgeom = new osg::Geometry;
    quadgeom->setDataVariance(osg::Object::DYNAMIC);
    quadgeom->setUseDisplayList(false);
    quadgeom->setUseVertexBufferObjects(true);
    quadgeom->setVertexArray(quadvertices.get());
    quadgeom->setNormalArray(quadnormals.get());
    quadgeom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    geode = new osg::Geode;
    geode->addDrawable(trigeom);
    geode->addDrawable(quadgeom);

    osg::ref_ptr<osg::LightModel> lightModel = new osg::LightModel;
    lightModel->setTwoSided(true);
    osg::StateSet* ss = geode->getOrCreateStateSet();
    ss->setAttributeAndModes(lightModel.get(), osg::StateAttribute::ON);

    transform = new osg::MatrixTransform;
    transform->addChild(geode);
    getEnvironment()->osg->root->addChild(transform);
}

void BulletSoftObject::setColor(float r, float g, float b, float a) {
  osg::Vec4Array* colors = new osg::Vec4Array;
  colors->push_back(osg::Vec4(r,g,b,a));
  trigeom->setColorArray(colors);
  trigeom->setColorBinding(osg::Geometry::BIND_OVERALL);
  quadgeom->setColorArray(colors);
  quadgeom->setColorBinding(osg::Geometry::BIND_OVERALL);

  if (a != 1.0f) { // precision problems?
    osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
    blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    osg::StateSet *ss = geode->getOrCreateStateSet();
    ss->setAttributeAndModes(blendFunc);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
  }
}

void BulletSoftObject::preDraw() {
    transform->setMatrix(osgbCollision::asOsgMatrix(softBody->getWorldTransform()));

    if (trigeom->getNumPrimitiveSets() > 0)
        trigeom->removePrimitiveSet(0); // there should only be one
    trivertices->clear();
    trinormals->clear();
    const btSoftBody::tFaceArray &faces = softBody->m_faces;
    for (int i = 0; i < faces.size(); ++i) {
        trivertices->push_back(util::toOSGVector(faces[i].m_n[0]->m_x));
        trivertices->push_back(util::toOSGVector(faces[i].m_n[1]->m_x));
        trivertices->push_back(util::toOSGVector(faces[i].m_n[2]->m_x));

        trinormals->push_back(util::toOSGVector(faces[i].m_n[0]->m_n));
        trinormals->push_back(util::toOSGVector(faces[i].m_n[1]->m_n));
        trinormals->push_back(util::toOSGVector(faces[i].m_n[2]->m_n));
    }
    trivertices->dirty();
    trinormals->dirty();
    trigeom->dirtyBound();
    trigeom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, trivertices->size()));

    if (quadgeom->getNumPrimitiveSets() > 0)
        quadgeom->removePrimitiveSet(0);
    quadvertices->clear();
    quadnormals->clear();
    const btSoftBody::tTetraArray &tetras = softBody->m_tetras;
    for (int i = 0; i < tetras.size(); ++i) {
        quadvertices->push_back(util::toOSGVector(tetras[i].m_n[0]->m_x));
        quadvertices->push_back(util::toOSGVector(tetras[i].m_n[1]->m_x));
        quadvertices->push_back(util::toOSGVector(tetras[i].m_n[2]->m_x));
        quadvertices->push_back(util::toOSGVector(tetras[i].m_n[3]->m_x));
/*
        quadnormals->push_back(util::toOSGVector(tetras[i].m_n[0]->m_n));
        quadnormals->push_back(util::toOSGVector(tetras[i].m_n[1]->m_n));
        quadnormals->push_back(util::toOSGVector(tetras[i].m_n[2]->m_n));
        quadnormals->push_back(util::toOSGVector(tetras[i].m_n[3]->m_n));*/
    }
    quadvertices->dirty();
    quadnormals->dirty();
    quadgeom->dirtyBound();
    quadgeom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, quadvertices->size()));

/*    osgUtil::SmoothingVisitor sv;
    sv.apply(*geode);*/
}

void BulletSoftObject::destroy() {
    getEnvironment()->bullet->dynamicsWorld->removeSoftBody(softBody.get());
}

// adapted from bullet's SerializeDemo.cpp
EnvironmentObject::Ptr BulletSoftObject::copy(Fork &f) const
{
    const btSoftBody* const orig = softBody.get();

    // create a new softBody with the data
    btSoftBody * const newPsb = new btSoftBody(&f.env->bullet->softBodyWorldInfo);
    f.registerCopy(orig, newPsb);

    // materials
    newPsb->m_materials.reserve(orig->m_materials.size());
    for (int i = 0; i < orig->m_materials.size(); i++)
    {
        const btSoftBody::Material *mat = orig->m_materials[i];
        btSoftBody::Material *newMat = newPsb->appendMaterial();
        newMat->m_flags = mat->m_flags;
        newMat->m_kAST = mat->m_kAST;
        newMat->m_kLST = mat->m_kLST;
        newMat->m_kVST = mat->m_kVST;
        f.registerCopy(mat, newMat);
    }

    // nodes
    newPsb->m_nodes.reserve(orig->m_nodes.size());
    for (int i = 0; i < orig->m_nodes.size(); i++)
    {
        const btSoftBody::Node *node = &orig->m_nodes[i];
        newPsb->appendNode(node->m_x, node->m_im ? 1./node->m_im : 0.);
        btSoftBody::Node *newNode = &newPsb->m_nodes[newPsb->m_nodes.size()-1];
        newNode->m_area = node->m_area;
        newNode->m_battach = node->m_battach;
        newNode->m_f = node->m_f;
        newNode->m_im = node->m_im;
        newNode->m_n = node->m_n;
        newNode->m_q = node->m_q;
        newNode->m_v = node->m_v;

        newNode->m_material = (btSoftBody::Material *) f.copyOf(node->m_material);
        BOOST_ASSERT(newNode->m_material);

        f.registerCopy(node, newNode);
        BOOST_ASSERT(f.copyOf(node) == newNode);
        BOOST_ASSERT(((btSoftBody::Node*)f.copyOf(node))->m_x == node->m_x);
    }

    // links
    newPsb->m_links.reserve(orig->m_links.size());
    for (int i = 0; i < orig->m_links.size(); i++)
    {
        const btSoftBody::Link *link = &orig->m_links[i];
        btSoftBody::Material *mat = (btSoftBody::Material *) f.copyOf(link->m_material);
        btSoftBody::Node *n0 = (btSoftBody::Node *) f.copyOf(link->m_n[0]);
        btSoftBody::Node *n1 = (btSoftBody::Node *) f.copyOf(link->m_n[1]);
        BOOST_ASSERT(n0 && n1);
        BOOST_ASSERT(((btSoftBody::Node*)f.copyOf(link->m_n[0]))->m_x == link->m_n[0]->m_x);
        BOOST_ASSERT(((btSoftBody::Node*)f.copyOf(link->m_n[1]))->m_x == link->m_n[1]->m_x);
        newPsb->appendLink(n0, n1, mat);

        btSoftBody::Link *newLink = &newPsb->m_links[newPsb->m_links.size() - 1];
        newLink->m_bbending = link->m_bbending;
        newLink->m_rl = link->m_rl;
    }

    // faces
    newPsb->m_faces.reserve(orig->m_faces.size());
    for (int i = 0; i < orig->m_faces.size(); i++)
    {
        const btSoftBody::Face *face = &orig->m_faces[i];
        btSoftBody::Material *mat = (btSoftBody::Material *) f.copyOf(face->m_material);
        btSoftBody::Node *n0 = (btSoftBody::Node *) f.copyOf(face->m_n[0]);
        btSoftBody::Node *n1 = (btSoftBody::Node *) f.copyOf(face->m_n[1]);
        btSoftBody::Node *n2 = (btSoftBody::Node *) f.copyOf(face->m_n[2]);
        BOOST_ASSERT(n0 && n1 && n2);

        btSoftBody_appendFace(newPsb, n0, n1, n2, mat);

        btSoftBody::Face *newFace = &newPsb->m_faces[newPsb->m_faces.size()-1];
        newFace->m_normal = face->m_normal;
        newFace->m_ra = face->m_ra;
    }

    // tetras
    // TODO: This code block has not been tested (never activated)
    newPsb->m_tetras.resize(orig->m_tetras.size());
    for (int i = 0; i < orig->m_tetras.size(); i++)
    {
        btSoftBody::Tetra *newTetra = &newPsb->m_tetras[i];
        const btSoftBody::Tetra *tetra = &orig->m_tetras[i];
        for (int j = 0; j < 4; j++)
        {
            newTetra->m_c0[j] = tetra->m_c0[j];
            newTetra->m_n[j] = (btSoftBody::Node *) f.copyOf(tetra->m_n[j]);
            BOOST_ASSERT(newTetra->m_n[j]);
        }
        newTetra->m_rv = tetra->m_rv;

        newTetra->m_leaf = (btDbvtNode *) f.copyOf(tetra->m_leaf);
        BOOST_ASSERT(newTetra->m_leaf);

        newTetra->m_c1 = tetra->m_c1;
        newTetra->m_c2 = tetra->m_c2;

        // I think I'm doing this wrong, as this data is likely to not have been copied yet
        newTetra->m_tag = (void *) f.copyOf(tetra->m_tag);
        BOOST_ASSERT(!tetra->m_tag || newTetra->m_tag);

        f.registerCopy(tetra, newTetra);
        BOOST_ASSERT(f.copyOf(tetra) == newTetra);
    }

    // anchors
    // Anchors are copied in postCopy to ensure that the rigid bodies they are
    // connected to have been copied over

    // pose
    newPsb->m_pose.m_bvolume = orig->m_pose.m_bvolume;
    newPsb->m_pose.m_bframe = orig->m_pose.m_bframe;
    newPsb->m_pose.m_volume = orig->m_pose.m_volume;
    COPY_ARRAY(newPsb->m_pose.m_pos, orig->m_pose.m_pos);
    COPY_ARRAY(newPsb->m_pose.m_wgh, orig->m_pose.m_wgh);
    newPsb->m_pose.m_com = orig->m_pose.m_com;
    newPsb->m_pose.m_rot = orig->m_pose.m_rot;
    newPsb->m_pose.m_scl = orig->m_pose.m_scl;
    newPsb->m_pose.m_aqq = orig->m_pose.m_aqq;

    // config
    newPsb->m_cfg.aeromodel = orig->m_cfg.aeromodel;
    newPsb->m_cfg.kVCF = orig->m_cfg.kVCF;
    newPsb->m_cfg.kDP = orig->m_cfg.kDP;
    newPsb->m_cfg.kDG = orig->m_cfg.kDG;
    newPsb->m_cfg.kLF = orig->m_cfg.kLF;
    newPsb->m_cfg.kPR = orig->m_cfg.kPR;
    newPsb->m_cfg.kVC = orig->m_cfg.kVC;
    newPsb->m_cfg.kDF = orig->m_cfg.kDF;
    newPsb->m_cfg.kMT = orig->m_cfg.kMT;
    newPsb->m_cfg.kCHR = orig->m_cfg.kCHR;
    newPsb->m_cfg.kKHR = orig->m_cfg.kKHR;
    newPsb->m_cfg.kSHR = orig->m_cfg.kSHR;
    newPsb->m_cfg.kAHR = orig->m_cfg.kAHR;
    newPsb->m_cfg.kSRHR_CL = orig->m_cfg.kSRHR_CL;
    newPsb->m_cfg.kSKHR_CL = orig->m_cfg.kSKHR_CL;
    newPsb->m_cfg.kSSHR_CL = orig->m_cfg.kSSHR_CL;
    newPsb->m_cfg.kSR_SPLT_CL = orig->m_cfg.kSR_SPLT_CL;
    newPsb->m_cfg.kSK_SPLT_CL = orig->m_cfg.kSK_SPLT_CL;
    newPsb->m_cfg.kSS_SPLT_CL = orig->m_cfg.kSS_SPLT_CL;
    newPsb->m_cfg.maxvolume = orig->m_cfg.maxvolume;
    newPsb->m_cfg.timescale = orig->m_cfg.timescale;
    newPsb->m_cfg.viterations = orig->m_cfg.viterations;
    newPsb->m_cfg.piterations = orig->m_cfg.piterations;
    newPsb->m_cfg.diterations = orig->m_cfg.diterations;
    newPsb->m_cfg.citerations = orig->m_cfg.citerations;
    newPsb->m_cfg.collisions = orig->m_cfg.collisions;
    COPY_ARRAY(newPsb->m_cfg.m_vsequence, orig->m_cfg.m_vsequence);
    COPY_ARRAY(newPsb->m_cfg.m_psequence, orig->m_cfg.m_psequence);
    COPY_ARRAY(newPsb->m_cfg.m_dsequence, orig->m_cfg.m_dsequence);

    newPsb->getCollisionShape()->setMargin(orig->getCollisionShape()->getMargin());

    // solver state
    newPsb->m_sst = orig->m_sst;

    // clusters
    newPsb->m_clusters.resize(orig->m_clusters.size());
    for (int i = 0; i < orig->m_clusters.size(); i++)
    {
        btSoftBody::Cluster *cl = orig->m_clusters[i];
        btSoftBody::Cluster *newcl = newPsb->m_clusters[i] =
            new(btAlignedAlloc(sizeof(btSoftBody::Cluster),16)) btSoftBody::Cluster();

        newcl->m_nodes.resize(cl->m_nodes.size());
        for (int j = 0; j < cl->m_nodes.size(); j++)
        {
            newcl->m_nodes[j] = (btSoftBody::Node *) f.copyOf(cl->m_nodes[j]);
        }
        COPY_ARRAY(newcl->m_masses, cl->m_masses);
        COPY_ARRAY(newcl->m_framerefs, cl->m_framerefs);
        newcl->m_framexform = cl->m_framexform;
        newcl->m_idmass = cl->m_idmass;
        newcl->m_imass = cl->m_imass;
        newcl->m_locii = cl->m_locii;
        newcl->m_invwi = cl->m_invwi;
        newcl->m_com = cl->m_com;
        newcl->m_vimpulses[0] = cl->m_vimpulses[0];
        newcl->m_vimpulses[1] = cl->m_vimpulses[1];
        newcl->m_dimpulses[0] = cl->m_dimpulses[0];
        newcl->m_dimpulses[1] = cl->m_dimpulses[1];
        newcl->m_nvimpulses = cl->m_nvimpulses;
        newcl->m_ndimpulses = cl->m_ndimpulses;
        newcl->m_lv = cl->m_lv;
        newcl->m_av = cl->m_av;
        newcl->m_leaf = 0; // soft body code will set this automatically
        newcl->m_ndamping = cl->m_ndamping;
        newcl->m_ldamping = cl->m_ldamping;
        newcl->m_adamping = cl->m_adamping;
        newcl->m_matching = cl->m_matching;
        newcl->m_maxSelfCollisionImpulse = cl->m_maxSelfCollisionImpulse;
        newcl->m_selfCollisionImpulseFactor = cl->m_selfCollisionImpulseFactor;
        newcl->m_containsAnchor = cl->m_containsAnchor;
        newcl->m_collide = cl->m_collide;
        newcl->m_clusterIndex = cl->m_clusterIndex;
    }

    // cluster connectivity
    COPY_ARRAY(newPsb->m_clusterConnectivity, orig->m_clusterConnectivity);

    // Stuff found through assertion failure
    newPsb->setInterpolationWorldTransform(orig->getInterpolationWorldTransform());
    newPsb->setInterpolationLinearVelocity(orig->getInterpolationLinearVelocity());
    newPsb->setInterpolationAngularVelocity(orig->getInterpolationAngularVelocity());
    newPsb->setCompanionId(orig->getCompanionId());
    newPsb->m_bounds[0] = orig->m_bounds[0];
    newPsb->m_bounds[1] = orig->m_bounds[1];
    newPsb->m_bUpdateRtCst = orig->m_bUpdateRtCst;

//    newPsb->getBroadphaseHandle()->m_aabbMax = orig->getBroadphaseHandle()->m_aabbMax;

//    newPsb->updateBounds();

//    if(newPsb->m_bUpdateRtCst)
//    {
//        newPsb->m_bUpdateRtCst=false;
//        newPsb->updateConstants();
//        newPsb->m_fdbvt.clear();
//        if(newPsb->m_cfg.collisions & btSoftBody::fCollision::VF_SS)
//        {
//            newPsb->initializeFaceTree();
//        }
//    }

//    newPsb->getBroadphaseHandle()->

    return Ptr(new BulletSoftObject(newPsb));
}

void BulletSoftObject::postCopy(EnvironmentObject::Ptr copy, Fork &f) const
{
    const btSoftBody *orig = softBody.get();
    btSoftBody *newPsb = boost::static_pointer_cast<BulletSoftObject>(copy)->softBody.get();

    // copy the anchors
    newPsb->m_anchors.reserve(orig->m_anchors.size());
    for (int i = 0; i < orig->m_anchors.size(); i++)
    {
        const btSoftBody::Anchor &anchor = orig->m_anchors[i];
        btRigidBody *body = btRigidBody::upcast((btCollisionObject *) f.copyOf(anchor.m_body));
        BOOST_ASSERT(body);
        if (!body) continue;

        btSoftBody::Anchor newAnchor = anchor;
        newAnchor.m_body = body;

        newAnchor.m_node = (btSoftBody::Node *) f.copyOf(anchor.m_node);
        BOOST_ASSERT(newAnchor.m_node);

        newPsb->m_anchors.push_back(newAnchor);

        // TODO: disableCollisionBetweenLinkedBodies
    }

    // copy the joints
    // TODO: THIS IS NOT TESTED
#if 0
    std::cout << orig->m_joints.size() << "          afdasdf\n\n\n";
    newPsb->m_joints.reserve(orig->m_joints.size());
    for (int i = 0; i < orig->m_joints.size(); i++)
    {
        btSoftBody::Joint *joint = orig->m_joints[i];

        const btTransform &transA = newPsb->m_clusters[0]->m_framexform;

        btSoftBody::Body bdyB;
        const btSoftBody::Body &origBodyB = joint->m_bodies[1];
        if (origBodyB.m_soft)
        {
            bdyB.m_soft = (btSoftBody::Cluster *) f.copyOf(origBodyB.m_soft);
        }
        if (origBodyB.m_rigid)
        {
            bdyB.m_rigid = (btRigidBody *) f.copyOf(origBodyB.m_rigid);
        }
        if (origBodyB.m_collisionObject)
        {
            bdyB.m_collisionObject = (btCollisionObject *) f.copyOf(origBodyB.m_collisionObject);
        }

        /*
        btCollisionObject *colB = f.copyOf(joint->m_bodies[1]);
        switch (colB->getInternalType()) {
        case CO_COLLISION_OBJECT:
            bdyB = colB; break;
        case CO_RIGID_BODY:
            bdyB = btRigidBody::upcast(colB); break;
        case CO_SOFT_BODY:
            bdyB = ((btSoftBody *) colB)->m_clusters[0]; break;
        }*/

        if (joint->Type() == btSoftBody::Joint::eType::Linear)
        {
            btSoftBody::LJoint *ljoint = (btSoftBody::LJoint *) joint, newljoint;
            btSoftBody::LJoint::Specs specs;
            specs.cfm = ljoint->m_cfm;
            specs.erp = ljoint->m_erp;
            specs.split = ljoint->m_split;
            specs.position = transA * ljoint->m_refs[0];
            newPsb->appendLinearJoint(specs, newPsb->m_clusters[0], bdyB);
        }
        else if (joint->Type() == btSoftBody::Joint::eType::Angular)
        {
            btSoftBody::AJoint *ajoint = (btSoftBody::AJoint *) joint;
            btSoftBody::AJoint::Specs specs;
            specs.cfm = ajoint->m_cfm;
            specs.erp = ajoint->m_erp;
            specs.split = ajoint->m_split;
            specs.axis = transA.getBasis() * ajoint->m_refs[0];
            newPsb->appendAngularJoint(specs, newPsb->m_clusters[0], bdyB);
        }
        else if (joint->Type() == btSoftBody::Joint::eType::Contact)
        {
            btSoftBody::CJoint *cjoint = (btSoftBody::CJoint *) joint;
            std::cout << "error: contact joints not implemented!\n";
        }
        else
        {
            std::cout << "error: unknown joint type " << joint->Type() << '\n';
        }
    }

#endif
}
