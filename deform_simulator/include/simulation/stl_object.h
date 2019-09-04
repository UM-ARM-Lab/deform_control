#pragma once
#include "simulation/environment.h"
#include "simulation/basicobjects.h"
#include <btBulletDynamicsCommon.h>

class StlObject : public BulletObject
{
protected:
    // Stored here because someone needs to store them (bullet doesn't),
    // and these are the internal data for this STL
    const std::vector<boost::shared_ptr<btCollisionShape>> subshapes_;
    const boost::shared_ptr<btCompoundShape> compound_shape_;

    const btScalar mass_;

public:
    typedef boost::shared_ptr<StlObject> Ptr;
    typedef boost::shared_ptr<const StlObject> ConstPtr;

    StlObject(const std::vector<boost::shared_ptr<btCollisionShape>> subshapes,
              const boost::shared_ptr<btCompoundShape> compound_shape,
              const btScalar mass,
              const btTransform& initial_transform,
              const bool is_kinematic);

    static StlObject::Ptr MakeStlObject(const std::string& filename,
                                        const btScalar mass,
                                        const btTransform& initial_transform,
                                        const bool is_kinematic);

    EnvironmentObject::Ptr copy(Fork &f) const;
};
