#ifndef TABLE_KINEMATIC_OBJECT_H
#define TABLE_KINEMATIC_OBJECT_H

#include "simulation/simplescene.h"

class TableKinematicObject : public CompoundObject<BoxObject>
{
    public:
        typedef boost::shared_ptr<TableKinematicObject> Ptr;

        TableKinematicObject(
                const std::string& name,
                const btTransform& surface_transform,
                const btVector3 extents,
                const btScalar thickness,
                const btVector4& color,
                const bool create_legs);

        EnvironmentObject::Ptr copy(Fork &f) const;

    private:
        void internalCopy(TableKinematicObject::Ptr o, Fork &f) const;

        const std::string name_;
        const btVector3 extents_;
};

#endif
