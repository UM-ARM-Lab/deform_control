#include "custom_scene/table_kinematic_object.h"

#include <boost/make_shared.hpp>



TableKinematicObject::TableKinematicObject(
        const std::string& name,
        const btTransform& surface_transform,
        const btVector3 extents,
        const btScalar thickness,
        const btVector4& color,
        const bool create_legs)
    : name_(name)
    , extents_(extents)
{
    // Zero mass indicates a unmovable object
    const btScalar mass_zero = 0.0f;
    const bool is_kinematic = true;

    const btVector3 table_surface_half_extents(extents.x() / 2.0f, extents.y() / 2.0f, thickness / 2.0f);
    const btTransform table_surface_com = surface_transform *
            btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.0f, 0.0f, -thickness / 2.0f));

    BoxObject::Ptr surface = boost::make_shared<BoxObject> (
                mass_zero,
                table_surface_half_extents,
                table_surface_com,
                is_kinematic);
    surface->setColor(color[0], color[1], color[2], color[3]);
    surface->rigidBody->setFriction(1.0f);

    children.push_back(surface);

    if (create_legs)
    {
        const btScalar leg_length = extents.z() - thickness;

        const btVector3 leg_half_extents =
                btVector3(thickness / 4.0f,
                          thickness / 4.0f,
                          leg_length / 2.0f);

        const float x_offset = table_surface_half_extents.x() - thickness / 2.0f;
        const float y_offset = table_surface_half_extents.y() - thickness / 2.0f;
        const float z_offset = -thickness - leg_half_extents.z();

        // Leg 0, -x, -y
        {
            const btTransform leg_transform = surface_transform *
                    btTransform(btQuaternion(0, 0, 0, 1), btVector3(-x_offset, -y_offset, z_offset));

            BoxObject::Ptr leg = boost::make_shared<BoxObject> (
                        mass_zero,
                        leg_half_extents,
                        leg_transform,
                        is_kinematic);
            leg->setColor(color[0], color[1], color[2], color[3]);
            leg->rigidBody->setFriction(1.0f);

            children.push_back(leg);
        }
        // Leg 1, -x, y
        {
            const btTransform leg_transform = surface_transform *
                    btTransform(btQuaternion(0, 0, 0, 1), btVector3(-x_offset, y_offset, z_offset));

            BoxObject::Ptr leg = boost::make_shared<BoxObject> (
                        mass_zero,
                        leg_half_extents,
                        leg_transform,
                        is_kinematic);
            leg->setColor(color[0], color[1], color[2], color[3]);
            leg->rigidBody->setFriction(1.0f);

            children.push_back(leg);
        }
        // Leg 2, x, -y
        {
            const btTransform leg_transform = surface_transform *
                    btTransform(btQuaternion(0, 0, 0, 1), btVector3(x_offset, -y_offset, z_offset));

            BoxObject::Ptr leg = boost::make_shared<BoxObject> (
                        mass_zero,
                        leg_half_extents,
                        leg_transform,
                        is_kinematic);
            leg->setColor(color[0], color[1], color[2], color[3]);
            leg->rigidBody->setFriction(1.0f);

            children.push_back(leg);
        }
        // Leg 3, x, y
        {
            const btTransform leg_transform = surface_transform *
                    btTransform(btQuaternion(0, 0, 0, 1), btVector3(x_offset, y_offset, z_offset));

            BoxObject::Ptr leg = boost::make_shared<BoxObject> (
                        mass_zero,
                        leg_half_extents,
                        leg_transform,
                        is_kinematic);
            leg->setColor(color[0], color[1], color[2], color[3]);
            leg->rigidBody->setFriction(1.0f);

            children.push_back(leg);
        }
    }
}

EnvironmentObject::Ptr TableKinematicObject::copy(Fork &f) const
{
    (void)f;
    #pragma message "Table copy not implemented in table_kinematic_object.cpp"
 //   assert(false && "Table copy not implemented");
}

void TableKinematicObject::internalCopy(TableKinematicObject::Ptr o, Fork &f) const
{
    (void)o;
    (void)f;
    #pragma message "Table copy not implemented in table_kinematic_object.cpp"
//    assert(false && "Table copy not implemented");
}
