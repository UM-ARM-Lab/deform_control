#ifndef STEP_STATE_H
#define STEP_STATE_H

#include "gripper_kinematic_object.h"

class StepState
{
    public:
        BulletInstance::Ptr bullet;
        OSGInstance::Ptr osg;
        Fork::Ptr fork;
        BulletSoftObject::Ptr cloth;
        GripperKinematicObject::Ptr left_gripper1;
        GripperKinematicObject::Ptr left_gripper2;
};

#endif
