cmake_minimum_required( VERSION 2.8.3 )
project( osgBullet )

find_package( catkin REQUIRED COMPONENTS
    BULLET_PHYSICS
    osgWorks
)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}
    SYSTEM
    ${catkin_INCLUDE_DIRS}
)

set( OSGBULLET_SOURCES
    osgbCollision/ComputeCylinderVisitor.cpp
    osgbCollision/ComputeTriMeshVisitor.cpp
    osgbCollision/Utils.cpp
    osgbCollision/Chart.cpp
    osgbCollision/BoundingCylinder.cpp
    osgbCollision/BoundingCone.cpp
    osgbCollision/CollectVerticesVisitor.cpp
    osgbCollision/Version.cpp
    osgbCollision/CollisionShapes.cpp
    osgbCollision/VertexAggOp.cpp
    osgbCollision/GLDebugDrawer.cpp
    osgbCollision/ComputeShapeVisitor.cpp
    osgbDynamics/PhysicsState.cpp
    osgbDynamics/RigidBody.cpp
    osgbDynamics/PhysicsThread.cpp
    osgbDynamics/RigidBodyAnimation.cpp
    osgbDynamics/TripleBuffer.cpp
    osgbDynamics/GroundPlane.cpp
    osgbDynamics/CreationRecord.cpp
    osgbDynamics/MotionState.cpp
    osgbInteraction/HandTestEventHandler.cpp
    osgbInteraction/ArticulationRecord.cpp
    osgbInteraction/LaunchHandler.cpp
    osgbInteraction/SaveRestoreHandler.cpp
    osgbInteraction/HandNode.cpp
    osgbInteraction/DragHandler.cpp
#    osgbInteraction/p5support.cpp
    osgbInteraction/GestureHandler.cpp
)

add_library( ${PROJECT_NAME}
    ${OSGBULLET_SOURCES}
)
add_dependencies( ${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries( ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

catkin_package(
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS      osgWorks
    LIBRARIES    ${PROJECT_NAME}
)
