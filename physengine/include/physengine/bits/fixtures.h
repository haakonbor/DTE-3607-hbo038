#ifndef DTE3607_PHYSENGINE_FIXTURES_H
#define DTE3607_PHYSENGINE_FIXTURES_H

#include "types.h"

// stl
#include <variant>

namespace dte3607::physengine::fixtures
{



  struct FixtureLevel2 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;

    // RigidBody types
//    using RBMode  = types::RBMode;
//    using RBState = types::RBState;


    /*** API concept required methods ***/

    // Global properties
    size_t noRigidBodies() const { return {}; }
    Forces externalForces() const { return {}; }

    void setGravity(Forces){}


    // RBs
    std::vector<size_t> nonFixedSpheres() const { return {}; }
    std::vector<size_t> fixedInfPlanes() const { return {}; }

    ValueType rbSphereRadius([[maybe_unused]] size_t s_rid) const { return {}; }
    Vector3   rbPlaneNormal([[maybe_unused]] size_t p_rid) const { return {}; }

    // RB properties
    types::Point3 globalFramePosition([[maybe_unused]] size_t rid) const
    {
      return {};
    }


    //    types::Vector3 globalVelocity([[maybe_unused]] size_t rid) const
    //    {
    //      return {};
    //    }

    // Modes and states
//    RBMode mode([[maybe_unused]] size_t rid) const { return {}; }

    // Transform
//    void translateParent([[maybe_unused]] size_t  rid,
//                         [[maybe_unused]] Vector3 lin_trajectory)
//    {
//    }
//    void setVelocity([[maybe_unused]] size_t  rid,
//                     [[maybe_unused]] Vector3 velocity)
//    {
//    }
//    void addAcceleration([[maybe_unused]] size_t  rid,
//                         [[maybe_unused]] Vector3 accel)
//    {
//    }


    /*** Fixture unit-test setup API ***/

    void createSphere([[maybe_unused]] ValueType radius        = 1.,
                      [[maybe_unused]] Vector3   velocity      = {0, 0, 0},
                      [[maybe_unused]] Vector3   translation   = {0, 0, 0},
                      [[maybe_unused]] ValueType friction_coef = 1.)
    {
    }

    void createFixedInfPlane([[maybe_unused]] Vector3   normal      = {0, 1, 0},
                             [[maybe_unused]] Vector3   translation = {0, 0, 0},
                             [[maybe_unused]] ValueType friction_coef = 1.0)
    {
    }

    /*** END API requirements ***/
  };


}   // namespace dte3607::physengine::fixtures


#endif // DTE3607_PHYSENGINE_FIXTURES_H
