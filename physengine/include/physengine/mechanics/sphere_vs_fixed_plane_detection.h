#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H

#include "../bits/types.h"
#include "../utils/type_conversion.h"
#include "compute_trajectory.h"

// stl
#include <optional>

namespace dte3607::physengine::mechanics
{

  inline std::optional<types::ValueType> detectCollisionSphereFixedPlane(
    [[maybe_unused]] types::HighResolutionTP const& sphere_tc,
    [[maybe_unused]] types::Point3 const&           sphere_p,
    [[maybe_unused]] types::ValueType               sphere_r,
    [[maybe_unused]] types::Vector3 const&          sphere_v,
    [[maybe_unused]] types::Point3 const&           fplane_q,
    [[maybe_unused]] types::Vector3 const&          fplane_n,
    [[maybe_unused]] types::Vector3 const&          external_forces,
    [[maybe_unused]] types::HighResolutionTP const& t_0,
    [[maybe_unused]] types::Duration                timestep)
  {
    auto const d = (fplane_q + sphere_r * fplane_n) - sphere_p;
    auto const ds
      = computeLinearTrajectory(sphere_v, external_forces, timestep).first;
    auto const inner_d_n  = blaze::inner(d, fplane_n);
    auto const inner_ds_n = blaze::inner(ds, fplane_n);

    // Scalar of trajectory for the point of collision
    auto const x = (inner_ds_n != 0) ? inner_d_n / inner_ds_n : NULL;

    // Sphere is moving parallel to wall (no collision)
    if (x == NULL) {
      return std::nullopt;
    }

    // Collision is backwards in time, or after current timestep
    else if (x <= 0 || x > 1) {
      return std::nullopt;
    }

    // Collision is before or at the same time as previous collision
    else if (t_0 + x * timestep <= sphere_tc) {
      return std::nullopt;
    }

    // Collision is valid :)
    return x;
  }

}   // namespace dte3607::physengine::mechanics



#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
