#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H

#include "../bits/types.h"
#include "../utils/type_conversion.h"
#include "compute_trajectory.h"

// stl
#include <optional>

namespace dte3607::physengine::mechanics
{

  inline std::optional<types::HighResolutionTP> detectCollisionSphereFixedPlane(
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
    auto const dt         = utils::toDt(timestep);

    // Scalar of trajectory for the point of collision
    auto const x = (inner_ds_n != 0) ? inner_d_n / inner_ds_n : NULL;

    // Sphere is moving parallel to wall (no collision)
    if (x == NULL) {
      return std::nullopt;
    }

    // Collision is before start time, or after the remaining time of the
    // timestep
    else if (x <= 0 || x > 1 - (utils::toDt(sphere_tc - t_0) / dt)) {
      return std::nullopt;
    }

    // Collision is valid :)
    return sphere_tc + utils::toDuration(types::SecondsD(x * dt));
  }

}   // namespace dte3607::physengine::mechanics



#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
