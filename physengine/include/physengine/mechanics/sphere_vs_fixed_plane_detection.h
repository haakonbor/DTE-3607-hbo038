#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H

#include "../bits/types.h"
#include "../utils/type_conversion.h"
#include "../mechanics/compute_trajectory.h"

// stl
#include <optional>
#include <chrono>

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

    auto const innerproduct_ds_n = blaze::inner(ds, fplane_n);

    // Sphere is moving parallel to wall
    if (innerproduct_ds_n == 0) {
      return std::nullopt;
    }

    // Scalar of trajectory where collision happens
    auto const x = blaze::inner(d, fplane_n) / innerproduct_ds_n;

    // Collision is backwards along trajectory, or after trajectory ends
    if (x <= 0 || x > 1) {
      return std::nullopt;
    }

    auto const time_of_impact = t_0 + x * timestep;

    // Collision is backwards in time, or before a previous handled collision,
    // or after the timestep
    if (time_of_impact > t_0 && time_of_impact > sphere_tc
        && time_of_impact < t_0 + timestep) {
      return x;
    }
    else {
      return std::nullopt;
    }
  }

}   // namespace dte3607::physengine::mechanics



#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
