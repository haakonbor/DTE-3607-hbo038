#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H

#include "../bits/types.h"
#include "../utils/type_conversion.h"

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
    auto const d  = (fplane_q + sphere_r * fplane_n) - sphere_p;
    auto const dt = utils::toDt(timestep);
    auto const ds = sphere_v * dt + 0.5 * external_forces * std::pow(dt, 2.0);
    auto const x  = blaze::inner(d, fplane_n) / blaze::inner(ds, fplane_n);
    auto const time_of_impact = x * dt;

    if (blaze::inner(d, fplane_n) < std::numeric_limits<double>::epsilon()) {
      return time_of_impact;
    }
    else {
      return false;
    }
  }

}   // namespace dte3607::physengine::mechanics



#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
