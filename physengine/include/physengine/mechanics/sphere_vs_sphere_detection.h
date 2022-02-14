#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H

#include "compute_trajectory.h"
#include "../bits/types.h"

// stl
#include <optional>

namespace dte3607::physengine::mechanics
{

  inline std::optional<types::HighResolutionTP> detectCollisionSphereSphere(
    [[maybe_unused]] types::HighResolutionTP const& s1_tc,
    [[maybe_unused]] types::Point3 const&           s1_p,
    [[maybe_unused]] types::ValueType               s1_r,
    [[maybe_unused]] types::Vector3 const&          s1_ds,
    [[maybe_unused]] types::HighResolutionTP const& s2_tc,
    [[maybe_unused]] types::Point3 const&           s2_p,
    [[maybe_unused]] types::ValueType               s2_r,
    [[maybe_unused]] types::Vector3 const&          s2_ds,
    //[[maybe_unused]] types::Vector3 const&          external_forces,
    [[maybe_unused]] types::HighResolutionTP const& t_0,
    [[maybe_unused]] types::Duration                timestep)
  {
    auto const r = s1_r + s2_r;
    //    auto const ds1
    //      = computeLinearTrajectory(s1_v, external_forces, timestep).first;
    //    auto const ds2
    //      = computeLinearTrajectory(s2_v, external_forces, timestep).first;
    auto const Q         = s2_p - s1_p;
    auto const R         = (s2_ds - s1_ds);
    auto const inner_Q_R = blaze::inner(Q, R);
    auto const inner_R_R = blaze::inner(R, R);
    auto const inner_Q_Q = blaze::inner(Q, Q);
    auto const potential_sqrt
      = std::pow(inner_Q_R, 2.0) - inner_R_R * (inner_Q_Q - std::pow(r, 2.0));
    auto const tc_max = std::max(s1_tc, s2_tc);
    auto const dt     = utils::toDt(timestep);

    auto const x = (potential_sqrt >= 0 && inner_R_R != 0)
                     ? (-inner_Q_R - std::sqrt(potential_sqrt)) / inner_R_R
                     : NULL;

    // Spheres are moving in parallel or opposite directions which are
    // orthogonal to each other
    if (x == NULL) {
      return std::nullopt;
    }

    // Collision is before one of the spheres current time, or after the
    // remaining time of the timestep
    else if (x <= 0 || x > 1 - (utils::toDt(tc_max - t_0) / dt)) {
      return std::nullopt;
    }

    // Collision is valid :)
    return tc_max + utils::toDuration(types::SecondsD(x * dt));
  }


}   // namespace dte3607::physengine::mechanics



#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_SPHERE_DETECTION_H
