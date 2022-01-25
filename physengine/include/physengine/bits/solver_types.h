#ifndef DTE3607_PHYSENGINE_SOLVER_TYPES_H
#define DTE3607_PHYSENGINE_SOLVER_TYPES_H

#include "types.h"

namespace dte3607::physengine::solver_types
{
  using Vector3 = types::Vector3;

  struct Params {
    Vector3                 F;          // External forces(environment)
    types::Duration         timestep;   // Time step (system)
    types::HighResolutionTP t_0;        // Start timepoint
  };

  struct CacheProcDataBlock {
    Vector3 in_p;   // Position
    Vector3 in_v;   // Velocity

    Vector3 out_a;    // Acceleration
    Vector3 out_ds;   // Trajectory

    // types::HighResolutionTP t_c;   // Current timepoint
  };

  struct SimProcDataBlock {
    Vector3 in_p;    // Position
    Vector3 in_ds;   // Trajectory

    Vector3 out_p;   // New Position
  };

  struct SphereGeomDataBlock {
    Vector3                 p;     // Position
    double                  r;     // Radius
    Vector3                 v;     // Velocity
    types::HighResolutionTP t_c;   // Current timepoint
  };

  struct InfPlaneGeomDataBlock {
    Vector3 p;   // Position
    Vector3 n;   // Normal
  };

  struct IntsecStatusDataBlock {
    bool is_collision;   // Has there been a collision
    types::HighResolutionTP
      col_tp;   // Time point in frame (t_0, t_0 + delta_t]
  };

  struct IntersectDetProcDataBlock {
    SphereGeomDataBlock   sphere;   // Sphere data
    InfPlaneGeomDataBlock plane;    // Infinite plane data
    IntsecStatusDataBlock status;   // Intersection status data
  };
}   // namespace dte3607::physengine::solver_types


#endif   // DTE3607_PHYSENGINE_FIXTURES_H
