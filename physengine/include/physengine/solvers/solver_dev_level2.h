#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H

#include "../utils/type_conversion.h"
#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../mechanics/compute_trajectory.h"
#include "../mechanics/sphere_vs_fixed_plane_detection.h"
#include "../mechanics/sphere_vs_fixed_plane_response.h"
#include "../bits/solver_types.h"

#include <set>

namespace dte3607::physengine::solver_dev::level2
{

  template <typename Sphere_T, typename Params_T>
  void simulateAll(Sphere_T& data, Params_T const& params)
  {
    for (auto& sphere : data) {
      auto [ds, a] = mechanics::computeLinearTrajectory(
        sphere.v, params.F, params.timestep - (sphere.t_c - params.t_0));

      sphere.p += ds;
      sphere.v += a;
      sphere.ds = ds;
      sphere.a  = a;
    }
  }

  template <typename Intersect_T, typename Plane_T, typename Sphere_T,
            typename Params_T>
  void detectCollisions(Intersect_T& data, Sphere_T& spheres, Plane_T& planes,
                        Params_T& params, uint8_t prime)
  {
    for (auto i = 0; i < spheres.size(); i++) {
      for (auto j = 0; j < planes.size(); j++) {

        auto x = mechanics::detectCollisionSphereFixedPlane(
          spheres[i].t_c, spheres[i].p, spheres[i].r, spheres[i].v, planes[j].p,
          planes[j].n, params.F, params.t_0,
          params.timestep - prime * (spheres[i].t_c - params.t_0));

        if (x.has_value()) {
          data.emplace_back(spheres[i], planes[j], x.value());
        }
      }
    }
  }

  struct compare_col_time {
    bool operator()(const solver_types::IntersectDetProcDataBlock& int1,
                    const solver_types::IntersectDetProcDataBlock& int2) const
    {
      return int1.col_tp < int2.col_tp;
    }
  };

  template <typename Intersect_T>
  std::set<solver_types::IntersectDetProcDataBlock, compare_col_time>
  sortAndReduce(Intersect_T& intersection_data)
  {
    std::set<solver_types::IntersectDetProcDataBlock, compare_col_time> sorted;

    for (auto intersect : intersection_data) {
      sorted.insert(intersect);
    }

    return sorted;
  }

  template <typename Data_T, typename Params_T>
  void simulateObject(Data_T& collision, Params_T const& params)
  {
    auto [ds, a] = mechanics::computeLinearTrajectory(
      collision.sphere.v, params.F, collision.col_tp - collision.sphere.t_c);

    collision.sphere.p += ds;
    collision.sphere.v += a;
  }

  template <typename Intersect_T, typename Params_T>
  void handleFirstCollision(Intersect_T& sorted, Params_T const& params)
  {
    auto collision = *sorted.begin();

    simulateObject(collision, params);

    auto new_v = mechanics::computeImpactResponseSphereFixedPlane(
      collision.sphere.v, collision.plane.n);

    collision.sphere.v   = new_v;
    collision.sphere.t_c = collision.col_tp;
  }

  template <concepts::SolverFixtureLevel2 Fixture_T>
  void solve([[maybe_unused]] Fixture_T&         scenario,
             [[maybe_unused]] types::NanoSeconds timestep)
  {
    solver_types::Params params;
    params.F        = scenario.m_forces;
    params.timestep = timestep;

    auto now   = types::HighResolutionClock::now();
    params.t_0 = now;
    for (auto& sphere : scenario.m_backend.m_sphere_data) {
      sphere.t_c = now;
    }

    detectCollisions(scenario.m_backend.m_intersection_data,
                     scenario.m_backend.m_sphere_data,
                     scenario.m_backend.m_fplane_data, params, 0);

    auto sortedCollisions
      = sortAndReduce(scenario.m_backend.m_intersection_data);

    while (sortedCollisions.size() > 0) {
      handleFirstCollision(sortedCollisions, params);
      scenario.m_backend.m_intersection_data.clear();
      detectCollisions(scenario.m_backend.m_intersection_data,
                       scenario.m_backend.m_sphere_data,
                       scenario.m_backend.m_fplane_data, params, 0);
      sortedCollisions = sortAndReduce(scenario.m_backend.m_intersection_data);
    }

    simulateAll(scenario.m_backend.m_sphere_data, params);

    for (auto const& [id, sphere] : scenario.m_backend.m_id_to_sphere) {
      scenario.translateParent(id, scenario.m_backend.m_sphere_data[sphere].ds);
      scenario.addAcceleration(id, scenario.m_backend.m_sphere_data[sphere].a);
    }
  }

}   // namespace dte3607::physengine::solver_dev::level2


#endif   // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
