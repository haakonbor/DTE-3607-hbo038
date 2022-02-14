#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H

#include "../utils/type_conversion.h"
#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../mechanics/compute_trajectory.h"
#include "../mechanics/sphere_vs_fixed_plane_detection.h"
#include "../mechanics/sphere_vs_fixed_plane_response.h"
#include "../mechanics/sphere_vs_sphere_detection.h"
#include "../mechanics/sphere_vs_sphere_response.h"
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

      sphere.ds += ds;
      sphere.a += a;

      sphere.p += ds;
      sphere.v += a;
    }
  }

  template <typename Intersect_T, typename Plane_T, typename Sphere_T,
            typename Params_T>
  void detectCollisions(Intersect_T& intersections, Sphere_T& spheres,
                        Plane_T& planes, Params_T& params)
  {
    for (auto i = 0; i < spheres.size(); i++) {
      for (auto j = 0; j < planes.size(); j++) {

        auto x = mechanics::detectCollisionSphereFixedPlane(
          spheres[i].t_c, spheres[i].p, spheres[i].r, spheres[i].current_ds,
          planes[j].p, planes[j].n, params.t_0, params.timestep);

        if (x.has_value()) {
          intersections.emplace_back(spheres[i], spheres[0], planes[j],
                                     x.value(), true);
        }
      }

      for (auto j = 0; j < spheres.size(); j++) {
        if (j == i) {
          continue;
        }

        auto x = mechanics::detectCollisionSphereSphere(
          spheres[i].t_c, spheres[i].p, spheres[i].r, spheres[i].current_ds,
          spheres[j].t_c, spheres[j].p, spheres[j].r, spheres[j].current_ds,
          params.t_0, params.timestep);

        if (x.has_value()) {
          intersections.emplace_back(spheres[i], spheres[j], planes[0],
                                     x.value(), false);
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
  std::multiset<solver_types::IntersectDetProcDataBlock, compare_col_time>
  sortAndReduce(Intersect_T& intersections)
  {
    std::multiset<solver_types::IntersectDetProcDataBlock, compare_col_time>
      sorted_intersections;

    for (auto intersection : intersections) {
      auto it  = sorted_intersections.begin();
      auto end = sorted_intersections.end();

      for (; it != end; it++) {
        if (intersection.fixed == false
            && (*it).sphere2.p == intersection.sphere1.p
            && (*it).sphere1.p == intersection.sphere2.p
            && (*it).col_tp == intersection.col_tp) {
          break;
        }
      }

      if (it == end) {
        sorted_intersections.insert(intersection);
      }
    }

    return sorted_intersections;
  }

  template <typename Data_T, typename Params_T>
  void simulateObject(Data_T& collision, Params_T const& params)
  {
    auto [ds1, a1] = mechanics::computeLinearTrajectory(
      collision.sphere1.v, params.F, collision.col_tp - collision.sphere1.t_c);

    collision.sphere1.ds += ds1;
    collision.sphere1.a += a1;

    collision.sphere1.p += ds1;
    collision.sphere1.v += a1;

    if (!collision.fixed) {
      auto [ds2, a2] = mechanics::computeLinearTrajectory(
        collision.sphere2.v, params.F,
        collision.col_tp - collision.sphere2.t_c);

      collision.sphere2.ds += ds2;
      collision.sphere2.a += a2;

      collision.sphere2.p += ds2;
      collision.sphere2.v += a2;
    }
  }

  template <typename SortedIntersect_T, typename Params_T>
  void handleFirstCollision(SortedIntersect_T& sorted_intersections,
                            Params_T const&    params)
  {
    auto first_col_tp = (*sorted_intersections.begin()).col_tp;

    for (auto& collision : sorted_intersections) {
      if (collision.col_tp != first_col_tp) {
        break;
      }

      simulateObject(collision, params);

      if (collision.fixed) {
        auto new_v = mechanics::computeImpactResponseSphereFixedPlane(
          collision.sphere1.v, collision.plane.n);

        collision.sphere1.v   = new_v;
        collision.sphere1.t_c = collision.col_tp;
      }
      else {
        auto new_vs = mechanics::computeImpactResponseSphereSphere(
          collision.sphere1.p, collision.sphere1.v, 1.0, collision.sphere2.p,
          collision.sphere2.v, 1.0);

        collision.sphere1.v   = new_vs.first;
        collision.sphere1.t_c = collision.col_tp;
        collision.sphere2.v   = new_vs.second;
        collision.sphere2.t_c = collision.col_tp;
      }
    }
  }

  template <typename Sphere_T, typename Params_T>
  void computeCache(Sphere_T& spheres, Params_T& params)
  {
    auto now = types::HighResolutionClock::now();

    params.t_0 = now;

    for (auto& sphere : spheres) {
      sphere.t_c        = now;
      sphere.current_ds = mechanics::computeLinearTrajectory(sphere.v, params.F,
                                                             params.timestep)
                            .first;
    }
  }

  template <concepts::SolverFixtureLevel2 Fixture_T>
  void solve([[maybe_unused]] Fixture_T&         scenario,
             [[maybe_unused]] types::NanoSeconds timestep)
  {
    solver_types::Params params;
    params.F        = scenario.m_forces;
    params.timestep = timestep;

    computeCache(scenario.m_backend.m_sphere_data, params);

    detectCollisions(scenario.m_backend.m_intersection_data,
                     scenario.m_backend.m_sphere_data,
                     scenario.m_backend.m_fplane_data, params);

    auto sortedCollisions
      = sortAndReduce(scenario.m_backend.m_intersection_data);

    while (sortedCollisions.size() > 0) {
      handleFirstCollision(sortedCollisions, params);
      scenario.m_backend.m_intersection_data.clear();
      computeCache(scenario.m_backend.m_sphere_data, params);
      detectCollisions(scenario.m_backend.m_intersection_data,
                       scenario.m_backend.m_sphere_data,
                       scenario.m_backend.m_fplane_data, params);
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
